from abc import ABC, abstractmethod
from typing import Any, Dict, List, Tuple
import numpy as np
import trimesh
from scipy.spatial import cKDTree
from shapely.geometry import LineString, MultiPolygon
from shapely.ops import unary_union
from shapely.validation import make_valid
from .geometryUtils import densifyPolyline, sampleMeshPointsWithNormals, splitPolylineByGap


class IToolpathStrategy(ABC):
    @abstractmethod
    def generate(self, targetMesh: trimesh.Trimesh, keepOutMesh: trimesh.Trimesh, toolRadius: float, params: Dict[str, Any], safetyMargin: float) -> List[np.ndarray]:
        pass


def cleanPolygon(polys: List[Any]) -> Any:
    cleanPolys = []
    for polyItem in polys:
        fixedPoly = polyItem
        if not fixedPoly.is_valid:
            fixedPoly = make_valid(fixedPoly)
        fixedPoly = fixedPoly.buffer(0)
        if not fixedPoly.is_empty:
            cleanPolys.append(fixedPoly)
    if not cleanPolys:
        return MultiPolygon()
    return unary_union(cleanPolys)


def robustSection(mesh: trimesh.Trimesh, zValue: float, tolerance: float = 0.05) -> Any:
    sectionResult = mesh.section(plane_origin=[0.0, 0.0, zValue], plane_normal=[0.0, 0.0, 1.0])
    if sectionResult is None:
        sectionResult = mesh.section(plane_origin=[0.0, 0.0, zValue - tolerance], plane_normal=[0.0, 0.0, 1.0])
    if sectionResult is None:
        sectionResult = mesh.section(plane_origin=[0.0, 0.0, zValue + tolerance], plane_normal=[0.0, 0.0, 1.0])
    if sectionResult is None:
        return None
    slice2d, _ = sectionResult.to_2D()
    return slice2d.polygons_full


def extractLineStrings(geomValue: Any) -> List[Any]:
    if geomValue is None or geomValue.is_empty:
        return []
    geomType = geomValue.geom_type
    if geomType == 'LineString':
        return [geomValue]
    if geomType == 'MultiLineString':
        return [l for l in geomValue.geoms if l.geom_type == 'LineString']
    if geomType == 'GeometryCollection':
        return [l for l in geomValue.geoms if l.geom_type == 'LineString']
    return []


class ZLevelRoughingStrategy(IToolpathStrategy):
    def generate(self, targetMesh: trimesh.Trimesh, keepOutMesh: trimesh.Trimesh, toolRadius: float, params: Dict[str, Any], safetyMargin: float) -> List[np.ndarray]:
        if targetMesh.is_empty:
            return []
        stepOver = float(params.get('stepOver', 1.0))
        layerStep = float(params.get('layerStep', 1.0))
        roughStock = float(params.get('roughStock', 0.0))
        boundsArray = np.asarray(targetMesh.bounds, dtype=float)
        zMin = float(boundsArray[0, 2])
        zMax = float(boundsArray[1, 2])
        zLevels = np.arange(zMax - layerStep, zMin, -layerStep, dtype=float)
        allPaths = []
        for zValue in zLevels:
            targetPolys = robustSection(targetMesh, zValue)
            if not targetPolys:
                continue
            targetSlice = targetMesh.section(plane_origin=[0.0, 0.0, zValue], plane_normal=[0.0, 0.0, 1.0])
            if targetSlice is None:
                continue
            _, to3dMat = targetSlice.to_2D()
            machinablePoly = cleanPolygon(targetPolys).buffer(-(toolRadius + roughStock))
            keepOutPoly = MultiPolygon()
            if keepOutMesh is not None and not keepOutMesh.is_empty:
                keepOutPolys = robustSection(keepOutMesh, zValue)
                if keepOutPolys:
                    keepOutPoly = cleanPolygon(keepOutPolys).buffer(toolRadius + safetyMargin + roughStock)
            if machinablePoly.is_empty:
                continue
            safePoly = machinablePoly.difference(keepOutPoly.buffer(0))
            if safePoly.is_empty:
                continue
            polyGeoms = list(safePoly.geoms) if safePoly.geom_type == 'MultiPolygon' else [safePoly]
            for polyItem in polyGeoms:
                if polyItem.is_empty:
                    continue
                minX, minY, maxX, maxY = polyItem.bounds
                yList = np.arange(minY, maxY + stepOver * 0.5, stepOver, dtype=float)
                reverseFlag = False
                for yValue in yList:
                    scanLine = LineString([(minX - 1.0, yValue), (maxX + 1.0, yValue)])
                    interResult = scanLine.intersection(polyItem)
                    lineItems = extractLineStrings(interResult)
                    for lineItem in lineItems:
                        coordArray = np.asarray(lineItem.coords, dtype=float)
                        if len(coordArray) < 2:
                            continue
                        if reverseFlag:
                            coordArray = coordArray[::-1]
                        coordHomo = np.column_stack([coordArray, np.zeros(len(coordArray)), np.ones(len(coordArray))])
                        path3d = (to3dMat @ coordHomo.T).T[:, :3]
                        if len(path3d) >= 2:
                            allPaths.append(path3d)
                    reverseFlag = not reverseFlag
        return allPaths


class DropRasterStrategy(IToolpathStrategy):
    def generate(self, targetMesh: trimesh.Trimesh, keepOutMesh: trimesh.Trimesh, toolRadius: float, params: Dict[str, Any], safetyMargin: float) -> List[np.ndarray]:
        if targetMesh.is_empty:
            return []
        combinedMesh = trimesh.util.concatenate([targetMesh, keepOutMesh]) if keepOutMesh is not None and not keepOutMesh.is_empty else targetMesh
        boundsArray = np.asarray(targetMesh.bounds, dtype=float)
        stepOver = float(params.get('stepOver', 1.0))
        safeHeight = float(params.get('safeHeight', 5.0))
        angleThreshold = float(params.get('angleThreshold', 1.047))
        minNormalZ = float(np.cos(angleThreshold))
        xMin, yMin = float(boundsArray[0, 0]), float(boundsArray[0, 1])
        xMax, yMax = float(boundsArray[1, 0]), float(boundsArray[1, 1])
        zMax = float(boundsArray[1, 2])
        xList = np.arange(xMin, xMax + stepOver * 0.5, stepOver, dtype=float)
        yList = np.arange(yMin, yMax + stepOver * 0.5, stepOver, dtype=float)
        zStart = zMax + safeHeight + toolRadius
        allPaths = []
        reverseFlag = False
        for yValue in yList:
            rayOrigins = np.column_stack([xList, np.full(len(xList), yValue), np.full(len(xList), zStart)])
            rayDirs = np.tile([0.0, 0.0, -1.0], (len(xList), 1))
            locations, indexRay, indexTri = combinedMesh.ray.intersects_location(ray_origins=rayOrigins, ray_directions=rayDirs)
            linePoints = []
            for rayIndex in range(len(xList)):
                hitIndices = np.where(indexRay == rayIndex)[0]
                if len(hitIndices) == 0:
                    continue
                bestHit = hitIndices[np.argmax(locations[hitIndices, 2])]
                triIndex = int(indexTri[bestHit])
                if combinedMesh.face_normals[triIndex][2] >= minNormalZ:
                    linePoints.append([xList[rayIndex], yValue, float(locations[bestHit, 2]) + toolRadius])
            if reverseFlag:
                linePoints.reverse()
            reverseFlag = not reverseFlag
            if len(linePoints) >= 2:
                allPaths.append(np.asarray(linePoints, dtype=float))
        return allPaths


class SurfaceProjectionFinishingStrategy(IToolpathStrategy):
    def isBallCollisionFree(self, centerPoint: np.ndarray, contactPoint: np.ndarray, contactNormal: np.ndarray, targetTree: cKDTree, targetSamples: np.ndarray, keepOutTree: cKDTree, toolRadius: float, safetyMargin: float, collisionClearance: float, contactPatchRadius: float, localAllowance: float) -> bool:
        if keepOutTree is not None:
            keepOutDist, _ = keepOutTree.query(centerPoint, k=1)
            if np.isfinite(keepOutDist) and keepOutDist < toolRadius + safetyMargin:
                return False
        if targetTree is None or len(targetSamples) == 0:
            return True
        sphereRadius = max(toolRadius - collisionClearance, toolRadius * 0.78)
        nearIndices = targetTree.query_ball_point(centerPoint, r=sphereRadius + localAllowance)
        if not nearIndices:
            return True
        for sampleIndex in nearIndices:
            samplePoint = targetSamples[sampleIndex]
            centerDist = float(np.linalg.norm(samplePoint - centerPoint))
            if centerDist >= sphereRadius + localAllowance:
                continue
            deltaVec = samplePoint - contactPoint
            normalOffset = float(np.dot(deltaVec, contactNormal))
            tangentVec = deltaVec - normalOffset * contactNormal
            tangentDist = float(np.linalg.norm(tangentVec))
            if tangentDist > contactPatchRadius and normalOffset > -localAllowance:
                return False
        return True

    def sampleReachableCenters(self, targetMesh: trimesh.Trimesh, keepOutMesh: trimesh.Trimesh, toolRadius: float, params: Dict[str, Any], safetyMargin: float) -> Tuple[np.ndarray, np.ndarray]:
        finishStock = float(params.get('finishStock', 0.03))
        finishNormalAngleDeg = float(params.get('finishNormalAngleDeg', 95.0))
        collisionClearance = float(params.get('collisionClearance', max(0.05 * toolRadius, 0.02)))
        contactPatchRadius = float(params.get('contactPatchRadius', max(0.85 * toolRadius, 0.8)))
        localAllowance = float(params.get('localAllowance', 0.06))
        surfaceSampleCount = int(params.get('surfaceSampleCount', 36000))
        keepOutSampleCount = int(params.get('keepOutSampleCount', 8000))
        minVisibleNormalZ = float(np.cos(np.deg2rad(finishNormalAngleDeg)))
        targetSamples, targetNormals = sampleMeshPointsWithNormals(targetMesh, surfaceSampleCount)
        if len(targetSamples) == 0:
            return np.empty((0, 3), dtype=float), np.empty((0, 3), dtype=float)
        keepOutTree = None
        if keepOutMesh is not None and not keepOutMesh.is_empty:
            keepOutSamples, _ = sampleMeshPointsWithNormals(keepOutMesh, keepOutSampleCount)
            if len(keepOutSamples) > 0:
                keepOutTree = cKDTree(keepOutSamples)
        targetTree = cKDTree(targetSamples)
        visibleMask = targetNormals[:, 2] >= minVisibleNormalZ
        candidateContacts = np.asarray(targetSamples[visibleMask], dtype=float)
        candidateNormals = np.asarray(targetNormals[visibleMask], dtype=float)
        if len(candidateContacts) == 0:
            return np.empty((0, 3), dtype=float), np.empty((0, 3), dtype=float)
        candidateCenters = candidateContacts + candidateNormals * (toolRadius + finishStock)
        validContacts = []
        validCenters = []
        for pointIndex in range(len(candidateCenters)):
            centerPoint = candidateCenters[pointIndex]
            contactPoint = candidateContacts[pointIndex]
            contactNormal = candidateNormals[pointIndex]
            if self.isBallCollisionFree(centerPoint, contactPoint, contactNormal, targetTree, targetSamples, keepOutTree, toolRadius, safetyMargin, collisionClearance, contactPatchRadius, localAllowance):
                validContacts.append(contactPoint)
                validCenters.append(centerPoint)
        if not validCenters:
            return np.empty((0, 3), dtype=float), np.empty((0, 3), dtype=float)
        return np.asarray(validContacts, dtype=float), np.asarray(validCenters, dtype=float)

    def reduceOrderedCenters(self, centerPoints: np.ndarray, projectionStep: float) -> np.ndarray:
        if len(centerPoints) <= 2:
            return np.asarray(centerPoints, dtype=float)
        reducedPoints = [centerPoints[0]]
        lastPoint = centerPoints[0]
        minKeepDist = max(projectionStep * 0.65, 1e-6)
        for pointItem in centerPoints[1:]:
            if float(np.linalg.norm(pointItem - lastPoint)) >= minKeepDist:
                reducedPoints.append(pointItem)
                lastPoint = pointItem
        if len(reducedPoints) == 1:
            reducedPoints.append(centerPoints[-1])
        return np.asarray(reducedPoints, dtype=float)

    def buildStripePaths(self, contactPoints: np.ndarray, centerPoints: np.ndarray, boundsArray: np.ndarray, scanAxis: str, stepOver: float, projectionStep: float, lineGapTolerance: float, keepOutChecker: Any, toolpathEngine: Any, hmSampleStep: float) -> List[np.ndarray]:
        if len(contactPoints) == 0 or len(centerPoints) == 0:
            return []
        fixedIndex = 1 if scanAxis == 'x' else 0
        travelIndex = 0 if scanAxis == 'x' else 1
        bandHalfWidth = max(stepOver * 0.55, projectionStep * 1.4)
        fixedValues = np.arange(boundsArray[0, fixedIndex], boundsArray[1, fixedIndex] + stepOver * 0.5, stepOver, dtype=float)
        reverseFlag = False
        allPaths = []
        for fixedValue in fixedValues:
            bandMask = np.abs(contactPoints[:, fixedIndex] - fixedValue) <= bandHalfWidth
            if int(np.count_nonzero(bandMask)) < 2:
                continue
            stripeContacts = np.asarray(contactPoints[bandMask], dtype=float)
            stripeCenters = np.asarray(centerPoints[bandMask], dtype=float)
            order = np.argsort(stripeContacts[:, travelIndex])
            stripeContacts = stripeContacts[order]
            stripeCenters = stripeCenters[order]
            subPaths = []
            currentStripe = [stripeCenters[0]]
            for pointIndex in range(1, len(stripeCenters)):
                travelGap = abs(float(stripeContacts[pointIndex, travelIndex] - stripeContacts[pointIndex - 1, travelIndex]))
                spatialGap = float(np.linalg.norm(stripeCenters[pointIndex] - stripeCenters[pointIndex - 1]))
                if travelGap > lineGapTolerance or spatialGap > lineGapTolerance * 1.5:
                    if len(currentStripe) >= 2:
                        subPaths.append(np.asarray(currentStripe, dtype=float))
                    currentStripe = [stripeCenters[pointIndex]]
                else:
                    currentStripe.append(stripeCenters[pointIndex])
            if len(currentStripe) >= 2:
                subPaths.append(np.asarray(currentStripe, dtype=float))
            for subPath in subPaths:
                reducedPath = self.reduceOrderedCenters(subPath, projectionStep)
                if len(reducedPath) < 2:
                    continue
                densePath = densifyPolyline(reducedPath, projectionStep)
                denseSubPaths = splitPolylineByGap(densePath, max(lineGapTolerance, projectionStep * 3.0))
                for denseSubPath in denseSubPaths:
                    if len(denseSubPath) < 2:
                        continue
                    finalPath = np.asarray(denseSubPath, dtype=float)
                    if reverseFlag:
                        finalPath = finalPath[::-1]
                    if keepOutChecker is not None and toolpathEngine is not None and not keepOutChecker.isEmpty:
                        clipped = toolpathEngine.clipPathsByCollisionChecker([finalPath], keepOutChecker, hmSampleStep)
                        for cp in clipped:
                            if len(cp) >= 2:
                                allPaths.append(cp)
                    else:
                        allPaths.append(finalPath)
            reverseFlag = not reverseFlag
        return allPaths

    def generate(self, targetMesh: trimesh.Trimesh, keepOutMesh: trimesh.Trimesh, toolRadius: float, params: Dict[str, Any], safetyMargin: float) -> List[np.ndarray]:
        if targetMesh.is_empty:
            return []
        scanAxis = str(params.get('scanAxis', 'x')).lower()
        if scanAxis not in {'x', 'y'}:
            scanAxis = 'x'
        stepOver = float(params.get('stepOver', 0.45))
        projectionStep = float(params.get('projectionStep', 0.25))
        lineGapTolerance = float(params.get('lineGapTolerance', max(stepOver * 2.0, projectionStep * 4.0)))
        keepOutChecker = params.get('_keepOutChecker', None)
        toolpathEngine = params.get('_toolpathEngine', None)
        hmSampleStep = float(params.get('_hmSampleStep', projectionStep))
        contactPoints, centerPoints = self.sampleReachableCenters(targetMesh, keepOutMesh, toolRadius, params, safetyMargin)
        if len(centerPoints) == 0:
            return []
        boundsArray = np.asarray(targetMesh.bounds, dtype=float)
        return self.buildStripePaths(contactPoints, centerPoints, boundsArray, scanAxis, stepOver, projectionStep, lineGapTolerance, keepOutChecker, toolpathEngine, hmSampleStep)


class ToolpathStrategyFactory:
    strategies = {
        'zlevelroughing': ZLevelRoughingStrategy,
        'zlr': ZLevelRoughingStrategy,
        'dropraster': DropRasterStrategy,
        'surfacefinishing': SurfaceProjectionFinishingStrategy,
        'spf': SurfaceProjectionFinishingStrategy
    }

    @classmethod
    def getStrategy(cls, mode: str) -> IToolpathStrategy:
        return cls.strategies.get(str(mode).lower(), DropRasterStrategy)()
