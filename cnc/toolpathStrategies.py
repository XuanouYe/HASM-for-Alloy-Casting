from abc import ABC, abstractmethod
from typing import Any, Dict, List
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

    def cleanPolygon(self, polys: List[Any]) -> Any:
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

    def robustSection(self, mesh: trimesh.Trimesh, zValue: float, tolerance: float = 0.05) -> Any:
        sectionResult = mesh.section(plane_origin=[0.0, 0.0, zValue], plane_normal=[0.0, 0.0, 1.0])
        if sectionResult is None:
            sectionResult = mesh.section(plane_origin=[0.0, 0.0, zValue - tolerance], plane_normal=[0.0, 0.0, 1.0])
        if sectionResult is None:
            sectionResult = mesh.section(plane_origin=[0.0, 0.0, zValue + tolerance], plane_normal=[0.0, 0.0, 1.0])
        if sectionResult is None:
            return None
        slice2d, _ = sectionResult.to_2D()
        return slice2d.polygons_full

    def extractLineStrings(self, geomValue: Any) -> List[Any]:
        if geomValue is None or geomValue.is_empty:
            return []
        geomType = geomValue.geom_type
        if geomType == 'LineString':
            return [geomValue]
        if geomType == 'MultiLineString':
            return [lineItem for lineItem in geomValue.geoms if lineItem.geom_type == 'LineString']
        if geomType == 'GeometryCollection':
            return [lineItem for lineItem in geomValue.geoms if lineItem.geom_type == 'LineString']
        return []


class ZLevelRoughingStrategy(IToolpathStrategy):
    def generate(self, targetMesh: trimesh.Trimesh, keepOutMesh: trimesh.Trimesh, toolRadius: float, params: Dict[str, Any], safetyMargin: float) -> List[np.ndarray]:
        if targetMesh.is_empty:
            return []
        stepOver = float(params.get('stepOver', 1.0))
        layerStep = float(params.get('layerStep', 1.0))
        boundsArray = np.asarray(targetMesh.bounds, dtype=float)
        zMin = float(boundsArray[0, 2])
        zMax = float(boundsArray[1, 2])
        zLevels = np.arange(zMax - layerStep, zMin, -layerStep, dtype=float)
        allPaths = []
        for zValue in zLevels:
            targetPolys = self.robustSection(targetMesh, zValue)
            if not targetPolys:
                continue
            targetSlice = targetMesh.section(plane_origin=[0.0, 0.0, zValue], plane_normal=[0.0, 0.0, 1.0])
            if targetSlice is None:
                continue
            _, to3dMat = targetSlice.to_2D()
            machinablePoly = self.cleanPolygon(targetPolys).buffer(-toolRadius)
            keepOutPoly = MultiPolygon()
            if keepOutMesh is not None and not keepOutMesh.is_empty:
                keepOutPolys = self.robustSection(keepOutMesh, zValue)
                if keepOutPolys:
                    keepOutPoly = self.cleanPolygon(keepOutPolys).buffer(toolRadius + safetyMargin + stepOver)
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
                    lineItems = self.extractLineStrings(interResult)
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
        if keepOutMesh is not None and not keepOutMesh.is_empty:
            combinedMesh = trimesh.util.concatenate([targetMesh, keepOutMesh])
        else:
            combinedMesh = targetMesh
        boundsArray = np.asarray(targetMesh.bounds, dtype=float)
        stepOver = float(params.get('stepOver', 1.0))
        safeHeight = float(params.get('safeHeight', 5.0))
        angleThreshold = float(params.get('angleThreshold', 1.047))
        minNormalZ = float(np.cos(angleThreshold))
        xMin = float(boundsArray[0, 0])
        yMin = float(boundsArray[0, 1])
        xMax = float(boundsArray[1, 0])
        yMax = float(boundsArray[1, 1])
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
                triNormal = combinedMesh.face_normals[triIndex]
                if triNormal[2] >= minNormalZ:
                    linePoints.append([xList[rayIndex], yValue, float(locations[bestHit, 2]) + toolRadius])
            if reverseFlag:
                linePoints.reverse()
            reverseFlag = not reverseFlag
            if len(linePoints) >= 2:
                allPaths.append(np.asarray(linePoints, dtype=float))
        return allPaths


class SurfaceProjectionFinishingStrategy(IToolpathStrategy):
    def isBallCollisionFree(self, centerPoint: np.ndarray, contactPoint: np.ndarray, targetTree: cKDTree, targetSamples: np.ndarray, keepOutTree: cKDTree, toolRadius: float, safetyMargin: float, collisionClearance: float, contactPatchRadius: float, localAllowance: float) -> bool:
        if keepOutTree is not None:
            keepOutDist, _ = keepOutTree.query(centerPoint, k=1)
            if np.isfinite(keepOutDist) and keepOutDist < toolRadius + safetyMargin:
                return False
        if targetTree is None or len(targetSamples) == 0:
            return True
        sphereRadius = max(toolRadius - collisionClearance, toolRadius * 0.75)
        nearIndices = targetTree.query_ball_point(centerPoint, r=sphereRadius + localAllowance)
        if not nearIndices:
            return True
        for sampleIndex in nearIndices:
            samplePoint = targetSamples[sampleIndex]
            radialDist = float(np.linalg.norm(samplePoint[:2] - centerPoint[:2]))
            if radialDist >= sphereRadius:
                continue
            sphereLowerZ = centerPoint[2] - np.sqrt(max(sphereRadius * sphereRadius - radialDist * radialDist, 0.0))
            if samplePoint[2] > sphereLowerZ + localAllowance:
                if float(np.linalg.norm(samplePoint - contactPoint)) > contactPatchRadius:
                    return False
        return True

    def generateSingleRaster(self, targetMesh: trimesh.Trimesh, keepOutTree: cKDTree, targetTree: cKDTree, targetSamples: np.ndarray, toolRadius: float, params: Dict[str, Any], safetyMargin: float, fixedValue: float, scanAxis: str, reverseFlag: bool) -> List[np.ndarray]:
        boundsArray = np.asarray(targetMesh.bounds, dtype=float)
        projectionStep = float(params.get('projectionStep', 0.35))
        safeHeight = float(params.get('safeHeight', 5.0))
        finishStock = float(params.get('finishStock', 0.03))
        finishNormalAngleDeg = float(params.get('finishNormalAngleDeg', 88.0))
        collisionClearance = float(params.get('collisionClearance', max(0.08 * toolRadius, 0.03)))
        contactPatchRadius = float(params.get('contactPatchRadius', max(0.75 * toolRadius, projectionStep * 2.4)))
        lineGapTolerance = float(params.get('lineGapTolerance', projectionStep * 3.0))
        localAllowance = float(params.get('localAllowance', 0.05))
        minVisibleNormalZ = float(np.cos(np.deg2rad(finishNormalAngleDeg)))
        zStart = float(boundsArray[1, 2] + safeHeight + toolRadius + finishStock)
        if scanAxis == 'x':
            scanValues = np.arange(boundsArray[0, 0], boundsArray[1, 0] + projectionStep * 0.5, projectionStep, dtype=float)
            rayOrigins = np.column_stack([scanValues, np.full(len(scanValues), fixedValue), np.full(len(scanValues), zStart)])
        else:
            scanValues = np.arange(boundsArray[0, 1], boundsArray[1, 1] + projectionStep * 0.5, projectionStep, dtype=float)
            rayOrigins = np.column_stack([np.full(len(scanValues), fixedValue), scanValues, np.full(len(scanValues), zStart)])
        rayDirs = np.tile([0.0, 0.0, -1.0], (len(rayOrigins), 1))
        locations, indexRay, indexTri = targetMesh.ray.intersects_location(ray_origins=rayOrigins, ray_directions=rayDirs)
        lineCenters = []
        for rayIndex in range(len(rayOrigins)):
            hitIndices = np.where(indexRay == rayIndex)[0]
            if len(hitIndices) == 0:
                continue
            bestHit = hitIndices[np.argmax(locations[hitIndices, 2])]
            triIndex = int(indexTri[bestHit])
            triNormal = targetMesh.face_normals[triIndex]
            if triNormal[2] < minVisibleNormalZ:
                continue
            contactPoint = np.asarray(locations[bestHit], dtype=float)
            centerPoint = contactPoint + np.array([0.0, 0.0, toolRadius + finishStock], dtype=float)
            if not self.isBallCollisionFree(centerPoint, contactPoint, targetTree, targetSamples, keepOutTree, toolRadius, safetyMargin, collisionClearance, contactPatchRadius, localAllowance):
                continue
            lineCenters.append(centerPoint)
        if reverseFlag:
            lineCenters.reverse()
        if len(lineCenters) < 2:
            return []
        lineArray = densifyPolyline(np.asarray(lineCenters, dtype=float), projectionStep)
        return splitPolylineByGap(lineArray, max(lineGapTolerance, projectionStep * 2.5))

    def generate(self, targetMesh: trimesh.Trimesh, keepOutMesh: trimesh.Trimesh, toolRadius: float, params: Dict[str, Any], safetyMargin: float) -> List[np.ndarray]:
        if targetMesh.is_empty:
            return []
        scanMode = str(params.get('scanMode', 'xy')).lower()
        stepOver = float(params.get('stepOver', 0.8))
        collisionSampleCount = int(params.get('collisionSampleCount', 18000))
        keepOutSampleCount = int(params.get('keepOutSampleCount', 6000))
        targetSamples, _ = sampleMeshPointsWithNormals(targetMesh, collisionSampleCount)
        keepOutSamples, _ = sampleMeshPointsWithNormals(keepOutMesh, keepOutSampleCount)
        targetTree = cKDTree(targetSamples) if len(targetSamples) > 0 else None
        keepOutTree = cKDTree(keepOutSamples) if len(keepOutSamples) > 0 else None
        boundsArray = np.asarray(targetMesh.bounds, dtype=float)
        reverseFlag = False
        allPaths = []
        scanAxes = ['x', 'y'] if scanMode == 'xy' else [scanMode]
        for scanAxis in scanAxes:
            if scanAxis == 'x':
                fixedValues = np.arange(boundsArray[0, 1], boundsArray[1, 1] + stepOver * 0.5, stepOver, dtype=float)
            else:
                fixedValues = np.arange(boundsArray[0, 0], boundsArray[1, 0] + stepOver * 0.5, stepOver, dtype=float)
            for fixedValue in fixedValues:
                subPaths = self.generateSingleRaster(targetMesh, keepOutTree, targetTree, targetSamples, toolRadius, params, safetyMargin, fixedValue, scanAxis, reverseFlag)
                reverseFlag = not reverseFlag
                allPaths.extend(subPaths)
        return allPaths


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
