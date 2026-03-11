from typing import Any, Dict, List, Optional, Tuple
import numpy as np
from scipy.spatial import cKDTree
import trimesh
from .geometryUtils import applyRotation

try:
    import fcl
    _FCL_AVAILABLE = True
except ImportError:
    _FCL_AVAILABLE = False


def _buildFclMesh(mesh: trimesh.Trimesh):
    if not _FCL_AVAILABLE or mesh is None or mesh.is_empty:
        return None
    verts = np.asarray(mesh.vertices, dtype=np.float64)
    faces = np.asarray(mesh.faces, dtype=np.int32)
    bvhModel = fcl.BVHModel()
    bvhModel.beginModel(len(faces), len(verts))
    bvhModel.addSubModel(verts, faces)
    bvhModel.endModel()
    return fcl.CollisionObject(bvhModel, fcl.Transform3f())


class SafeEnvelope:
    def __init__(self, obstacleMesh: trimesh.Trimesh, toolRadius: float, safetyMargin: float):
        self.mesh = obstacleMesh
        self.isEmpty = obstacleMesh is None or obstacleMesh.is_empty
        self.clearance = float(toolRadius + safetyMargin)
        self.safeZ = float(-np.inf)
        self._surfaceTree = None
        self._surfaceSamples = None
        if not self.isEmpty:
            bounds = np.asarray(obstacleMesh.bounds, dtype=float)
            self.safeZ = float(bounds[1, 2]) + self.clearance + float(toolRadius) + 1.0
            try:
                pts, _ = trimesh.sample.sample_surface(obstacleMesh, 40000)
                self._surfaceSamples = np.asarray(pts, dtype=float)
                self._surfaceTree = cKDTree(self._surfaceSamples)
            except Exception:
                pass

    def _nearestSurfaceDist(self, pts: np.ndarray) -> np.ndarray:
        if self._surfaceTree is None or len(pts) == 0:
            return np.full(len(pts), np.inf, dtype=float)
        dists, _ = self._surfaceTree.query(pts, k=1)
        return np.asarray(dists, dtype=float)

    def isPointSafe(self, pt: np.ndarray) -> bool:
        if self.isEmpty:
            return True
        ptArr = np.asarray(pt, dtype=float).reshape(1, 3)
        d = self._nearestSurfaceDist(ptArr)[0]
        return float(d) >= self.clearance

    def isSegmentSafe(self, startPt: np.ndarray, endPt: np.ndarray, sampleStep: float = 0.5) -> bool:
        if self.isEmpty:
            return True
        s = np.asarray(startPt, dtype=float)
        e = np.asarray(endPt, dtype=float)
        segLen = float(np.linalg.norm(e - s))
        if segLen < 1e-9:
            return self.isPointSafe(s)
        nSamples = max(int(np.ceil(segLen / sampleStep)) + 1, 2)
        tVals = np.linspace(0.0, 1.0, nSamples)
        samplePts = np.outer(1.0 - tVals, s) + np.outer(tVals, e)
        dists = self._nearestSurfaceDist(samplePts)
        return bool(np.all(dists >= self.clearance))

    def computeSafeRetractZ(self, pt: np.ndarray, extraClearance: float = 2.0) -> float:
        if self.isEmpty:
            return float(pt[2]) + extraClearance
        return float(self.safeZ) + extraClearance

    def buildSafeTransit(self, fromPt: np.ndarray, toPt: np.ndarray, sampleStep: float = 0.5, extraClearance: float = 2.0) -> List[np.ndarray]:
        if self.isEmpty:
            return [fromPt, toPt]
        if self.isSegmentSafe(fromPt, toPt, sampleStep):
            return [fromPt, toPt]
        retractZ = self.computeSafeRetractZ(fromPt, extraClearance)
        retractZ = max(retractZ, float(fromPt[2]) + extraClearance, float(toPt[2]) + extraClearance)
        r1 = fromPt.copy()
        r1[2] = retractZ
        r2 = toPt.copy()
        r2[2] = retractZ
        r1Safe = self.isSegmentSafe(fromPt, r1, sampleStep)
        r2Safe = self.isSegmentSafe(r2, toPt, sampleStep)
        midSafe = self.isSegmentSafe(r1, r2, sampleStep)
        if r1Safe and midSafe and r2Safe:
            return [fromPt, r1, r2, toPt]
        higherZ = retractZ + self.clearance * 2.0 + extraClearance
        h1 = fromPt.copy()
        h1[2] = higherZ
        h2 = toPt.copy()
        h2[2] = higherZ
        return [fromPt, h1, h2, toPt]


class MeshCollisionChecker:
    def __init__(self, obstacleMesh: trimesh.Trimesh, toolRadius: float, safetyMargin: float):
        self.obstacleMesh = obstacleMesh
        self.clearanceDist = float(toolRadius + safetyMargin)
        self.isEmpty = obstacleMesh is None or obstacleMesh.is_empty
        self._envelope = SafeEnvelope(obstacleMesh, toolRadius, safetyMargin)

    def isPointColliding(self, point: np.ndarray) -> bool:
        if self.isEmpty:
            return False
        return not self._envelope.isPointSafe(np.asarray(point, dtype=float))

    def batchPointsColliding(self, points: np.ndarray) -> np.ndarray:
        if self.isEmpty or len(points) == 0:
            return np.zeros(len(points), dtype=bool)
        pts = np.asarray(points, dtype=float)
        dists = self._envelope._nearestSurfaceDist(pts)
        collidingMask = dists < self.clearanceDist
        return collidingMask

    def isSegmentColliding(self, startPoint: np.ndarray, endPoint: np.ndarray, sampleStep: float) -> bool:
        if self.isEmpty:
            return False
        return not self._envelope.isSegmentSafe(
            np.asarray(startPoint, dtype=float),
            np.asarray(endPoint, dtype=float),
            sampleStep
        )


class PointCloudIPW:
    def __init__(self, mesh: trimesh.Trimesh, sampleCount: int = 50000):
        try:
            self.points, _ = trimesh.sample.sample_surface(mesh, sampleCount)
        except Exception:
            self.points = np.zeros((0, 3), dtype=float)
        self.activeMask = np.ones(len(self.points), dtype=bool)

    def getActivePointsWcs(self) -> np.ndarray:
        return self.points[self.activeMask]

    def filterPathsLocal(self, pathsLocal: List[np.ndarray], rotToToolFrame: np.ndarray, toolRadius: float, stepOver: float) -> List[np.ndarray]:
        if not pathsLocal or not np.any(self.activeMask):
            return []
        activePointsWcs = self.getActivePointsWcs()
        if len(activePointsWcs) == 0:
            return []
        activePointsLocal = applyRotation(activePointsWcs, rotToToolFrame)
        tree = cKDTree(activePointsLocal)
        validPaths = []
        searchRadius = toolRadius + stepOver * 1.5
        for pathLocal in pathsLocal:
            distances, _ = tree.query(pathLocal, k=1, distance_upper_bound=searchRadius)
            if np.any(distances != np.inf):
                validPaths.append(np.asarray(pathLocal, dtype=float))
        return validPaths

    def updateIpwLocal(self, pathsLocal: List[np.ndarray], rotToToolFrame: np.ndarray, toolRadius: float) -> None:
        if not pathsLocal or not np.any(self.activeMask):
            return
        activePointsWcs = self.getActivePointsWcs()
        activeIndices = np.where(self.activeMask)[0]
        activePointsLocal = applyRotation(activePointsWcs, rotToToolFrame)
        tree = cKDTree(activePointsLocal[:, :2])
        removeLocalIndices = set()
        for pathLocal in pathsLocal:
            ballIndices = tree.query_ball_point(pathLocal[:, :2], r=toolRadius)
            for pointIndex, indexList in enumerate(ballIndices):
                pathZ = float(pathLocal[pointIndex, 2])
                for localIndex in indexList:
                    if activePointsLocal[localIndex, 2] >= pathZ - toolRadius * 0.5:
                        removeLocalIndices.add(localIndex)
        if removeLocalIndices:
            globalIndices = activeIndices[list(removeLocalIndices)]
            self.activeMask[globalIndices] = False


class TrimeshToolpathEngine:
    def buildCollisionChecker(self, obstacleMesh: trimesh.Trimesh, toolRadius: float, safetyMargin: float) -> MeshCollisionChecker:
        return MeshCollisionChecker(obstacleMesh, toolRadius, safetyMargin)

    def buildSafeEnvelope(self, obstacleMesh: trimesh.Trimesh, toolRadius: float, safetyMargin: float) -> SafeEnvelope:
        return SafeEnvelope(obstacleMesh, toolRadius, safetyMargin)

    def buildObstacleHeightMapLocal(self, obstacleMesh: trimesh.Trimesh, gridStep: float, toolRadius: float, safetyMargin: float) -> Optional[Dict[str, Any]]:
        if obstacleMesh is None or obstacleMesh.is_empty:
            return None
        marginVal = float(toolRadius + safetyMargin)
        bounds = np.asarray(obstacleMesh.bounds, dtype=float)
        xGrid = np.arange(bounds[0][0] - marginVal - gridStep, bounds[1][0] + marginVal + gridStep, gridStep, dtype=float)
        yGrid = np.arange(bounds[0][1] - marginVal - gridStep, bounds[1][1] + marginVal + gridStep, gridStep, dtype=float)
        if len(xGrid) == 0 or len(yGrid) == 0:
            return None
        zStart = float(bounds[1][2] + marginVal + gridStep)
        zLimit = np.full((len(yGrid), len(xGrid)), -np.inf, dtype=float)
        for rowIndex, yVal in enumerate(yGrid):
            rayOrigins = np.column_stack((xGrid, np.full(len(xGrid), yVal), np.full(len(xGrid), zStart)))
            rayDirections = np.tile([0.0, 0.0, -1.0], (len(xGrid), 1))
            locations, rayIndices, _ = obstacleMesh.ray.intersects_location(ray_origins=rayOrigins, ray_directions=rayDirections)
            if len(locations) == 0:
                continue
            for colIndex in range(len(xGrid)):
                hitMask = np.where(rayIndices == colIndex)[0]
                if len(hitMask) == 0:
                    continue
                zLimit[rowIndex, colIndex] = float(np.max(locations[hitMask, 2]) + marginVal)
        finiteMask = np.isfinite(zLimit)
        maxLimit = float(np.max(zLimit[finiteMask])) if np.any(finiteMask) else -np.inf
        return {'xGrid': xGrid, 'yGrid': yGrid, 'zLimit': zLimit, 'gridStep': float(gridStep), 'maxLimit': maxLimit}

    def queryHeightLimitLocal(self, heightMapLocal: Optional[Dict[str, Any]], xVal: float, yVal: float) -> float:
        if heightMapLocal is None:
            return -np.inf
        xGrid = heightMapLocal['xGrid']
        yGrid = heightMapLocal['yGrid']
        zLimit = heightMapLocal['zLimit']
        if len(xGrid) == 1 or len(yGrid) == 1:
            return float(zLimit[0, 0])
        if xVal < xGrid[0] or xVal > xGrid[-1] or yVal < yGrid[0] or yVal > yGrid[-1]:
            return -np.inf
        xIndex = int(np.clip(np.searchsorted(xGrid, xVal), 1, len(xGrid) - 1))
        yIndex = int(np.clip(np.searchsorted(yGrid, yVal), 1, len(yGrid) - 1))
        x0, x1 = float(xGrid[xIndex - 1]), float(xGrid[xIndex])
        y0, y1 = float(yGrid[yIndex - 1]), float(yGrid[yIndex])
        q11 = float(zLimit[yIndex - 1, xIndex - 1])
        q21 = float(zLimit[yIndex - 1, xIndex])
        q12 = float(zLimit[yIndex, xIndex - 1])
        q22 = float(zLimit[yIndex, xIndex])
        finiteVals = [v for v in [q11, q21, q12, q22] if np.isfinite(v)]
        if not finiteVals:
            return -np.inf
        if len(finiteVals) < 4 or x1 <= x0 or y1 <= y0:
            return float(max(finiteVals))
        q11 = q11 if np.isfinite(q11) else max(finiteVals)
        q21 = q21 if np.isfinite(q21) else max(finiteVals)
        q12 = q12 if np.isfinite(q12) else max(finiteVals)
        q22 = q22 if np.isfinite(q22) else max(finiteVals)
        tx = (xVal - x0) / (x1 - x0)
        ty = (yVal - y0) / (y1 - y0)
        return float((q11 * (1 - tx) + q21 * tx) * (1 - ty) + (q12 * (1 - tx) + q22 * tx) * ty)

    def sampleSegmentLocal(self, startPoint: np.ndarray, endPoint: np.ndarray, sampleStep: float) -> np.ndarray:
        segmentLength = float(np.linalg.norm(endPoint - startPoint))
        if segmentLength <= sampleStep:
            return np.asarray([startPoint, endPoint], dtype=float)
        sampleCount = int(np.ceil(segmentLength / sampleStep))
        tValues = np.linspace(0.0, 1.0, sampleCount + 1)
        return np.outer(1.0 - tValues, startPoint) + np.outer(tValues, endPoint)

    def clipPathsByCollisionChecker(self, pathsLocal: List[np.ndarray], collisionChecker: MeshCollisionChecker, sampleStep: float) -> List[np.ndarray]:
        if collisionChecker is None or collisionChecker.isEmpty:
            return [np.asarray(p, dtype=float) for p in pathsLocal if len(p) >= 2]
        clippedPaths = []
        for pathLocal in pathsLocal:
            pathArray = np.asarray(pathLocal, dtype=float)
            if len(pathArray) < 2:
                continue
            denseList = [pathArray[0]]
            for ptIdx in range(1, len(pathArray)):
                segs = self.sampleSegmentLocal(pathArray[ptIdx - 1], pathArray[ptIdx], sampleStep)
                denseList.extend(segs[1:].tolist())
            denseArray = np.asarray(denseList, dtype=float)
            collidingMask = collisionChecker.batchPointsColliding(denseArray)
            currentPath = []
            for ptIdx in range(len(denseArray)):
                if not collidingMask[ptIdx]:
                    currentPath.append(denseArray[ptIdx])
                else:
                    if len(currentPath) >= 2:
                        clippedPaths.append(np.asarray(currentPath, dtype=float))
                    currentPath = []
            if len(currentPath) >= 2:
                clippedPaths.append(np.asarray(currentPath, dtype=float))
        return clippedPaths

    def clipPathsByObstacleLocal(self, pathsLocal: List[np.ndarray], heightMapLocal: Optional[Dict[str, Any]], sampleStep: float, clearance: float) -> List[np.ndarray]:
        if heightMapLocal is None:
            return [np.asarray(p, dtype=float) for p in pathsLocal if len(p) >= 2]
        clippedPaths = []
        for pathLocal in pathsLocal:
            pathArray = np.asarray(pathLocal, dtype=float)
            if len(pathArray) < 2:
                continue
            denseList = [pathArray[0]]
            for ptIdx in range(1, len(pathArray)):
                segs = self.sampleSegmentLocal(pathArray[ptIdx - 1], pathArray[ptIdx], sampleStep)
                denseList.extend(segs[1:].tolist())
            denseArray = np.asarray(denseList, dtype=float)
            currentPath = []
            for pointLocal in denseArray:
                hLimit = self.queryHeightLimitLocal(heightMapLocal, float(pointLocal[0]), float(pointLocal[1]))
                if float(pointLocal[2]) >= float(hLimit + clearance):
                    currentPath.append(pointLocal)
                else:
                    if len(currentPath) >= 2:
                        clippedPaths.append(np.asarray(currentPath, dtype=float))
                    currentPath = []
            if len(currentPath) >= 2:
                clippedPaths.append(np.asarray(currentPath, dtype=float))
        return clippedPaths

    def isSegmentCollisionFreeLocal(self, startPoint: np.ndarray, endPoint: np.ndarray, heightMapLocal: Optional[Dict[str, Any]], sampleStep: float, clearance: float) -> bool:
        if heightMapLocal is None:
            return True
        segs = self.sampleSegmentLocal(startPoint, endPoint, sampleStep)
        for pt in segs:
            hLimit = self.queryHeightLimitLocal(heightMapLocal, float(pt[0]), float(pt[1]))
            if float(pt[2]) < float(hLimit + clearance):
                return False
        return True

    def _greedyOrderPaths(self, validPaths: List[np.ndarray]) -> List[np.ndarray]:
        from scipy.spatial.distance import cdist
        orderedPaths = []
        unvisited = list(range(len(validPaths)))
        currentPos = None
        while unvisited:
            if currentPos is None:
                chosenListIndex = 0
                reverseFlag = False
            else:
                startPoints = np.asarray([validPaths[i][0] for i in unvisited], dtype=float)
                endPoints = np.asarray([validPaths[i][-1] for i in unvisited], dtype=float)
                startDists = cdist([currentPos], startPoints)[0]
                endDists = cdist([currentPos], endPoints)[0]
                bestStartIdx = int(np.argmin(startDists))
                bestEndIdx = int(np.argmin(endDists))
                if startDists[bestStartIdx] <= endDists[bestEndIdx]:
                    chosenListIndex = bestStartIdx
                    reverseFlag = False
                else:
                    chosenListIndex = bestEndIdx
                    reverseFlag = True
            chosenPathIndex = unvisited.pop(chosenListIndex)
            pathArray = validPaths[chosenPathIndex] if not reverseFlag else validPaths[chosenPathIndex][::-1]
            orderedPaths.append(pathArray)
            currentPos = pathArray[-1]
        return orderedPaths

    def optimizePathLinking(self, pathsLocal: List[np.ndarray], safeZ: float, stepOver: float, obstacleMeshLocal: Optional[trimesh.Trimesh] = None, toolRadius: float = 3.0, safetyMargin: float = 0.5, collisionChecker: Optional[MeshCollisionChecker] = None) -> np.ndarray:
        if not pathsLocal:
            return np.zeros((0, 3), dtype=float)
        validPaths = [np.asarray(p, dtype=float) for p in pathsLocal if len(p) >= 2]
        if not validPaths:
            return np.zeros((0, 3), dtype=float)

        envelope = SafeEnvelope(obstacleMeshLocal, toolRadius, safetyMargin) if obstacleMeshLocal is not None and not obstacleMeshLocal.is_empty else None
        effectiveSafeZ = float(safeZ)
        if envelope is not None:
            effectiveSafeZ = max(effectiveSafeZ, envelope.safeZ + 1.0)

        sampleStep = max(float(stepOver * 0.3), toolRadius * 0.25, 0.15)
        orderedPaths = self._greedyOrderPaths(validPaths)

        linkedPoints = list(orderedPaths[0])
        for pathIdx in range(1, len(orderedPaths)):
            nextPath = orderedPaths[pathIdx]
            lastPt = np.asarray(linkedPoints[-1], dtype=float)
            nextStart = np.asarray(nextPath[0], dtype=float)

            directFree = False
            if envelope is not None:
                directFree = envelope.isSegmentSafe(lastPt, nextStart, sampleStep)
            elif collisionChecker is not None:
                directFree = not collisionChecker.isSegmentColliding(lastPt, nextStart, sampleStep)
            else:
                directFree = True

            if directFree:
                transitSegs = self.sampleSegmentLocal(lastPt, nextStart, sampleStep)
                linkedPoints.extend(transitSegs[1:].tolist())
            else:
                if envelope is not None:
                    transitPts = envelope.buildSafeTransit(lastPt, nextStart, sampleStep, max(toolRadius, 2.0))
                else:
                    retractPt = lastPt.copy()
                    retractPt[2] = effectiveSafeZ
                    approachPt = nextStart.copy()
                    approachPt[2] = effectiveSafeZ
                    transitPts = [lastPt, retractPt, approachPt, nextStart]
                linkedPoints.extend(transitPts[1:])

            linkedPoints.extend(nextPath.tolist())
        return np.asarray(linkedPoints, dtype=float)

    def slicePathByPlatformZ(self, pathLocal: np.ndarray, rotBack: np.ndarray, platformSafeZ: float) -> List[np.ndarray]:
        if len(pathLocal) == 0:
            return []
        pathWcs = applyRotation(pathLocal, rotBack)
        validMask = pathWcs[:, 2] >= platformSafeZ
        subPaths = []
        currentPath = []
        for pointIndex, isValid in enumerate(validMask):
            if isValid:
                currentPath.append(pathLocal[pointIndex])
            else:
                if len(currentPath) >= 2:
                    subPaths.append(np.asarray(currentPath, dtype=float))
                currentPath = []
        if len(currentPath) >= 2:
            subPaths.append(np.asarray(currentPath, dtype=float))
        return subPaths
