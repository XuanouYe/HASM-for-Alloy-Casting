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

class SolidKeepOutClipper:
    def __init__(self, solidMesh: trimesh.Trimesh):
        self.mesh = solidMesh
        self.isEmpty = solidMesh is None or solidMesh.is_empty

    def _densify(self, path: np.ndarray, sampleStep: float) -> np.ndarray:
        denseList = [path[0]]
        for i in range(1, len(path)):
            seg = path[i] - path[i - 1]
            segLen = float(np.linalg.norm(seg))
            if segLen <= sampleStep:
                denseList.append(path[i])
                continue
            nSamples = int(np.ceil(segLen / sampleStep))
            tVals = np.linspace(0.0, 1.0, nSamples + 1)
            segPts = np.outer(1.0 - tVals, path[i - 1]) + np.outer(tVals, path[i])
            denseList.extend(segPts[1:].tolist())
        return np.asarray(denseList, dtype=float)

    def segmentIsSafe(self, ptsArr: np.ndarray) -> bool:
        if self.isEmpty:
            return True
        return not bool(np.any(self.mesh.contains(ptsArr)))

    def clipPath(self, path: np.ndarray, sampleStep: float) -> List[np.ndarray]:
        if self.isEmpty or len(path) < 2:
            return [np.asarray(path, dtype=float)] if len(path) >= 2 else []
        denseArr = self._densify(np.asarray(path, dtype=float), sampleStep)
        insideMask = self.mesh.contains(denseArr)
        subPaths = []
        current = []
        for i in range(len(denseArr)):
            if not insideMask[i]:
                current.append(denseArr[i])
            else:
                if len(current) >= 2:
                    subPaths.append(np.asarray(current, dtype=float))
                current = []
        if len(current) >= 2:
            subPaths.append(np.asarray(current, dtype=float))
        return subPaths

    def clipPaths(self, paths: List[np.ndarray], sampleStep: float) -> List[np.ndarray]:
        if self.isEmpty:
            return [np.asarray(p, dtype=float) for p in paths if len(p) >= 2]
        result = []
        for p in paths:
            result.extend(self.clipPath(np.asarray(p, dtype=float), sampleStep))
        return result

class SafeEnvelope:
    def __init__(self, obstacleMesh: trimesh.Trimesh, toolRadius: float, safetyMargin: float):
        self.mesh = obstacleMesh
        self.isEmpty = obstacleMesh is None or obstacleMesh.is_empty
        self.clearance = float(toolRadius + safetyMargin)
        self.safeZ = float(-np.inf)
        self.meshCenter = None
        self.meshRadius = float(0.0)
        self._surfaceTree = None
        self._surfaceSamples = None
        if not self.isEmpty:
            bounds = np.asarray(obstacleMesh.bounds, dtype=float)
            self.meshCenter = (bounds[0] + bounds[1]) * 0.5
            self.meshRadius = float(np.linalg.norm(bounds[1] - bounds[0])) * 0.5
            self.safeZ = float(self.meshCenter[2]) + self.meshRadius + self.clearance
            try:
                pts, _ = trimesh.sample.sample_surface(obstacleMesh, 80000)
                self._surfaceSamples = np.asarray(pts, dtype=float)
                self._surfaceTree = cKDTree(self._surfaceSamples)
            except Exception:
                pass

    def _nearestSurfaceDist(self, pts: np.ndarray) -> np.ndarray:
        if self._surfaceTree is None or len(pts) == 0:
            return np.full(len(pts), np.inf, dtype=float)
        dists, _ = self._surfaceTree.query(pts, k=1)
        return np.asarray(dists, dtype=float)

    def _signedClearance(self, pts: np.ndarray) -> np.ndarray:
        if self.isEmpty:
            return np.full(len(pts), np.inf, dtype=float)
        dists = self._nearestSurfaceDist(pts)
        insideMask = self.mesh.contains(pts)
        signedVals = dists.copy()
        signedVals[insideMask] *= -1.0
        return signedVals

    def isPointSafe(self, pt: np.ndarray) -> bool:
        if self.isEmpty:
            return True
        ptArr = np.asarray(pt, dtype=float).reshape(1, 3)
        return float(self._signedClearance(ptArr)[0]) >= self.clearance

    def isSegmentSafe(self, startPt: np.ndarray, endPt: np.ndarray, sampleStep: float = 0.5) -> bool:
        if self.isEmpty:
            return True
        s = np.asarray(startPt, dtype=float)
        e = np.asarray(endPt, dtype=float)
        segLen = float(np.linalg.norm(e - s))
        if segLen < 1e-9:
            return self.isPointSafe(s)
        stepVal = max(min(sampleStep, self.clearance * 0.4), 0.05)
        nSamples = max(int(np.ceil(segLen / stepVal)) + 1, 2)
        tVals = np.linspace(0.0, 1.0, nSamples)
        samplePts = np.outer(1.0 - tVals, s) + np.outer(tVals, e)
        return bool(np.all(self._signedClearance(samplePts) >= self.clearance))

class MeshCollisionChecker:
    def __init__(self, obstacleMesh: trimesh.Trimesh, toolRadius: float, safetyMargin: float):
        self.obstacleMesh = obstacleMesh
        self.clearanceDist = float(toolRadius + safetyMargin)
        self.isEmpty = obstacleMesh is None or obstacleMesh.is_empty
        self.envelope = SafeEnvelope(obstacleMesh, toolRadius, safetyMargin)

    def isPointColliding(self, point: np.ndarray) -> bool:
        if self.isEmpty:
            return False
        return not self.envelope.isPointSafe(np.asarray(point, dtype=float))

    def batchPointsColliding(self, points: np.ndarray) -> np.ndarray:
        if self.isEmpty or len(points) == 0:
            return np.zeros(len(points), dtype=bool)
        pts = np.asarray(points, dtype=float)
        return self.envelope._signedClearance(pts) < self.clearanceDist

    def isSegmentColliding(self, startPoint: np.ndarray, endPoint: np.ndarray, sampleStep: float) -> bool:
        if self.isEmpty:
            return False
        return not self.envelope.isSegmentSafe(
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

    def buildSolidClipper(self, solidMesh: trimesh.Trimesh) -> SolidKeepOutClipper:
        return SolidKeepOutClipper(solidMesh)

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
                if float(pointLocal[2]) >= float(hLimit) - 1e-4:
                    currentPath.append(pointLocal)
                else:
                    if len(currentPath) >= 2:
                        clippedPaths.append(np.asarray(currentPath, dtype=float))
                    currentPath = []
            if len(currentPath) >= 2:
                clippedPaths.append(np.asarray(currentPath, dtype=float))
        return clippedPaths

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