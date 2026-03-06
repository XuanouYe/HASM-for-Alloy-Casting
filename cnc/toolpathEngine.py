from typing import Any, Dict, List, Optional
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.distance import cdist
import trimesh
from .geometryUtils import applyRotation


class PointCloudIPW:
    def __init__(self, mesh: trimesh.Trimesh, sampleCount: int = 50000):
        try:
            self.points, _ = trimesh.sample.sample_surface(mesh, sampleCount)
        except Exception:
            self.points = np.zeros((0, 3), dtype=float)
        self.activeMask = np.ones(len(self.points), dtype=bool)

    def getActivePointsWcs(self) -> np.ndarray:
        return self.points[self.activeMask]

    def filterPathsLocal(self, pathsLocal: List[np.ndarray], rotToToolFrame: np.ndarray, toolRadius: float,
                         stepOver: float) -> List[np.ndarray]:
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
    def buildObstacleHeightMapLocal(self, obstacleMesh: trimesh.Trimesh, gridStep: float,
                                    toolRadius: float, safetyMargin: float) -> Optional[Dict[str, Any]]:
        if obstacleMesh.is_empty:
            return None
        marginVal = float(toolRadius + safetyMargin)
        bounds = np.asarray(obstacleMesh.bounds, dtype=float)
        xGrid = np.arange(bounds[0][0] - marginVal - gridStep, bounds[1][0] + marginVal + gridStep, gridStep, dtype=float)
        yGrid = np.arange(bounds[0][1] - marginVal - gridStep, bounds[1][1] + marginVal + gridStep, gridStep, dtype=float)
        if len(xGrid) == 0 or len(yGrid) == 0:
            return None
        zStart = float(bounds[1][2] + marginVal + gridStep)
        zLimit = np.full((len(yGrid), len(xGrid)), -np.inf, dtype=float)
        rayDirections = np.tile([0.0, 0.0, -1.0], (len(xGrid), 1))
        for rowIndex, yVal in enumerate(yGrid):
            rayOrigins = np.column_stack((xGrid, np.full(len(xGrid), yVal), np.full(len(xGrid), zStart)))
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
        x0 = float(xGrid[xIndex - 1])
        x1 = float(xGrid[xIndex])
        y0 = float(yGrid[yIndex - 1])
        y1 = float(yGrid[yIndex])
        q11 = float(zLimit[yIndex - 1, xIndex - 1])
        q21 = float(zLimit[yIndex - 1, xIndex])
        q12 = float(zLimit[yIndex, xIndex - 1])
        q22 = float(zLimit[yIndex, xIndex])
        finiteVals = [val for val in [q11, q21, q12, q22] if np.isfinite(val)]
        if not finiteVals:
            return -np.inf
        if len(finiteVals) < 4 or x1 <= x0 or y1 <= y0:
            return float(max(finiteVals))
        q11 = q11 if np.isfinite(q11) else max(finiteVals)
        q21 = q21 if np.isfinite(q21) else max(finiteVals)
        q12 = q12 if np.isfinite(q12) else max(finiteVals)
        q22 = q22 if np.isfinite(q22) else max(finiteVals)
        txVal = float((xVal - x0) / (x1 - x0))
        tyVal = float((yVal - y0) / (y1 - y0))
        z0Val = q11 * (1.0 - txVal) + q21 * txVal
        z1Val = q12 * (1.0 - txVal) + q22 * txVal
        return float(z0Val * (1.0 - tyVal) + z1Val * tyVal)

    def isPointAboveObstacleLocal(self, pointLocal: np.ndarray, heightMapLocal: Optional[Dict[str, Any]], clearance: float) -> bool:
        if heightMapLocal is None:
            return True
        heightLimit = self.queryHeightLimitLocal(heightMapLocal, float(pointLocal[0]), float(pointLocal[1]))
        return bool(float(pointLocal[2]) >= float(heightLimit + clearance))

    def sampleSegmentLocal(self, startPoint: np.ndarray, endPoint: np.ndarray, sampleStep: float) -> np.ndarray:
        segmentLength = float(np.linalg.norm(endPoint - startPoint))
        if segmentLength <= sampleStep:
            return np.asarray([startPoint, endPoint], dtype=float)
        sampleCount = int(np.ceil(segmentLength / sampleStep))
        tValues = np.linspace(0.0, 1.0, sampleCount + 1)
        return np.asarray([(1.0 - tVal) * startPoint + tVal * endPoint for tVal in tValues], dtype=float)

    def clipPathsByObstacleLocal(self, pathsLocal: List[np.ndarray], heightMapLocal: Optional[Dict[str, Any]],
                                 sampleStep: float, clearance: float) -> List[np.ndarray]:
        if heightMapLocal is None:
            return [np.asarray(pathLocal, dtype=float) for pathLocal in pathsLocal if len(pathLocal) >= 2]
        clippedPaths = []
        for pathLocal in pathsLocal:
            pathArray = np.asarray(pathLocal, dtype=float)
            if len(pathArray) < 2:
                continue
            densePoints = [pathArray[0]]
            for pointIndex in range(1, len(pathArray)):
                segmentSamples = self.sampleSegmentLocal(pathArray[pointIndex - 1], pathArray[pointIndex], sampleStep)
                densePoints.extend(segmentSamples[1:])
            denseArray = np.asarray(densePoints, dtype=float)
            currentPath = []
            for pointLocal in denseArray:
                if self.isPointAboveObstacleLocal(pointLocal, heightMapLocal, clearance):
                    currentPath.append(pointLocal)
                else:
                    if len(currentPath) >= 2:
                        clippedPaths.append(np.asarray(currentPath, dtype=float))
                    currentPath = []
            if len(currentPath) >= 2:
                clippedPaths.append(np.asarray(currentPath, dtype=float))
        return clippedPaths

    def isSegmentCollisionFreeLocal(self, startPoint: np.ndarray, endPoint: np.ndarray,
                                    heightMapLocal: Optional[Dict[str, Any]], sampleStep: float,
                                    clearance: float) -> bool:
        if heightMapLocal is None:
            return True
        segmentSamples = self.sampleSegmentLocal(startPoint, endPoint, sampleStep)
        for pointLocal in segmentSamples:
            if not self.isPointAboveObstacleLocal(pointLocal, heightMapLocal, clearance):
                return False
        return True

    def optimizePathLinking(self, pathsLocal: List[np.ndarray], safeZ: float, stepOver: float,
                            collisionMapLocal: Optional[Dict[str, Any]] = None,
                            clearance: float = 0.0) -> np.ndarray:
        if not pathsLocal:
            return np.zeros((0, 3), dtype=float)
        orderedPaths = []
        unvisited = list(range(len(pathsLocal)))
        currentPos = None
        while unvisited:
            if currentPos is None:
                chosenListIndex = 0
                reverseFlag = False
            else:
                startPoints = np.asarray([pathsLocal[pathIndex][0] for pathIndex in unvisited], dtype=float)
                endPoints = np.asarray([pathsLocal[pathIndex][-1] for pathIndex in unvisited], dtype=float)
                startDists = cdist([currentPos], startPoints)[0]
                endDists = cdist([currentPos], endPoints)[0]
                bestStartIndex = int(np.argmin(startDists))
                bestEndIndex = int(np.argmin(endDists))
                if startDists[bestStartIndex] <= endDists[bestEndIndex]:
                    chosenListIndex = bestStartIndex
                    reverseFlag = False
                else:
                    chosenListIndex = bestEndIndex
                    reverseFlag = True
            chosenPathIndex = unvisited.pop(chosenListIndex)
            pathArray = np.asarray(pathsLocal[chosenPathIndex], dtype=float)
            if reverseFlag:
                pathArray = pathArray[::-1]
            orderedPaths.append(pathArray)
            currentPos = pathArray[-1]
        linkedPoints = list(np.asarray(orderedPaths[0], dtype=float))
        sampleStep = max(float(stepOver * 0.5), 0.5)
        for pathIndex in range(1, len(orderedPaths)):
            nextPath = np.asarray(orderedPaths[pathIndex], dtype=float)
            lastPoint = np.asarray(linkedPoints[-1], dtype=float)
            nextStart = np.asarray(nextPath[0], dtype=float)
            shortLink = float(np.linalg.norm(lastPoint[:2] - nextStart[:2])) <= float(stepOver * 2.2)
            sameLevel = abs(float(lastPoint[2] - nextStart[2])) <= float(stepOver * 0.5 + 1e-6)
            directFree = self.isSegmentCollisionFreeLocal(lastPoint, nextStart, collisionMapLocal, sampleStep, clearance)
            if shortLink and sameLevel and directFree:
                directLink = self.sampleSegmentLocal(lastPoint, nextStart, sampleStep)
                linkedPoints.extend(directLink[1:])
            else:
                retractPoint = lastPoint.copy()
                retractPoint[2] = float(safeZ)
                approachPoint = nextStart.copy()
                approachPoint[2] = float(safeZ)
                linkedPoints.append(retractPoint)
                linkedPoints.append(approachPoint)
            linkedPoints.extend(nextPath)
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
