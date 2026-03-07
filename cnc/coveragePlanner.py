from typing import Any, Dict, List, Optional
import numpy as np
import trimesh
from .geometryUtils import deduplicateAxes, densifyPolyline, generateHemisphereAxes, nearestDistanceToPath, normalizeVector, sampleMeshPointsWithNormals

class ShellCoveragePlanner:
    def __init__(self, shellMesh: trimesh.Trimesh, keepOutMesh: trimesh.Trimesh, toolRadius: float, safetyMargin: float, sampleCount: int = 8000, accessAngleDeg: float = 72.0, minAxisZ: float = 0.02):
        self.shellMesh = shellMesh
        self.keepOutMesh = keepOutMesh
        self.toolRadius = float(toolRadius)
        self.safetyMargin = float(safetyMargin)
        self.samplePoints, self.sampleNormals = sampleMeshPointsWithNormals(shellMesh, sampleCount)
        self.coveredMask = np.zeros(len(self.samplePoints), dtype=bool)
        self.accessCos = float(np.cos(np.radians(accessAngleDeg)))
        self.minAxisZ = float(minAxisZ)
        combinedMesh = shellMesh if keepOutMesh.is_empty else trimesh.util.concatenate([shellMesh, keepOutMesh])
        bounds = np.asarray(combinedMesh.bounds, dtype=float)
        self.rayOffset = float(np.linalg.norm(bounds[1] - bounds[0]) * 0.25 + toolRadius + safetyMargin + 2.0)
        self.hitTolerance = float(max(toolRadius * 1.2, 1.0))
        self.coverDistance = float(max(toolRadius * 1.6, 1.5))
        self.combinedMesh = combinedMesh

    def getCoverageRatio(self) -> float:
        if len(self.samplePoints) == 0:
            return 1.0
        return float(np.mean(self.coveredMask))

    def getUncoveredMask(self) -> np.ndarray:
        return np.logical_not(self.coveredMask)

    def buildLocalAxesFromUncovered(self, localAxisCount: int) -> List[np.ndarray]:
        uncoveredMask = self.getUncoveredMask()
        if not np.any(uncoveredMask):
            return []
        uncoveredNormals = self.sampleNormals[uncoveredMask]
        uncoveredPoints = self.samplePoints[uncoveredMask]
        sideScore = 1.0 - np.clip(uncoveredNormals[:, 2], 0.0, 1.0)
        order = np.argsort(-sideScore)
        candidateAxes = []
        stepVal = max(int(len(order) / max(localAxisCount, 1)), 1)
        for orderIndex in order[::stepVal]:
            axisVec = normalizeVector(np.asarray(uncoveredNormals[orderIndex], dtype=float))
            if axisVec[2] < self.minAxisZ:
                axisVec[2] = self.minAxisZ
            axisVec = normalizeVector(axisVec)
            candidateAxes.append(axisVec)
            if len(candidateAxes) >= localAxisCount:
                break
        if len(candidateAxes) < localAxisCount and len(uncoveredPoints) > 0:
            pointOrder = np.linspace(0, len(uncoveredPoints) - 1, min(localAxisCount, len(uncoveredPoints)), dtype=int)
            for pointIndex in pointOrder:
                axisVec = normalizeVector(np.asarray(uncoveredNormals[pointIndex], dtype=float))
                if axisVec[2] < self.minAxisZ:
                    axisVec[2] = self.minAxisZ
                axisVec = normalizeVector(axisVec)
                candidateAxes.append(axisVec)
        return deduplicateAxes(candidateAxes, 10.0)

    def buildCandidateAxes(self, baseAxisCount: int, localAxisCount: int, existingAxes: List[np.ndarray]) -> List[np.ndarray]:
        globalAxes = generateHemisphereAxes(baseAxisCount, self.minAxisZ)
        localAxes = self.buildLocalAxesFromUncovered(localAxisCount)
        return deduplicateAxes(globalAxes + localAxes + existingAxes, 10.0)

    def evaluateAxis(self, toolAxis: np.ndarray) -> Dict[str, Any]:
        if len(self.samplePoints) == 0:
            return {'axis': toolAxis, 'score': 0.0, 'indices': np.zeros(0, dtype=int)}
        toolAxis = normalizeVector(np.asarray(toolAxis, dtype=float))
        if toolAxis[2] < self.minAxisZ:
            return {'axis': toolAxis, 'score': 0.0, 'indices': np.zeros(0, dtype=int)}
        uncoveredIndices = np.where(self.getUncoveredMask())[0]
        if len(uncoveredIndices) == 0:
            return {'axis': toolAxis, 'score': 0.0, 'indices': np.zeros(0, dtype=int)}
        points = self.samplePoints[uncoveredIndices]
        normals = self.sampleNormals[uncoveredIndices]
        normalMask = np.dot(normals, toolAxis) >= self.accessCos
        if not np.any(normalMask):
            return {'axis': toolAxis, 'score': 0.0, 'indices': np.zeros(0, dtype=int)}
        candidateIndices = uncoveredIndices[normalMask]
        candidatePoints = self.samplePoints[candidateIndices]
        rayOrigins = candidatePoints + toolAxis * self.rayOffset
        rayDirections = np.tile(-toolAxis, (len(candidatePoints), 1))
        locations, rayIndices, _ = self.combinedMesh.ray.intersects_location(ray_origins=rayOrigins, ray_directions=rayDirections)
        if len(locations) == 0:
            return {'axis': toolAxis, 'score': 0.0, 'indices': np.zeros(0, dtype=int)}
        accessibleIndices = []
        sideWeights = []
        for localIndex in range(len(candidatePoints)):
            hitMask = np.where(rayIndices == localIndex)[0]
            if len(hitMask) == 0:
                continue
            originPoint = rayOrigins[localIndex]
            hitLocations = locations[hitMask]
            distances = np.linalg.norm(hitLocations - originPoint, axis=1)
            nearestIndex = hitMask[int(np.argmin(distances))]
            nearestPoint = locations[nearestIndex]
            pointDistance = float(np.linalg.norm(nearestPoint - candidatePoints[localIndex]))
            if pointDistance <= self.hitTolerance:
                accessibleIndices.append(int(candidateIndices[localIndex]))
                sideWeights.append(float(1.0 + 0.8 * (1.0 - max(self.sampleNormals[candidateIndices[localIndex], 2], 0.0))))
        if not accessibleIndices:
            return {'axis': toolAxis, 'score': 0.0, 'indices': np.zeros(0, dtype=int)}
        return {'axis': toolAxis, 'score': float(np.sum(sideWeights)), 'indices': np.asarray(accessibleIndices, dtype=int)}

    def suggestAxes(self, baseAxisCount: int, localAxisCount: int, existingAxes: List[np.ndarray], topK: int = 8) -> List[Dict[str, Any]]:
        candidateAxes = self.buildCandidateAxes(baseAxisCount, localAxisCount, existingAxes)
        results = []
        for axisVec in candidateAxes:
            isNearExisting = False
            for existingAxis in existingAxes:
                if float(np.dot(axisVec, existingAxis)) > float(np.cos(np.radians(8.0))):
                    isNearExisting = True
                    break
            if isNearExisting:
                continue
            result = self.evaluateAxis(axisVec)
            if result['score'] > 0.0:
                results.append(result)
        results.sort(key=lambda item: item['score'], reverse=True)
        return results[:topK]

    def updateCoverageByPath(self, toolAxis: np.ndarray, pathPointsWcs: np.ndarray) -> None:
        if len(self.samplePoints) == 0 or len(pathPointsWcs) == 0:
            return
        toolAxis = normalizeVector(np.asarray(toolAxis, dtype=float))
        uncoveredMask = self.getUncoveredMask()
        if not np.any(uncoveredMask):
            return
        candidateIndices = np.where(uncoveredMask)[0]
        candidateNormals = self.sampleNormals[candidateIndices]
        normalMask = np.dot(candidateNormals, toolAxis) >= self.accessCos
        if not np.any(normalMask):
            return
        validIndices = candidateIndices[normalMask]
        densePath = densifyPolyline(np.asarray(pathPointsWcs, dtype=float), max(self.toolRadius * 0.6, 0.8))
        if len(densePath) == 0:
            return
        distances = nearestDistanceToPath(self.samplePoints[validIndices], densePath)
        coverMask = distances <= self.coverDistance
        if np.any(coverMask):
            self.coveredMask[validIndices[coverMask]] = True
