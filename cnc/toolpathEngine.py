from typing import Any, Dict, List, Optional
import numpy as np
from scipy.spatial import cKDTree
import trimesh
from .geometryUtils import applyRotation
from .toolModel import FlatEndMillTool
from .implicitGeometry import SdfVolume, buildSdfVolume, buildOffsetSdf
from .sweptCollision import SweptVolumeCollisionEngine

class PointCloudIPW:
    def __init__(self, mesh: trimesh.Trimesh, sampleCount: int = 50000):
        self.points = np.zeros((0, 3), dtype=float)
        self.activeMask = np.ones(0, dtype=bool)
        if mesh is not None and not mesh.is_empty:
            pts, _ = trimesh.sample.sample_surface(mesh, sampleCount)
            self.points = np.asarray(pts, dtype=float)
            self.activeMask = np.ones(len(self.points), dtype=bool)

    def getActivePointsWcs(self) -> np.ndarray:
        return self.points[self.activeMask]

    def filterPathsLocal(self, pathsLocal: List[np.ndarray], rotToToolFrame: np.ndarray, toolRadius: float, stepOver: float) -> List[np.ndarray]:
        if not pathsLocal or not np.any(self.activeMask):
            return []
        activeWcs = self.getActivePointsWcs()
        if len(activeWcs) == 0:
            return []
        activeLocal = applyRotation(activeWcs, rotToToolFrame)
        tree = cKDTree(activeLocal)
        validPaths = []
        searchRadius = toolRadius + stepOver * 1.5
        for pathLocal in pathsLocal:
            dists, _ = tree.query(np.asarray(pathLocal, dtype=float), k=1, distance_upper_bound=searchRadius)
            if np.any(dists != np.inf):
                validPaths.append(np.asarray(pathLocal, dtype=float))
        return validPaths

    def updateIpwLocal(self, pathsLocal: List[np.ndarray], rotToToolFrame: np.ndarray, toolRadius: float) -> None:
        if not pathsLocal or not np.any(self.activeMask):
            return
        activeWcs = self.getActivePointsWcs()
        activeIndices = np.where(self.activeMask)[0]
        activeLocal = applyRotation(activeWcs, rotToToolFrame)
        tree = cKDTree(activeLocal[:, :2])
        removeSet = set()
        for pathLocal in pathsLocal:
            ballIdxList = tree.query_ball_point(np.asarray(pathLocal, dtype=float)[:, :2], r=toolRadius)
            for ptIdx, idxList in enumerate(ballIdxList):
                pz = float(pathLocal[ptIdx][2])
                for li in idxList:
                    if activeLocal[li, 2] >= pz - toolRadius * 0.5:
                        removeSet.add(li)
        if removeSet:
            self.activeMask[activeIndices[list(removeSet)]] = False

class TrimeshToolpathEngine:
    def buildFlatEndMillTool(self, toolParams: Dict[str, Any]) -> FlatEndMillTool:
        radius = float(toolParams.get("diameter", 6.0)) * 0.5
        length = float(toolParams.get("toolLength", radius * 4.0))
        shankRadius = float(toolParams.get("shankDiameter", toolParams.get("diameter", 6.0))) * 0.5
        return FlatEndMillTool(radius, length, shankRadius)

    def buildSdfForMesh(self, mesh: trimesh.Trimesh, voxelSize: float) -> SdfVolume:
        return buildSdfVolume(mesh, voxelSize)

    def buildSweptCollisionEngine(self, tool: FlatEndMillTool, protectMeshes: List[trimesh.Trimesh],
                                  clearances: List[float], voxelSize: float) -> SweptVolumeCollisionEngine:
        sdfList = [buildSdfVolume(m, voxelSize) for m in protectMeshes]
        return SweptVolumeCollisionEngine(tool, sdfList, clearances)

    def buildSweptCollisionEngineWithOffset(self, tool: FlatEndMillTool, protectMeshes: List[trimesh.Trimesh],
                                            clearances: List[float], offsets: List[float], voxelSize: float) -> SweptVolumeCollisionEngine:
        sdfList = []
        for mesh, offset in zip(protectMeshes, offsets):
            if offset != 0.0:
                sdfList.append(buildOffsetSdf(mesh, offset, voxelSize))
            else:
                sdfList.append(buildSdfVolume(mesh, voxelSize))
        return SweptVolumeCollisionEngine(tool, sdfList, clearances)

    def sampleSegmentLocal(self, startPoint: np.ndarray, endPoint: np.ndarray, sampleStep: float) -> np.ndarray:
        segLen = float(np.linalg.norm(endPoint - startPoint))
        if segLen <= sampleStep:
            return np.asarray([startPoint, endPoint], dtype=float)
        n = int(np.ceil(segLen / sampleStep))
        tVals = np.linspace(0.0, 1.0, n + 1)
        return np.outer(1.0 - tVals, startPoint) + np.outer(tVals, endPoint)

    def slicePathByPlatformZ(self, pathLocal: np.ndarray, rotBack: np.ndarray, platformSafeZ: float) -> List[np.ndarray]:
        if len(pathLocal) == 0:
            return []
        pathWcs = applyRotation(pathLocal, rotBack)
        validMask = pathWcs[:, 2] >= platformSafeZ
        subPaths = []
        current = []
        for i, isValid in enumerate(validMask):
            if isValid:
                current.append(pathLocal[i])
            else:
                if len(current) >= 2:
                    subPaths.append(np.asarray(current, dtype=float))
                current = []
        if len(current) >= 2:
            subPaths.append(np.asarray(current, dtype=float))
        return subPaths
