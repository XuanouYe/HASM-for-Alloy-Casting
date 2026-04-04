from typing import List, Tuple
import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from .toolModel import FlatEndMillTool
from .implicitGeometry import SdfVolume

class SweptVolumeCollisionEngine:
    def __init__(self, tool: FlatEndMillTool, sdfList: List[SdfVolume], clearanceList: List[float]):
        self.tool = tool
        self.sdfList = sdfList
        self.clearanceList = [float(c) for c in clearanceList]

    def _slerpAxis(self, a0: np.ndarray, a1: np.ndarray, t: float) -> np.ndarray:
        a0n = a0 / np.linalg.norm(a0)
        a1n = a1 / np.linalg.norm(a1)
        dot = float(np.clip(np.dot(a0n, a1n), -1.0, 1.0))
        if dot > 0.9999:
            result = (1.0 - t) * a0n + t * a1n
            norm = np.linalg.norm(result)
            return result / norm if norm > 1e-9 else a0n
        angle = np.arccos(dot)
        sinAngle = np.sin(angle)
        return (np.sin((1.0 - t) * angle) / sinAngle) * a0n + (np.sin(t * angle) / sinAngle) * a1n

    def _checkPose(self, tipCenter: np.ndarray, axisVec: np.ndarray, diskCount: int = 32, ringCount: int = 8) -> bool:
        worldPts = self.tool.worldSurfaceSamples(tipCenter, axisVec, diskCount, ringCount)
        for sdfVol, clearance in zip(self.sdfList, self.clearanceList):
            if sdfVol.isEmpty:
                continue
            sdValues = sdfVol.query(worldPts)
            if float(np.min(sdValues)) < -clearance:
                return True
        return False

    def checkSegment(self, p0: np.ndarray, a0: np.ndarray, p1: np.ndarray, a1: np.ndarray, tol: float,
                     maxSubdivDepth: int = 6, samplesPerSub: int = 4) -> bool:
        p0 = np.asarray(p0, dtype=float)
        p1 = np.asarray(p1, dtype=float)
        a0 = np.asarray(a0, dtype=float) / np.linalg.norm(a0)
        a1 = np.asarray(a1, dtype=float) / np.linalg.norm(a1)

        stack = [(0.0, 1.0, 0)]
        while stack:
            tStart, tEnd, depth = stack.pop()
            tVals = np.linspace(tStart, tEnd, samplesPerSub + 2)
            for tVal in tVals:
                p = (1.0 - tVal) * p0 + tVal * p1
                a = self._slerpAxis(a0, a1, tVal)
                if self._checkPose(p, a):
                    return True
            if depth < maxSubdivDepth:
                segLen = float(np.linalg.norm((tEnd - tStart) * (p1 - p0)))
                axisAngle = np.arccos(np.clip(float(np.dot(self._slerpAxis(a0, a1, tStart), self._slerpAxis(a0, a1, tEnd))), -1.0, 1.0))
                if segLen > tol or axisAngle > np.radians(5.0):
                    tMid = (tStart + tEnd) * 0.5
                    stack.append((tMid, tEnd, depth + 1))
                    stack.append((tStart, tMid, depth + 1))
        return False

    def filterPaths(self, pathsLocal: List[np.ndarray], axisVec: np.ndarray,
                    rotBack: np.ndarray, tol: float) -> List[np.ndarray]:
        from .geometryUtils import applyRotation, normalizeVector
        safePaths = []
        axisUnit = normalizeVector(np.asarray(axisVec, dtype=float))
        for pathLocal in pathsLocal:
            pathArr = np.asarray(pathLocal, dtype=float)
            if len(pathArr) < 2:
                continue
            pathWcs = applyRotation(pathArr, rotBack)
            currentSafe = [pathWcs[0]]
            for i in range(1, len(pathWcs)):
                p0w = pathWcs[i - 1]
                p1w = pathWcs[i]
                if not self.checkSegment(p0w, axisUnit, p1w, axisUnit, tol):
                    currentSafe.append(p1w)
                else:
                    if len(currentSafe) >= 2:
                        localSeg = applyRotation(np.asarray(currentSafe, dtype=float),
                                                  np.linalg.inv(rotBack))
                        safePaths.append(localSeg)
                    currentSafe = []
            if len(currentSafe) >= 2:
                localSeg = applyRotation(np.asarray(currentSafe, dtype=float),
                                          np.linalg.inv(rotBack))
                safePaths.append(localSeg)
        return safePaths
