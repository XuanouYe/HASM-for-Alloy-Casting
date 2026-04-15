from typing import List, Optional
import numpy as np
from scipy.spatial.transform import Rotation

from cnc.toolModel import FlatEndMillTool
from cnc.implicitGeometry import SdfVolume


def _buildRotMatFromAxis(axisVec: np.ndarray) -> np.ndarray:
    zUnit = np.array([0.0, 0.0, 1.0], dtype=float)
    axis = np.asarray(axisVec, dtype=float)
    axisNorm = np.linalg.norm(axis)
    if axisNorm < 1e-9:
        return np.eye(3, dtype=float)
    axis = axis / axisNorm
    rotAxis = np.cross(zUnit, axis)
    sinVal = float(np.linalg.norm(rotAxis))
    cosVal = float(np.dot(zUnit, axis))
    if sinVal < 1e-9:
        return np.eye(3, dtype=float) if cosVal > 0.0 else -np.eye(3, dtype=float)
    rotAxis = rotAxis / sinVal
    angle = np.arctan2(sinVal, cosVal)
    return Rotation.from_rotvec(angle * rotAxis).as_matrix()


class SweptVolumeCollisionEngine:
    def __init__(self, tool: FlatEndMillTool, sdfList: List[SdfVolume], clearanceList: List[float],
                 diskCount: int = 16, ringCount: int = 6, safeBuffer: float = 2.0):
        self.tool = tool
        self.sdfList = sdfList
        self.clearanceList = [float(c) for c in clearanceList]
        self.diskCount = int(diskCount)
        self.ringCount = int(ringCount)
        self.safeBuffer = float(safeBuffer)
        self._localPts = self.tool.sampleToolSurfaceLocal(self.diskCount, self.ringCount)
        self._nPts = len(self._localPts)

    def _minSdfAtPose(self, tipCenter: np.ndarray, rotMat: np.ndarray) -> float:
        worldPts = (rotMat @ self._localPts.T).T + tipCenter
        minDist = np.inf
        for sdfVol, clearance in zip(self.sdfList, self.clearanceList):
            if sdfVol.isEmpty:
                continue
            sdVals = sdfVol.query(worldPts)
            localMin = float(np.min(sdVals)) - clearance
            if localMin < minDist:
                minDist = localMin
        return minDist

    def _checkPoseBatch(self, tipCenters: np.ndarray, rotMats: np.ndarray) -> np.ndarray:
        N = len(tipCenters)
        worldPtsAll = np.einsum("nij,kj->nki", rotMats, self._localPts) + tipCenters[:, np.newaxis, :]
        worldPtsFlat = worldPtsAll.reshape(N * self._nPts, 3)
        minDists = np.full(N, np.inf, dtype=float)
        for sdfVol, clearance in zip(self.sdfList, self.clearanceList):
            if sdfVol.isEmpty:
                continue
            sdFlat = sdfVol.query(worldPtsFlat)
            sdPerPose = sdFlat.reshape(N, self._nPts)
            poseMin = np.min(sdPerPose, axis=1) - clearance
            np.minimum(minDists, poseMin, out=minDists)
        return minDists

    def _slerpAxis(self, a0: np.ndarray, a1: np.ndarray, t: float) -> np.ndarray:
        dot = float(np.clip(np.dot(a0, a1), -1.0, 1.0))
        if dot > 0.9999:
            v = (1.0 - t) * a0 + t * a1
            n = np.linalg.norm(v)
            return v / n if n > 1e-9 else a0
        angle = np.arccos(dot)
        sinA = np.sin(angle)
        return (np.sin((1.0 - t) * angle) / sinA) * a0 + (np.sin(t * angle) / sinA) * a1

    def _minSdfSegEndpoints(self, p0: np.ndarray, a0: np.ndarray,
                             p1: np.ndarray, a1: np.ndarray) -> float:
        r0 = _buildRotMatFromAxis(a0)
        r1 = _buildRotMatFromAxis(a1)
        centers = np.stack([p0, p1], axis=0)
        rots = np.stack([r0, r1], axis=0)
        dists = self._checkPoseBatch(centers, rots)
        return float(np.min(dists))

    def checkSegment(self, p0: np.ndarray, a0: np.ndarray, p1: np.ndarray, a1: np.ndarray,
                     tol: float, maxSubdivDepth: int = 8, samplesPerSub: int = 3) -> bool:
        p0 = np.asarray(p0, dtype=float)
        p1 = np.asarray(p1, dtype=float)
        a0 = np.asarray(a0, dtype=float)
        a0 /= np.linalg.norm(a0)
        a1 = np.asarray(a1, dtype=float)
        a1 /= np.linalg.norm(a1)

        endpointMin = self._minSdfSegEndpoints(p0, a0, p1, a1)
        if endpointMin > self.safeBuffer:
            return False

        stack = [(0.0, 1.0, 0, endpointMin)]
        while stack:
            tStart, tEnd, depth, parentMinDist = stack.pop()

            if parentMinDist < 0.0:
                return True

            tVals = np.linspace(tStart, tEnd, samplesPerSub + 2)
            tipCenters = np.array([(1.0 - t) * p0 + t * p1 for t in tVals], dtype=float)
            axisVecs = np.array([self._slerpAxis(a0, a1, t) for t in tVals], dtype=float)
            rotMats = np.stack([_buildRotMatFromAxis(av) for av in axisVecs], axis=0)
            batchDists = self._checkPoseBatch(tipCenters, rotMats)

            segMinDist = float(np.min(batchDists))
            if segMinDist < 0.0:
                return True

            if depth >= maxSubdivDepth:
                continue

            segLen = float(np.linalg.norm((tEnd - tStart) * (p1 - p0)))
            axisAngle = float(np.arccos(np.clip(
                np.dot(self._slerpAxis(a0, a1, tStart), self._slerpAxis(a0, a1, tEnd)), -1.0, 1.0
            )))

            if segLen <= tol and axisAngle <= np.radians(3.0):
                continue

            if segMinDist > self.safeBuffer:
                continue

            tMid = (tStart + tEnd) * 0.5
            stack.append((tMid, tEnd, depth + 1, segMinDist))
            stack.append((tStart, tMid, depth + 1, segMinDist))

        return False

    def filterPaths(self, pathsLocal: List[np.ndarray], axisVec: np.ndarray,
                    rotBack: np.ndarray, tol: float) -> List[np.ndarray]:
        from .geometryUtils import applyRotation, normalizeVector
        safePaths = []
        axisUnit = normalizeVector(np.asarray(axisVec, dtype=float))
        rotInv = np.linalg.inv(rotBack)
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
                        localSeg = applyRotation(np.asarray(currentSafe, dtype=float), rotInv)
                        safePaths.append(localSeg)
                    currentSafe = [p1w]
            if len(currentSafe) >= 2:
                localSeg = applyRotation(np.asarray(currentSafe, dtype=float), rotInv)
                safePaths.append(localSeg)
        return safePaths

    def recheckAndRepairLinks(self, clPoints: List[dict],
                              globalClearanceZ: float,
                              linkFeedRate: float,
                              tol: float) -> List[dict]:
        from copy import deepcopy
        repairedPoints: List[dict] = []
        i = 0
        while i < len(clPoints):
            pt = clPoints[i]
            repairedPoints.append(deepcopy(pt))
            if i + 1 < len(clPoints):
                nxt = clPoints[i + 1]
                p0 = np.asarray(pt["position"], dtype=float)
                p1 = np.asarray(nxt["position"], dtype=float)
                a0 = np.asarray(pt.get("toolAxis", [0.0, 0.0, 1.0]), dtype=float)
                n0 = np.linalg.norm(a0)
                a0 = a0 / n0 if n0 > 1e-9 else np.array([0.0, 0.0, 1.0])
                a1 = np.asarray(nxt.get("toolAxis", [0.0, 0.0, 1.0]), dtype=float)
                n1 = np.linalg.norm(a1)
                a1 = a1 / n1 if n1 > 1e-9 else np.array([0.0, 0.0, 1.0])
                if self.checkSegment(p0, a0, p1, a1, tol):
                    retractPt = {"pointId": 0,
                                 "position": [float(p0[0]), float(p0[1]), float(globalClearanceZ)],
                                 "toolAxis": [float(a0[0]), float(a0[1]), float(a0[2])],
                                 "feedrate": float(linkFeedRate),
                                 "segmentId": -1,
                                 "motionType": "retract"}
                    rapidPt  = {"pointId": 0,
                                "position": [float(p1[0]), float(p1[1]), float(globalClearanceZ)],
                                "toolAxis": [float(a1[0]), float(a1[1]), float(a1[2])],
                                "feedrate": float(linkFeedRate),
                                "segmentId": -1,
                                "motionType": "rapid"}
                    repairedPoints.append(retractPt)
                    repairedPoints.append(rapidPt)
            i += 1
        for idx, ptItem in enumerate(repairedPoints, start=1):
            ptItem["pointId"] = idx
        return repairedPoints
