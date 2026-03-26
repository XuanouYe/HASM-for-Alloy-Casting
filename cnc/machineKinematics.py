import math
from typing import Any, Dict, List, Optional, Tuple


def buildTranslation(tx: float, ty: float, tz: float) -> List[List[float]]:
    return [
        [1.0, 0.0, 0.0, tx],
        [0.0, 1.0, 0.0, ty],
        [0.0, 0.0, 1.0, tz],
        [0.0, 0.0, 0.0, 1.0],
    ]


def buildRotationA(thetaA: float) -> List[List[float]]:
    ca = math.cos(thetaA)
    sa = math.sin(thetaA)
    return [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, ca, -sa, 0.0],
        [0.0, sa, ca, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def buildRotationC(thetaC: float) -> List[List[float]]:
    cc = math.cos(thetaC)
    sc = math.sin(thetaC)
    return [
        [cc, -sc, 0.0, 0.0],
        [sc, cc, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def multiplyHomogeneous(leftMat: List[List[float]], rightMat: List[List[float]]) -> List[List[float]]:
    resultMat = [[0.0] * 4 for _ in range(4)]
    for rowIdx in range(4):
        for colIdx in range(4):
            resultMat[rowIdx][colIdx] = sum(leftMat[rowIdx][kIdx] * rightMat[kIdx][colIdx] for kIdx in range(4))
    return resultMat


def applyHomogeneous(mat: List[List[float]], vec4: List[float]) -> List[float]:
    return [sum(mat[rowIdx][colIdx] * vec4[colIdx] for colIdx in range(4)) for rowIdx in range(4)]


def inverseHomogeneous(mat: List[List[float]]) -> List[List[float]]:
    rotMat = [row[:3] for row in mat[:3]]
    transVec = [mat[0][3], mat[1][3], mat[2][3]]
    rotInv = [[rotMat[colIdx][rowIdx] for colIdx in range(3)] for rowIdx in range(3)]
    transInv = [-sum(rotInv[rowIdx][kIdx] * transVec[kIdx] for kIdx in range(3)) for rowIdx in range(3)]
    return [
        [rotInv[0][0], rotInv[0][1], rotInv[0][2], transInv[0]],
        [rotInv[1][0], rotInv[1][1], rotInv[1][2], transInv[1]],
        [rotInv[2][0], rotInv[2][1], rotInv[2][2], transInv[2]],
        [0.0, 0.0, 0.0, 1.0],
    ]


class XyzacTrtKinematics:
    def __init__(self, kinematicsCfg: Dict[str, Any]):
        self.kinematicsCfg = kinematicsCfg
        self.aSign = float(kinematicsCfg.get("aSign", 1.0))
        self.cSign = float(kinematicsCfg.get("cSign", 1.0))
        self.aAxisLimit = kinematicsCfg.get("aAxisLimit", [-120.0, 120.0])
        self.cAxisLimit = kinematicsCfg.get("cAxisLimit", [-360.0, 360.0])
        self.pivotOffsetY = float(kinematicsCfg.get("pivotOffsetY", 0.0))
        self.pivotOffsetZ = float(kinematicsCfg.get("pivotOffsetZ", 0.0))
        self.workOffsetX = float(kinematicsCfg.get("workOffsetX", 0.0))
        self.workOffsetY = float(kinematicsCfg.get("workOffsetY", 0.0))
        self.workOffsetZ = float(kinematicsCfg.get("workOffsetZ", 0.0))
        self.singularityEps = float(kinematicsCfg.get("singularityEps", 0.01))
        self.prevA: Optional[float] = None
        self.prevC: Optional[float] = None

    def solveRotaryAngles(self, toolAxisVec: List[float]) -> Tuple[float, float]:
        dx = float(toolAxisVec[0])
        dy = float(toolAxisVec[1])
        dz = float(toolAxisVec[2])
        vecLen = math.sqrt(dx * dx + dy * dy + dz * dz)
        if vecLen > 0.0:
            dx, dy, dz = dx / vecLen, dy / vecLen, dz / vecLen
        aRad = math.acos(max(-1.0, min(1.0, dz)))
        sinA = math.sin(aRad)
        if abs(sinA) < self.singularityEps:
            cDegBase = 0.0 if self.prevC is None else self.prevC
        else:
            cDegBase = math.degrees(math.atan2(dx, -dy))
        aDegBase = math.degrees(aRad)
        aDeg = self.limitAngle(aDegBase * self.aSign, self.aAxisLimit)
        cDeg = self.limitAngle(cDegBase * self.cSign, self.cAxisLimit)
        return aDeg, cDeg

    def buildForwardTransform(self, aDeg: float, cDeg: float) -> List[List[float]]:
        aRad = math.radians(aDeg)
        cRad = math.radians(cDeg)
        workMat = buildTranslation(self.workOffsetX, self.workOffsetY, self.workOffsetZ)
        pivotMat = buildTranslation(0.0, self.pivotOffsetY, self.pivotOffsetZ)
        rotAMat = buildRotationA(aRad)
        rotCMat = buildRotationC(cRad)
        return multiplyHomogeneous(workMat, multiplyHomogeneous(rotCMat, multiplyHomogeneous(pivotMat, rotAMat)))

    def solveLinearAxes(self, tipPositionWcs: List[float], aDeg: float, cDeg: float) -> Tuple[float, float, float]:
        chainMat = self.buildForwardTransform(aDeg, cDeg)
        invMat = inverseHomogeneous(chainMat)
        qVec = [float(tipPositionWcs[0]), float(tipPositionWcs[1]), float(tipPositionWcs[2]), 1.0]
        pVec = applyHomogeneous(invMat, qVec)
        return pVec[0], pVec[1], pVec[2]

    def minimizeAngularJump(self, prevA: Optional[float], prevC: Optional[float], newA: float, newC: float) -> Tuple[float, float]:
        if prevA is None or prevC is None:
            return newA, newC
        cCandidates = [newC + shiftVal for shiftVal in (-720.0, -360.0, 0.0, 360.0, 720.0)]
        pairedCandidates: List[Tuple[float, float]] = []
        for cCandidate in cCandidates:
            pairedCandidates.append((newA, cCandidate))
            pairedCandidates.append((-newA, cCandidate + 180.0))
        bestA, bestC = pairedCandidates[0]
        bestCost = abs(bestA - prevA) + abs(bestC - prevC)
        for aCandidate, cCandidate in pairedCandidates[1:]:
            costVal = abs(aCandidate - prevA) + abs(cCandidate - prevC)
            if costVal < bestCost:
                bestCost = costVal
                bestA = aCandidate
                bestC = cCandidate
        bestA = self.limitAngle(bestA, self.aAxisLimit)
        bestC = self.limitAngle(bestC, self.cAxisLimit)
        return bestA, bestC

    def convertPoint(self, tipPositionWcs: List[float], toolAxisVec: List[float]) -> Tuple[float, float, float, float, float]:
        newA, newC = self.solveRotaryAngles(toolAxisVec)
        optA, optC = self.minimizeAngularJump(self.prevA, self.prevC, newA, newC)
        px, py, pz = self.solveLinearAxes(tipPositionWcs, optA, optC)
        self.prevA = optA
        self.prevC = optC
        return px, py, pz, optA, optC

    @staticmethod
    def limitAngle(angleVal: float, limits: List[float]) -> float:
        return max(float(limits[0]), min(float(limits[1]), angleVal))
