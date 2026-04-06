import math
from copy import deepcopy
from typing import Any, Dict, List, Optional
import numpy as np


def buildRetractIntersector(mesh):
    if mesh is None:
        return None
    try:
        import trimesh
        return trimesh.ray.ray_triangle.RayMeshIntersector(mesh)
    except Exception:
        return None


def sdfGradientOutwardAxis(pos: List[float], toolAxis: List[float], mesh) -> List[float]:
    ax, ay, az = [float(v) for v in toolAxis]
    norm = math.sqrt(ax*ax + ay*ay + az*az)
    if norm < 1e-9:
        ax, ay, az = 0.0, 0.0, 1.0
    else:
        ax, ay, az = ax/norm, ay/norm, az/norm
    if mesh is None:
        return [ax, ay, az]
    try:
        import trimesh
        posArr = np.array([[pos[0], pos[1], pos[2]]], dtype=float)
        closestPts, distances, triangleIds = trimesh.proximity.closest_point(mesh, posArr)
        closestPt = closestPts[0]
        grad = np.array([pos[0] - closestPt[0],
                         pos[1] - closestPt[1],
                         pos[2] - closestPt[2]], dtype=float)
        gradNorm = float(np.linalg.norm(grad))
        if gradNorm < 1e-9:
            triIdx = int(triangleIds[0])
            grad = mesh.face_normals[triIdx].copy()
            gradNorm = float(np.linalg.norm(grad))
        if gradNorm < 1e-9:
            return [ax, ay, az]
        outward = grad / gradNorm
        dot = ax*outward[0] + ay*outward[1] + az*outward[2]
        if dot >= 0.0:
            return [ax, ay, az]
        return [-ax, -ay, -az]
    except Exception:
        return [ax, ay, az]


def resolveOutwardAxisMultiRay(pos: List[float], toolAxis: List[float],
                               intersector, mesh) -> List[float]:
    ax, ay, az = [float(v) for v in toolAxis]
    norm = math.sqrt(ax*ax + ay*ay + az*az)
    if norm < 1e-9:
        ax, ay, az = 0.0, 0.0, 1.0
    else:
        ax, ay, az = ax/norm, ay/norm, az/norm
    if intersector is None:
        return [ax, ay, az]
    try:
        epsilon = 1e-4
        originArr = np.array([
            [pos[0] + ax*epsilon, pos[1] + ay*epsilon, pos[2] + az*epsilon],
            [pos[0] - ax*epsilon, pos[1] - ay*epsilon, pos[2] - az*epsilon],
        ], dtype=float)
        dirArr = np.array([
            [ax, ay, az],
            [-ax, -ay, -az],
        ], dtype=float)
        hitCounts = intersector.intersects_count(originArr, dirArr)
        fwdParity = int(hitCounts[0]) % 2
        bwdParity = int(hitCounts[1]) % 2
        if fwdParity == 0 and bwdParity == 1:
            return [ax, ay, az]
        if fwdParity == 1 and bwdParity == 0:
            return [-ax, -ay, -az]
        gradResult = sdfGradientOutwardAxis(pos, [ax, ay, az], mesh)
        return gradResult
    except Exception:
        return sdfGradientOutwardAxis(pos, [ax, ay, az], mesh)


class ClPathLinker:
    def __init__(self, linkerConfig: Dict[str, Any]):
        self.safeHeight = float(linkerConfig.get("safeHeight", 5.0))
        self.maxRetractOffset = float(linkerConfig.get("maxRetractOffset", 100.0))
        self.directLinkThreshold = float(linkerConfig.get("directLinkThreshold", 2.0))
        self.rotationChangeThreshold = float(linkerConfig.get("rotationChangeThreshold", 5.0))
        self.rotationRetractAngle = float(linkerConfig.get("rotationRetractAngle", 30.0))
        self.rotationSafeZ = float(linkerConfig.get("rotationSafeZ", 30.0))
        self.linkFeedRate = float(linkerConfig.get("linkFeedRate", 2000.0))
        self.retractAlongAxis = bool(linkerConfig.get("retractAlongAxis", True))
        self.retractAxisLength = float(linkerConfig.get("retractAxisLength", 10.0))
        self.retractCollisionMesh = linkerConfig.get("retractCollisionMesh", None)
        self.stepLinkingEnabled = linkerConfig.get("stepLinkingEnabled", {})
        self._intersector = buildRetractIntersector(self.retractCollisionMesh)
        self._retractCache: Dict[tuple, List[float]] = {}

    def _outwardAxis(self, pos: List[float], toolAxis: List[float]) -> List[float]:
        cacheKey = (
            round(pos[0], 4), round(pos[1], 4), round(pos[2], 4),
            round(toolAxis[0], 6), round(toolAxis[1], 6), round(toolAxis[2], 6)
        )
        if cacheKey in self._retractCache:
            return self._retractCache[cacheKey]
        result = resolveOutwardAxisMultiRay(
            pos, toolAxis, self._intersector, self.retractCollisionMesh)
        self._retractCache[cacheKey] = result
        return result

    def _safeRetractLength(self, pos: List[float], outAxis: List[float],
                           desiredLen: float) -> float:
        if self._intersector is None:
            return desiredLen
        try:
            epsilon = 1e-4
            ax, ay, az = outAxis
            origin = np.array([
                [pos[0] + ax*epsilon,
                 pos[1] + ay*epsilon,
                 pos[2] + az*epsilon]
            ], dtype=float)
            direction = np.array([[ax, ay, az]], dtype=float)
            locs, rayIdxs, _ = self._intersector.intersects_location(origin, direction)
            if len(locs) == 0:
                return desiredLen
            dists = np.linalg.norm(locs - np.array([pos[0], pos[1], pos[2]]), axis=1)
            minDist = float(np.min(dists))
            if minDist > desiredLen:
                return desiredLen
            return max(0.0, minDist - 0.5)
        except Exception:
            return desiredLen

    def _buildRetractPos(self, pos: List[float], toolAxis: List[float],
                         length: float) -> List[float]:
        outAxis = self._outwardAxis(pos, toolAxis)
        safeLen = self._safeRetractLength(pos, outAxis, length)
        ax, ay, az = outAxis
        return [
            float(pos[0]) + ax * safeLen,
            float(pos[1]) + ay * safeLen,
            float(pos[2]) + az * safeLen,
        ]

    def processClData(self, clData: Dict[str, Any]) -> Dict[str, Any]:
        linkedData = deepcopy(clData)
        for stepItem in linkedData.get("steps", []):
            stepType = str(stepItem.get("stepType", ""))
            allowDirect = bool(self.stepLinkingEnabled.get(stepType, True))
            self._retractCache.clear()
            self.insertLinkPoints(stepItem, allowDirect)
        return linkedData

    def classifyLink(self, endPt: Dict[str, Any], startPt: Dict[str, Any],
                     endAxis: List[float], startAxis: List[float],
                     allowDirect: bool) -> int:
        endPos = endPt.get("position", [0.0, 0.0, 0.0])
        startPos = startPt.get("position", [0.0, 0.0, 0.0])
        moveDist = self.computeDistance(endPos, startPos)
        axisChange = self.computeAxisAngle(endAxis, startAxis)
        if axisChange >= self.rotationRetractAngle:
            return 3
        if (moveDist <= self.directLinkThreshold
                and axisChange <= self.rotationChangeThreshold
                and allowDirect):
            return 0
        if (moveDist <= self.maxRetractOffset
                and axisChange <= self.rotationChangeThreshold):
            return 1
        return 2

    def _buildAxisLink(self, endPos: List[float], endAxis: List[float],
                       startPos: List[float], startAxis: List[float],
                       length: float) -> List[Dict[str, Any]]:
        retractEnd = self._buildRetractPos(endPos, endAxis, length)
        retractStart = self._buildRetractPos(startPos, startAxis, length)
        return [
            self.makePoint(retractEnd,   endAxis,   "retract"),
            self.makePoint(retractStart, startAxis, "rapid"),
            self.makePoint(startPos,     startAxis, "approach"),
        ]

    def buildLevel1Link(self, endPt: Dict[str, Any], startPt: Dict[str, Any],
                        endAxis: List[float], startAxis: List[float],
                        localSafeZ: float) -> List[Dict[str, Any]]:
        endPos = endPt.get("position", [0.0, 0.0, 0.0])
        startPos = startPt.get("position", [0.0, 0.0, 0.0])
        if self.retractAlongAxis:
            return self._buildAxisLink(endPos, endAxis, startPos, startAxis,
                                       self.retractAxisLength)
        return [
            self.makePoint([endPos[0],   endPos[1],   localSafeZ], endAxis,   "retract"),
            self.makePoint([startPos[0], startPos[1], localSafeZ], endAxis,   "rapid"),
            self.makePoint(startPos,                               startAxis, "approach"),
        ]

    def buildLevel2Link(self, endPt: Dict[str, Any], startPt: Dict[str, Any],
                        endAxis: List[float], startAxis: List[float],
                        clearanceZ: float) -> List[Dict[str, Any]]:
        endPos = endPt.get("position", [0.0, 0.0, 0.0])
        startPos = startPt.get("position", [0.0, 0.0, 0.0])
        if self.retractAlongAxis:
            return self._buildAxisLink(endPos, endAxis, startPos, startAxis,
                                       self.retractAxisLength)
        return [
            self.makePoint([endPos[0],   endPos[1],   clearanceZ], endAxis,   "retract"),
            self.makePoint([startPos[0], startPos[1], clearanceZ], endAxis,   "rapid"),
            self.makePoint(startPos,                               startAxis, "approach"),
        ]

    def buildLevel3Link(self, endPt: Dict[str, Any], startPt: Dict[str, Any],
                        endAxis: List[float], startAxis: List[float],
                        rotationSafeZ: float, clearanceZ: float) -> List[Dict[str, Any]]:
        endPos = endPt.get("position", [0.0, 0.0, 0.0])
        startPos = startPt.get("position", [0.0, 0.0, 0.0])
        if self.retractAlongAxis:
            longLen = self.retractAxisLength * 2.0
            retractEnd = self._buildRetractPos(endPos,   endAxis,   longLen)
            retractStart = self._buildRetractPos(startPos, startAxis, longLen)
            return [
                self.makePoint(retractEnd,   endAxis,   "retract"),
                self.makePoint(retractEnd,   startAxis, "rapid"),
                self.makePoint(retractStart, startAxis, "rapid"),
                self.makePoint(startPos,     startAxis, "approach"),
            ]
        highZ = max(rotationSafeZ, clearanceZ)
        return [
            self.makePoint([endPos[0],   endPos[1],   highZ], endAxis,   "retract"),
            self.makePoint([endPos[0],   endPos[1],   highZ], startAxis, "rapid"),
            self.makePoint([startPos[0], startPos[1], highZ], startAxis, "rapid"),
            self.makePoint(startPos,                          startAxis, "approach"),
        ]

    def insertLinkPoints(self, step: Dict[str, Any], allowDirect: bool) -> None:
        clPoints = step.get("clPoints", [])
        if not clPoints:
            return
        pointsSorted = sorted(clPoints,
                              key=lambda p: (int(p.get("segmentId", 0)),
                                             int(p.get("pointId", 0))))
        maxZ = max(float(p.get("position", [0, 0, 0])[2]) for p in pointsSorted)
        clearanceZ = maxZ + self.safeHeight
        mergedPoints: List[Dict[str, Any]] = []
        lastSegmentId = None
        segEndPt = None
        segEndAxis = None

        for pointItem in pointsSorted:
            segmentId = int(pointItem.get("segmentId", 0))
            ptCopy = deepcopy(pointItem)
            ptCopy["motionType"] = str(ptCopy.get("motionType", "cut"))
            if lastSegmentId is None:
                mergedPoints.append(ptCopy)
                lastSegmentId = segmentId
                segEndPt = ptCopy
                segEndAxis = ptCopy.get("toolAxis", [0.0, 0.0, 1.0])
                continue
            startAxis = ptCopy.get("toolAxis", segEndAxis)
            moveDist = self.computeDistance(
                segEndPt.get("position", [0, 0, 0]),
                ptCopy.get("position", [0, 0, 0]))
            axisChange = self.computeAxisAngle(segEndAxis, startAxis)
            needsLink = (segmentId != lastSegmentId
                         or moveDist > self.directLinkThreshold
                         or axisChange > self.rotationChangeThreshold)
            if needsLink:
                level = self.classifyLink(
                    segEndPt, ptCopy, segEndAxis, startAxis, allowDirect)
                if level == 1:
                    localSafeZ = (max(float(segEndPt["position"][2]),
                                      float(ptCopy["position"][2]))
                                  + self.safeHeight)
                    mergedPoints.extend(self.buildLevel1Link(
                        segEndPt, ptCopy, segEndAxis, startAxis, localSafeZ))
                elif level == 2:
                    mergedPoints.extend(self.buildLevel2Link(
                        segEndPt, ptCopy, segEndAxis, startAxis, clearanceZ))
                elif level == 3:
                    mergedPoints.extend(self.buildLevel3Link(
                        segEndPt, ptCopy, segEndAxis, startAxis,
                        self.rotationSafeZ, clearanceZ))
            mergedPoints.append(ptCopy)
            lastSegmentId = segmentId
            segEndPt = ptCopy
            segEndAxis = ptCopy.get("toolAxis", [0.0, 0.0, 1.0])

        for idx, ptItem in enumerate(mergedPoints, start=1):
            ptItem["pointId"] = idx
            if ptItem.get("motionType") != "cut":
                ptItem["feedrate"] = float(ptItem.get("feedrate", self.linkFeedRate))
        step["clPoints"] = mergedPoints

    def makePoint(self, position: List[float], toolAxis: List[float],
                  motionType: str) -> Dict[str, Any]:
        return {
            "pointId": 0,
            "position": [float(position[0]), float(position[1]), float(position[2])],
            "toolAxis": [float(toolAxis[0]), float(toolAxis[1]), float(toolAxis[2])],
            "feedrate": self.linkFeedRate,
            "segmentId": -1,
            "motionType": motionType,
        }

    @staticmethod
    def computeDistance(p0: List[float], p1: List[float]) -> float:
        dx = float(p1[0]) - float(p0[0])
        dy = float(p1[1]) - float(p0[1])
        dz = float(p1[2]) - float(p0[2])
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    @staticmethod
    def computeAxisAngle(a: List[float], b: List[float]) -> float:
        ax, ay, az = float(a[0]), float(a[1]), float(a[2])
        bx, by, bz = float(b[0]), float(b[1]), float(b[2])
        la = math.sqrt(ax*ax + ay*ay + az*az)
        lb = math.sqrt(bx*bx + by*by + bz*bz)
        if la < 1e-9 or lb < 1e-9:
            return 0.0
        dotVal = max(-1.0, min(1.0, (ax*bx + ay*by + az*bz) / (la*lb)))
        return math.degrees(math.acos(dotVal))
