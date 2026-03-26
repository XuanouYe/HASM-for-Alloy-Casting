import math
from copy import deepcopy
from typing import Any, Dict, List, Tuple


class ClPathLinker:
    def __init__(self, linkerConfig: Dict[str, Any]):
        self.safeHeight = float(linkerConfig.get("safeHeight", 5.0))
        self.maxRetractOffset = float(linkerConfig.get("maxRetractOffset", 100.0))
        self.directLinkThreshold = float(linkerConfig.get("directLinkThreshold", 2.0))
        self.rotationChangeThreshold = float(linkerConfig.get("rotationChangeThreshold", 5.0))
        self.rotationRetractAngle = float(linkerConfig.get("rotationRetractAngle", 30.0))
        self.rotationSafeZ = float(linkerConfig.get("rotationSafeZ", 30.0))
        self.linkFeedRate = float(linkerConfig.get("linkFeedRate", 2000.0))
        self.stepLinkingEnabled = linkerConfig.get("stepLinkingEnabled", {})

    def processClData(self, clData: Dict[str, Any]) -> Dict[str, Any]:
        linkedData = deepcopy(clData)
        stepsList = linkedData.get("steps", [])
        for stepItem in stepsList:
            stepType = str(stepItem.get("stepType", ""))
            allowDirect = bool(self.stepLinkingEnabled.get(stepType, True))
            self.insertLinkPoints(stepItem, allowDirect)
        return linkedData

    def classifyLink(self, endPt: Dict[str, Any], startPt: Dict[str, Any], endAxis: List[float], startAxis: List[float], allowDirect: bool) -> int:
        endPos = endPt.get("position", [0.0, 0.0, 0.0])
        startPos = startPt.get("position", [0.0, 0.0, 0.0])
        moveDist = self.computeDistance(endPos, startPos)
        axisChange = self.computeAxisAngle(endAxis, startAxis)
        if axisChange >= self.rotationRetractAngle:
            return 3
        if moveDist <= self.directLinkThreshold and axisChange <= self.rotationChangeThreshold and allowDirect:
            return 0
        if moveDist <= self.maxRetractOffset and axisChange <= self.rotationChangeThreshold:
            return 1
        return 2

    def computeLocalSafeZ(self, endPt: Dict[str, Any], startPt: Dict[str, Any], safeHeight: float) -> float:
        endZ = float(endPt.get("position", [0.0, 0.0, 0.0])[2])
        startZ = float(startPt.get("position", [0.0, 0.0, 0.0])[2])
        return max(endZ, startZ) + safeHeight

    def buildLevel1Link(self, endPt: Dict[str, Any], startPt: Dict[str, Any], toolAxis: List[float], localSafeZ: float) -> List[Dict[str, Any]]:
        endPos = endPt.get("position", [0.0, 0.0, 0.0])
        startPos = startPt.get("position", [0.0, 0.0, 0.0])
        return [
            self.makePoint([endPos[0], endPos[1], localSafeZ], toolAxis, "retract"),
            self.makePoint([startPos[0], startPos[1], localSafeZ], toolAxis, "rapid"),
            self.makePoint([startPos[0], startPos[1], startPos[2]], toolAxis, "approach"),
        ]

    def buildLevel2Link(self, endPt: Dict[str, Any], startPt: Dict[str, Any], endAxis: List[float], startAxis: List[float], clearanceZ: float) -> List[Dict[str, Any]]:
        endPos = endPt.get("position", [0.0, 0.0, 0.0])
        startPos = startPt.get("position", [0.0, 0.0, 0.0])
        return [
            self.makePoint([endPos[0], endPos[1], clearanceZ], endAxis, "retract"),
            self.makePoint([startPos[0], startPos[1], clearanceZ], endAxis, "rapid"),
            self.makePoint([startPos[0], startPos[1], startPos[2]], startAxis, "approach"),
        ]

    def buildLevel3Link(self, endPt: Dict[str, Any], startPt: Dict[str, Any], endAxis: List[float], startAxis: List[float], rotationSafeZ: float, clearanceZ: float) -> List[Dict[str, Any]]:
        endPos = endPt.get("position", [0.0, 0.0, 0.0])
        startPos = startPt.get("position", [0.0, 0.0, 0.0])
        highZ = max(rotationSafeZ, clearanceZ)
        return [
            self.makePoint([endPos[0], endPos[1], highZ], endAxis, "retract"),
            self.makePoint([endPos[0], endPos[1], highZ], startAxis, "rapid"),
            self.makePoint([startPos[0], startPos[1], highZ], startAxis, "rapid"),
            self.makePoint([startPos[0], startPos[1], startPos[2]], startAxis, "approach"),
        ]

    def insertLinkPoints(self, step: Dict[str, Any], allowDirect: bool) -> None:
        clPoints = step.get("clPoints", [])
        if not clPoints:
            return
        pointsSorted = sorted(clPoints, key=lambda pointItem: (int(pointItem.get("segmentId", 0)), int(pointItem.get("pointId", 0))))
        maxZ = max(float(pointItem.get("position", [0.0, 0.0, 0.0])[2]) for pointItem in pointsSorted)
        clearanceZ = maxZ + self.safeHeight
        mergedPoints: List[Dict[str, Any]] = []
        lastSegmentId = None
        segmentEndPoint = None
        segmentEndAxis = None

        for pointItem in pointsSorted:
            segmentId = int(pointItem.get("segmentId", 0))
            pointCopy = deepcopy(pointItem)
            pointCopy["motionType"] = str(pointCopy.get("motionType", "cut"))
            if lastSegmentId is None:
                mergedPoints.append(pointCopy)
                lastSegmentId = segmentId
                segmentEndPoint = pointCopy
                segmentEndAxis = pointCopy.get("toolAxis", [0.0, 0.0, 1.0])
                continue
            if segmentEndPoint is not None and segmentEndAxis is not None:
                startAxis = pointCopy.get("toolAxis", segmentEndAxis)
                moveDist = self.computeDistance(segmentEndPoint.get("position", [0.0, 0.0, 0.0]), pointCopy.get("position", [0.0, 0.0, 0.0]))
                axisChange = self.computeAxisAngle(segmentEndAxis, startAxis)
                shouldLink = segmentId != lastSegmentId or moveDist > self.directLinkThreshold or axisChange > self.rotationChangeThreshold
                if shouldLink:
                    linkLevel = self.classifyLink(segmentEndPoint, pointCopy, segmentEndAxis, startAxis, allowDirect)
                else:
                    linkLevel = 0
                if linkLevel == 1:
                    localSafeZ = self.computeLocalSafeZ(segmentEndPoint, pointCopy, self.safeHeight)
                    mergedPoints.extend(self.buildLevel1Link(segmentEndPoint, pointCopy, segmentEndAxis, localSafeZ))
                elif linkLevel == 2:
                    mergedPoints.extend(self.buildLevel2Link(segmentEndPoint, pointCopy, segmentEndAxis, startAxis, clearanceZ))
                elif linkLevel == 3:
                    mergedPoints.extend(self.buildLevel3Link(segmentEndPoint, pointCopy, segmentEndAxis, startAxis, self.rotationSafeZ, clearanceZ))
            mergedPoints.append(pointCopy)
            lastSegmentId = segmentId
            segmentEndPoint = pointCopy
            segmentEndAxis = pointCopy.get("toolAxis", [0.0, 0.0, 1.0])

        for idx, pointItem in enumerate(mergedPoints, start=1):
            pointItem["pointId"] = idx
            if pointItem.get("motionType") != "cut":
                pointItem["feedrate"] = float(pointItem.get("feedrate", self.linkFeedRate))
        step["clPoints"] = mergedPoints

    def makePoint(self, position: List[float], toolAxis: List[float], motionType: str) -> Dict[str, Any]:
        return {
            "pointId": 0,
            "position": [float(position[0]), float(position[1]), float(position[2])],
            "toolAxis": [float(toolAxis[0]), float(toolAxis[1]), float(toolAxis[2])],
            "feedrate": self.linkFeedRate,
            "segmentId": -1,
            "motionType": motionType,
        }

    @staticmethod
    def computeDistance(startPos: List[float], endPos: List[float]) -> float:
        dx = float(endPos[0]) - float(startPos[0])
        dy = float(endPos[1]) - float(startPos[1])
        dz = float(endPos[2]) - float(startPos[2])
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    @staticmethod
    def computeAxisAngle(endAxis: List[float], startAxis: List[float]) -> float:
        ex, ey, ez = [float(val) for val in endAxis]
        sx, sy, sz = [float(val) for val in startAxis]
        endLen = math.sqrt(ex * ex + ey * ey + ez * ez)
        startLen = math.sqrt(sx * sx + sy * sy + sz * sz)
        if endLen <= 0.0 or startLen <= 0.0:
            return 0.0
        dotVal = (ex * sx + ey * sy + ez * sz) / (endLen * startLen)
        dotVal = max(-1.0, min(1.0, dotVal))
        return math.degrees(math.acos(dotVal))
