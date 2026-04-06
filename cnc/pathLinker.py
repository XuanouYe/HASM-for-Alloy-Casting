import math
from copy import deepcopy
from typing import Any, Dict, List, Optional
import numpy as np


def resolveOutwardAxis(toolAxis: List[float], pos: List[float],
                       collisionMesh) -> List[float]:
    ax, ay, az = [float(v) for v in toolAxis]
    axisLen = math.sqrt(ax * ax + ay * ay + az * az)
    if axisLen < 1e-9:
        return [0.0, 0.0, 1.0]
    ax, ay, az = ax / axisLen, ay / axisLen, az / axisLen

    if collisionMesh is None:
        return [ax, ay, az]

    try:
        import trimesh
        rayOrigin = np.array([[pos[0], pos[1], pos[2]]], dtype=float)
        fwdDir = np.array([[ax, ay, az]], dtype=float)
        bwdDir = np.array([[-ax, -ay, -az]], dtype=float)
        intersector = trimesh.ray.ray_triangle.RayMeshIntersector(collisionMesh)
        fwdHits = intersector.intersects_any(rayOrigin, fwdDir)
        bwdHits = intersector.intersects_any(rayOrigin, bwdDir)
        if fwdHits[0] and not bwdHits[0]:
            return [-ax, -ay, -az]
        return [ax, ay, az]
    except Exception:
        return [ax, ay, az]


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
        self._intersector = None
        if self.retractCollisionMesh is not None:
            try:
                import trimesh
                self._intersector = trimesh.ray.ray_triangle.RayMeshIntersector(
                    self.retractCollisionMesh)
            except Exception:
                self._intersector = None

    def processClData(self, clData: Dict[str, Any]) -> Dict[str, Any]:
        linkedData = deepcopy(clData)
        stepsList = linkedData.get("steps", [])
        for stepItem in stepsList:
            stepType = str(stepItem.get("stepType", ""))
            allowDirect = bool(self.stepLinkingEnabled.get(stepType, True))
            self.insertLinkPoints(stepItem, allowDirect)
        return linkedData

    def classifyLink(self, endPt: Dict[str, Any], startPt: Dict[str, Any],
                     endAxis: List[float], startAxis: List[float], allowDirect: bool) -> int:
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

    def computeLocalSafeZ(self, endPt: Dict[str, Any], startPt: Dict[str, Any],
                          safeHeight: float) -> float:
        endZ = float(endPt.get("position", [0.0, 0.0, 0.0])[2])
        startZ = float(startPt.get("position", [0.0, 0.0, 0.0])[2])
        return max(endZ, startZ) + safeHeight

    def outwardAxisAt(self, pos: List[float], toolAxis: List[float]) -> List[float]:
        return resolveOutwardAxis(toolAxis, pos, self.retractCollisionMesh)

    def computeRetractPointAlongAxis(self, pos: List[float], outwardAxis: List[float],
                                     retractLength: float) -> List[float]:
        ax, ay, az = [float(v) for v in outwardAxis]
        axisLen = math.sqrt(ax * ax + ay * ay + az * az)
        if axisLen < 1e-9:
            ax, ay, az = 0.0, 0.0, 1.0
        else:
            ax, ay, az = ax / axisLen, ay / axisLen, az / axisLen
        return [
            float(pos[0]) + ax * retractLength,
            float(pos[1]) + ay * retractLength,
            float(pos[2]) + az * retractLength,
        ]

    def safeRetractLength(self, pos: List[float], outwardAxis: List[float],
                          desiredLength: float) -> float:
        if self._intersector is None:
            return desiredLength
        try:
            import vtk
            import pyvista as pv
            verts = np.array(self.retractCollisionMesh.vertices)
            faces = self.retractCollisionMesh.faces
            pvFaces = np.column_stack((np.full(len(faces), 3), faces)).flatten()
            pvMesh = pv.PolyData(verts, pvFaces)
            tree = vtk.vtkOBBTree()
            tree.SetDataSet(pvMesh)
            tree.BuildLocator()
            endPt = self.computeRetractPointAlongAxis(pos, outwardAxis, desiredLength)
            intersectPts = vtk.vtkPoints()
            code = tree.IntersectWithLine(pos, endPt, intersectPts, None)
            if code == 0:
                return desiredLength
            closestDist = desiredLength
            for i in range(intersectPts.GetNumberOfPoints()):
                iPt = intersectPts.GetPoint(i)
                d = math.sqrt(sum((iPt[k] - float(pos[k])) ** 2 for k in range(3)))
                if d < closestDist:
                    closestDist = max(0.0, d - 0.5)
            return closestDist
        except Exception:
            return desiredLength

    def buildAxisRetractPoint(self, pos: List[float], toolAxis: List[float],
                              desiredLength: float) -> List[float]:
        outAxis = self.outwardAxisAt(pos, toolAxis)
        safeLen = self.safeRetractLength(pos, outAxis, desiredLength)
        return self.computeRetractPointAlongAxis(pos, outAxis, safeLen)

    def buildLevel1Link(self, endPt: Dict[str, Any], startPt: Dict[str, Any],
                        endAxis: List[float], startAxis: List[float],
                        localSafeZ: float) -> List[Dict[str, Any]]:
        endPos = endPt.get("position", [0.0, 0.0, 0.0])
        startPos = startPt.get("position", [0.0, 0.0, 0.0])
        if self.retractAlongAxis:
            retractEndPos = self.buildAxisRetractPoint(endPos, endAxis, self.retractAxisLength)
            retractStartPos = self.buildAxisRetractPoint(startPos, startAxis, self.retractAxisLength)
            return [
                self.makePoint(retractEndPos, endAxis, "retract"),
                self.makePoint(retractStartPos, startAxis, "rapid"),
                self.makePoint(startPos, startAxis, "approach"),
            ]
        return [
            self.makePoint([endPos[0], endPos[1], localSafeZ], endAxis, "retract"),
            self.makePoint([startPos[0], startPos[1], localSafeZ], endAxis, "rapid"),
            self.makePoint(startPos, startAxis, "approach"),
        ]

    def buildLevel2Link(self, endPt: Dict[str, Any], startPt: Dict[str, Any],
                        endAxis: List[float], startAxis: List[float],
                        clearanceZ: float) -> List[Dict[str, Any]]:
        endPos = endPt.get("position", [0.0, 0.0, 0.0])
        startPos = startPt.get("position", [0.0, 0.0, 0.0])
        if self.retractAlongAxis:
            retractEndPos = self.buildAxisRetractPoint(endPos, endAxis, self.retractAxisLength)
            retractStartPos = self.buildAxisRetractPoint(startPos, startAxis, self.retractAxisLength)
            return [
                self.makePoint(retractEndPos, endAxis, "retract"),
                self.makePoint(retractStartPos, startAxis, "rapid"),
                self.makePoint(startPos, startAxis, "approach"),
            ]
        return [
            self.makePoint([endPos[0], endPos[1], clearanceZ], endAxis, "retract"),
            self.makePoint([startPos[0], startPos[1], clearanceZ], endAxis, "rapid"),
            self.makePoint(startPos, startAxis, "approach"),
        ]

    def buildLevel3Link(self, endPt: Dict[str, Any], startPt: Dict[str, Any],
                        endAxis: List[float], startAxis: List[float],
                        rotationSafeZ: float, clearanceZ: float) -> List[Dict[str, Any]]:
        endPos = endPt.get("position", [0.0, 0.0, 0.0])
        startPos = startPt.get("position", [0.0, 0.0, 0.0])
        if self.retractAlongAxis:
            longRetract = self.retractAxisLength * 2.0
            retractEndPos = self.buildAxisRetractPoint(endPos, endAxis, longRetract)
            retractStartPos = self.buildAxisRetractPoint(startPos, startAxis, longRetract)
            return [
                self.makePoint(retractEndPos, endAxis, "retract"),
                self.makePoint(retractEndPos, startAxis, "rapid"),
                self.makePoint(retractStartPos, startAxis, "rapid"),
                self.makePoint(startPos, startAxis, "approach"),
            ]
        highZ = max(rotationSafeZ, clearanceZ)
        return [
            self.makePoint([endPos[0], endPos[1], highZ], endAxis, "retract"),
            self.makePoint([endPos[0], endPos[1], highZ], startAxis, "rapid"),
            self.makePoint([startPos[0], startPos[1], highZ], startAxis, "rapid"),
            self.makePoint(startPos, startAxis, "approach"),
        ]

    def insertLinkPoints(self, step: Dict[str, Any], allowDirect: bool) -> None:
        clPoints = step.get("clPoints", [])
        if not clPoints:
            return
        pointsSorted = sorted(clPoints, key=lambda p: (int(p.get("segmentId", 0)),
                                                        int(p.get("pointId", 0))))
        maxZ = max(float(p.get("position", [0.0, 0.0, 0.0])[2]) for p in pointsSorted)
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
                moveDist = self.computeDistance(segmentEndPoint.get("position", [0.0, 0.0, 0.0]),
                                                pointCopy.get("position", [0.0, 0.0, 0.0]))
                axisChange = self.computeAxisAngle(segmentEndAxis, startAxis)
                shouldLink = (segmentId != lastSegmentId
                              or moveDist > self.directLinkThreshold
                              or axisChange > self.rotationChangeThreshold)
                linkLevel = (self.classifyLink(segmentEndPoint, pointCopy,
                                               segmentEndAxis, startAxis, allowDirect)
                             if shouldLink else 0)
                if linkLevel == 1:
                    localSafeZ = self.computeLocalSafeZ(segmentEndPoint, pointCopy, self.safeHeight)
                    mergedPoints.extend(self.buildLevel1Link(
                        segmentEndPoint, pointCopy, segmentEndAxis, startAxis, localSafeZ))
                elif linkLevel == 2:
                    mergedPoints.extend(self.buildLevel2Link(
                        segmentEndPoint, pointCopy, segmentEndAxis, startAxis, clearanceZ))
                elif linkLevel == 3:
                    mergedPoints.extend(self.buildLevel3Link(
                        segmentEndPoint, pointCopy, segmentEndAxis, startAxis,
                        self.rotationSafeZ, clearanceZ))
            mergedPoints.append(pointCopy)
            lastSegmentId = segmentId
            segmentEndPoint = pointCopy
            segmentEndAxis = pointCopy.get("toolAxis", [0.0, 0.0, 1.0])

        for idx, pointItem in enumerate(mergedPoints, start=1):
            pointItem["pointId"] = idx
            if pointItem.get("motionType") != "cut":
                pointItem["feedrate"] = float(pointItem.get("feedrate", self.linkFeedRate))
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
    def computeDistance(startPos: List[float], endPos: List[float]) -> float:
        dx = float(endPos[0]) - float(startPos[0])
        dy = float(endPos[1]) - float(startPos[1])
        dz = float(endPos[2]) - float(startPos[2])
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    @staticmethod
    def computeAxisAngle(endAxis: List[float], startAxis: List[float]) -> float:
        ex, ey, ez = [float(v) for v in endAxis]
        sx, sy, sz = [float(v) for v in startAxis]
        endLen = math.sqrt(ex * ex + ey * ey + ez * ez)
        startLen = math.sqrt(sx * sx + sy * sy + sz * sz)
        if endLen <= 0.0 or startLen <= 0.0:
            return 0.0
        dotVal = (ex * sx + ey * sy + ez * sz) / (endLen * startLen)
        dotVal = max(-1.0, min(1.0, dotVal))
        return math.degrees(math.acos(dotVal))
