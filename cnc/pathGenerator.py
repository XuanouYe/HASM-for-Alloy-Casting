import json
from typing import Any, Dict, List, Optional, Tuple
import numpy as np
import trimesh
from cnc.geometryUtils import (applyRotation, buildRotationFromTo, concatenateMeshes,
                            normalizeVector, sampleMeshPointsWithNormals)
from cnc.toolpathEngine import TrimeshToolpathEngine, PointCloudIPW
from cnc.sweptCollision import SweptVolumeCollisionEngine
from cnc.toolpathStrategies import ToolpathStrategyFactory
from cnc.implicitGeometry import SdfVolume, buildSdfVolume, buildOffsetSdf


class FiveAxisCncPathGenerator:
    def __init__(self, version: str = "4.0"):
        self.version = str(version)
        self.toolpathEngine = TrimeshToolpathEngine()

    def loadMesh(self, stlPath: str) -> trimesh.Trimesh:
        meshData = trimesh.load_mesh(stlPath)
        if isinstance(meshData, trimesh.Scene):
            return concatenateMeshes([m for m in meshData.geometry.values()])
        return meshData

    def rotateMesh(self, mesh: trimesh.Trimesh, rotMat: np.ndarray) -> trimesh.Trimesh:
        if mesh is None or mesh.is_empty:
            return concatenateMeshes([])
        rotatedVertices = applyRotation(np.asarray(mesh.vertices, dtype=float), rotMat)
        return trimesh.Trimesh(vertices=rotatedVertices,
                               faces=np.asarray(mesh.faces, dtype=int), process=False)

    def buildClPointDicts(self, positions: np.ndarray, toolAxis: np.ndarray,
                          feedrate: float, segmentId: int,
                          startPointId: int) -> List[Dict[str, Any]]:
        toolAxisUnit = normalizeVector(np.asarray(toolAxis, dtype=float))
        return [
            {
                "pointId": int(startPointId + i),
                "position": [float(v[0]), float(v[1]), float(v[2])],
                "toolAxis": [float(toolAxisUnit[0]), float(toolAxisUnit[1]),
                             float(toolAxisUnit[2])],
                "feedrate": float(feedrate),
                "segmentId": int(segmentId)
            }
            for i, v in enumerate(np.asarray(positions, dtype=float))
        ]

    def buildAxisCoverageTable(self, targetMesh: trimesh.Trimesh,
                                candidateAxes: List[np.ndarray],
                                axisStrategyParams: Dict[str, Any]
                                ) -> Tuple[List[Dict[str, Any]], int]:
        sampleCount = int(axisStrategyParams.get("step3AxisSampleCount", 12000))
        minNormalDot = float(axisStrategyParams.get("step3MinNormalDot", 0.02))
        samplePoints, sampleNormals = sampleMeshPointsWithNormals(targetMesh, sampleCount)
        if len(sampleNormals) == 0:
            return [], 0
        uniqueAxes = []
        for axisItem in candidateAxes:
            axisUnit = normalizeVector(np.asarray(axisItem, dtype=float))
            if not any(abs(float(np.dot(axisUnit, ea))) > 0.999 for ea in uniqueAxes):
                uniqueAxes.append(axisUnit)
        axisCoverages = []
        for axisIndex, axisUnit in enumerate(uniqueAxes):
            dotVals = sampleNormals @ axisUnit
            coveredIndices = set(np.where(dotVals >= minNormalDot)[0].tolist())
            axisCoverages.append({
                "axisIndex": int(axisIndex),
                "axis": axisUnit,
                "coveredSet": coveredIndices
            })
        return axisCoverages, len(samplePoints)

    def pickBestAxis(self, axisCoverages: List[Dict], selectedAxes: List[np.ndarray],
                     coveredUniverse: set, diversityDot: float, enforceDiversity: bool):
        bestAxis = None
        bestNewSet = set()
        bestCount = 0
        for axisData in axisCoverages:
            axisUnit = axisData["axis"]
            if enforceDiversity and selectedAxes:
                if max(abs(float(np.dot(axisUnit, sa))) for sa in selectedAxes) > diversityDot:
                    continue
            newSet = axisData["coveredSet"] - coveredUniverse
            if len(newSet) > bestCount:
                bestCount = len(newSet)
                bestAxis = axisUnit
                bestNewSet = newSet
        return bestAxis, bestNewSet

    def selectAxesGreedyCoverage(self, targetMesh: trimesh.Trimesh,
                                  candidateAxes: List[np.ndarray],
                                  axisStrategyParams: Dict[str, Any]) -> List[np.ndarray]:
        if not candidateAxes:
            return [np.array([0.0, 0.0, 1.0], dtype=float)]
        axisCoverages, totalPoints = self.buildAxisCoverageTable(
            targetMesh, candidateAxes, axisStrategyParams)
        maxAxisCount = int(axisStrategyParams.get("step3AxisCount",
                                                   min(8, len(candidateAxes))))
        targetCoverage = float(axisStrategyParams.get("step3TargetCoverage", 0.995))
        diversityDot = float(axisStrategyParams.get("step3AxisDiversityDot", 0.985))
        if totalPoints == 0 or not axisCoverages:
            return [normalizeVector(np.asarray(candidateAxes[0], dtype=float))]
        selectedAxes = []
        coveredUniverse = set()
        while len(selectedAxes) < maxAxisCount:
            bestAxis, bestNewSet = self.pickBestAxis(
                axisCoverages, selectedAxes, coveredUniverse, diversityDot, True)
            if bestAxis is None:
                bestAxis, bestNewSet = self.pickBestAxis(
                    axisCoverages, selectedAxes, coveredUniverse, diversityDot, False)
            if bestAxis is None or len(bestNewSet) == 0:
                break
            selectedAxes.append(bestAxis)
            coveredUniverse |= bestNewSet
            if len(coveredUniverse) / float(totalPoints) >= targetCoverage:
                break
        if not selectedAxes:
            selectedAxes = [normalizeVector(np.asarray(candidateAxes[0], dtype=float))]
        return selectedAxes

    def _selectStep1Axes(self, axisStrategyParams: Dict[str, Any]) -> List[np.ndarray]:
        tiltAngleDeg = float(axisStrategyParams.get("step1TiltAngleDeg", 35.0))
        tiltCount = int(axisStrategyParams.get("step1TiltCount", 2))
        tiltCount = max(0, min(tiltCount, 2))
        axes = [np.array([0.0, 0.0, 1.0], dtype=float)]
        tiltRad = np.radians(tiltAngleDeg)
        tiltDirs = [
            np.array([np.sin(tiltRad), 0.0, np.cos(tiltRad)], dtype=float),
            np.array([0.0, np.sin(tiltRad), np.cos(tiltRad)], dtype=float),
        ]
        for i in range(tiltCount):
            axes.append(normalizeVector(tiltDirs[i]))
        return axes

    def _filterStep1ByPartSdf(self, stepData: Dict[str, Any],
                               partSdf: SdfVolume,
                               safeClearance: float) -> Dict[str, Any]:
        clPoints = stepData.get("clPoints", [])
        if not clPoints or partSdf.isEmpty:
            return stepData
        sortedPts = sorted(clPoints, key=lambda p: int(p.get("pointId", 0)))
        positions = np.array([p["position"] for p in sortedPts], dtype=float)
        sdVals = partSdf.query(positions)
        segMap: Dict[int, List] = {}
        for i, pt in enumerate(sortedPts):
            sid = int(pt.get("segmentId", -1))
            segMap.setdefault(sid, []).append((i, pt))
        newClPoints = []
        newSegments = []
        newPointId = 0
        newSegId = 0
        for sid, indexedPts in segMap.items():
            currentRun = []
            for origIdx, pt in indexedPts:
                if float(sdVals[origIdx]) >= -safeClearance:
                    currentRun.append(pt)
                else:
                    if len(currentRun) >= 2:
                        for p in currentRun:
                            p["pointId"] = newPointId
                            p["segmentId"] = newSegId
                            newPointId += 1
                        newClPoints.extend(currentRun)
                        newSegments.append({
                            "segmentId": newSegId,
                            "axisIndex": currentRun[0].get("axisIndex", 0),
                            "toolAxis": currentRun[0].get("toolAxis", [0.0, 0.0, 1.0]),
                            "pointCount": len(currentRun)
                        })
                        newSegId += 1
                    currentRun = []
            if len(currentRun) >= 2:
                for p in currentRun:
                    p["pointId"] = newPointId
                    p["segmentId"] = newSegId
                    newPointId += 1
                newClPoints.extend(currentRun)
                newSegments.append({
                    "segmentId": newSegId,
                    "axisIndex": currentRun[0].get("axisIndex", 0),
                    "toolAxis": currentRun[0].get("toolAxis", [0.0, 0.0, 1.0]),
                    "pointCount": len(currentRun)
                })
                newSegId += 1
        stepData["clPoints"] = newClPoints
        stepData["segments"] = newSegments
        return stepData

    def _emitSegments(self, localPaths: List[np.ndarray], axisUnit: np.ndarray,
                      feedrate: float, rotBack: np.ndarray,
                      outputSegmentId: int, pointId: int, axisIndex: int,
                      worldSafeZ: float = -np.inf):
        segments = []
        allClPoints = []
        for pathLocal in localPaths:
            pathArray = np.asarray(pathLocal, dtype=float)
            if len(pathArray) < 2:
                continue
            finalWcsPath = applyRotation(pathArray, rotBack)
            safeRows = finalWcsPath[:, 2] >= worldSafeZ
            currentRun = []
            for i, safe in enumerate(safeRows):
                if safe:
                    currentRun.append(i)
                else:
                    if len(currentRun) >= 2:
                        runWcs = finalWcsPath[currentRun]
                        clPts = self.buildClPointDicts(
                            runWcs, axisUnit, feedrate, outputSegmentId, pointId)
                        pointId += len(clPts)
                        segments.append({
                            "segmentId": int(outputSegmentId),
                            "axisIndex": int(axisIndex),
                            "toolAxis": [float(axisUnit[0]), float(axisUnit[1]),
                                         float(axisUnit[2])],
                            "pointCount": int(len(clPts))
                        })
                        allClPoints.extend(clPts)
                        outputSegmentId += 1
                    currentRun = []
            if len(currentRun) >= 2:
                runWcs = finalWcsPath[currentRun]
                clPts = self.buildClPointDicts(
                    runWcs, axisUnit, feedrate, outputSegmentId, pointId)
                pointId += len(clPts)
                segments.append({
                    "segmentId": int(outputSegmentId),
                    "axisIndex": int(axisIndex),
                    "toolAxis": [float(axisUnit[0]), float(axisUnit[1]),
                                 float(axisUnit[2])],
                    "pointCount": int(len(clPts))
                })
                allClPoints.extend(clPts)
                outputSegmentId += 1
        return segments, allClPoints, outputSegmentId, pointId

    def _emptyStep(self, stepId: int, stepType: str,
                   toolParams: Dict[str, Any]) -> Dict[str, Any]:
        return {
            "stepId": int(stepId),
            "stepType": str(stepType),
            "toolParams": toolParams,
            "motionPolicy": "externalPost",
            "segments": [],
            "clPoints": []
        }

    def _filterClPointsByPartSdf(self, stepData: Dict[str, Any],
                                  partSdf: SdfVolume,
                                  safeClearance: float) -> Dict[str, Any]:
        clPoints = stepData.get("clPoints", [])
        if not clPoints or partSdf.isEmpty:
            return stepData
        sortedPts = sorted(clPoints, key=lambda p: int(p.get("pointId", 0)))
        positions = np.array([p["position"] for p in sortedPts], dtype=float)
        sdVals = partSdf.query(positions)
        segMap: Dict[int, List] = {}
        for i, pt in enumerate(sortedPts):
            sid = int(pt.get("segmentId", -1))
            segMap.setdefault(sid, []).append((i, pt))
        newClPoints = []
        newSegments = []
        newPointId = 0
        newSegId = 0
        for sid, indexedPts in segMap.items():
            currentRun = []
            for origIdx, pt in indexedPts:
                if float(sdVals[origIdx]) >= -safeClearance:
                    currentRun.append(pt)
                else:
                    if len(currentRun) >= 2:
                        for p in currentRun:
                            p["pointId"] = newPointId
                            p["segmentId"] = newSegId
                            newPointId += 1
                        newClPoints.extend(currentRun)
                        newSegments.append({
                            "segmentId": newSegId,
                            "axisIndex": currentRun[0].get("axisIndex", 0),
                            "toolAxis": currentRun[0].get("toolAxis", [0.0, 0.0, 1.0]),
                            "pointCount": len(currentRun)
                        })
                        newSegId += 1
                    currentRun = []
            if len(currentRun) >= 2:
                for p in currentRun:
                    p["pointId"] = newPointId
                    p["segmentId"] = newSegId
                    newPointId += 1
                newClPoints.extend(currentRun)
                newSegments.append({
                    "segmentId": newSegId,
                    "axisIndex": currentRun[0].get("axisIndex", 0),
                    "toolAxis": currentRun[0].get("toolAxis", [0.0, 0.0, 1.0]),
                    "pointCount": len(currentRun)
                })
                newSegId += 1
        stepData["clPoints"] = newClPoints
        stepData["segments"] = newSegments
        return stepData

    def generateStepWithAxes(self, stepId: int, stepType: str,
                              targetMesh: trimesh.Trimesh,
                              collisionEngine: SweptVolumeCollisionEngine,
                              toolParams: Dict[str, Any],
                              stepParam: Dict[str, Any],
                              candidateAxes: List[np.ndarray],
                              globalMinZ: float,
                              safetyMargin: float,
                              worldSafeZ: float = -np.inf) -> Dict[str, Any]:
        modeValue = str(stepParam.get("mode", "dropRaster"))
        feedrate = float(stepParam.get("feedrate", 500.0))
        toolRadius = float(toolParams.get("diameter", 6.0)) * 0.5
        stepOver = float(stepParam.get("stepOver", 1.0))
        isFinishing = modeValue.lower() in {
            "surfacefinishing", "spf", "isoplanarpatchfinishing", "ippf"}
        strategy = ToolpathStrategyFactory.getStrategy(modeValue)
        useIpw = modeValue.lower() in {"zlevelroughing", "zlr", "dropraster"}
        ipwData = (PointCloudIPW(targetMesh, int(stepParam.get("ipwSampleCount", 50000)))
                   if useIpw else None)
        sweepTol = float(stepParam.get("sweepTol", toolRadius * 0.1))
        platformSafeZ = float(globalMinZ + toolRadius + safetyMargin)
        effectiveWorldSafeZ = max(worldSafeZ, platformSafeZ)
        segments = []
        allClPoints = []
        pointId = 0
        outputSegmentId = 0
        for axisIndex, toolAxis in enumerate(candidateAxes):
            axisUnit = normalizeVector(np.asarray(toolAxis, dtype=float))
            rotToToolFrame = buildRotationFromTo(
                axisUnit, np.array([0.0, 0.0, 1.0], dtype=float))
            rotBack = rotToToolFrame.T
            rotatedTarget = self.rotateMesh(targetMesh, rotToToolFrame)
            strategyCtx = dict(stepParam)
            strategyCtx["_toolpathEngine"] = self.toolpathEngine
            strategyCtx["bottomClearance"] = float(
                toolParams.get("bottomClearance", 0.0))
            rawPathsLocal = strategy.generate(
                rotatedTarget, trimesh.Trimesh(), toolRadius, strategyCtx, safetyMargin)
            if collisionEngine is not None:
                rawPathsLocal = collisionEngine.filterPaths(
                    rawPathsLocal, axisUnit, rotBack, sweepTol)
            else:
                rawPathsLocal = [np.asarray(p, dtype=float)
                                 for p in rawPathsLocal if len(p) >= 2]
            rawPathsWcs = [applyRotation(np.asarray(p, dtype=float), rotBack)
                           for p in rawPathsLocal if len(p) >= 2]
            clippedWcs = self.toolpathEngine.clipWcsPathsByZ(
                rawPathsWcs, effectiveWorldSafeZ)
            rotInv = rotToToolFrame
            validPathsLocal = [applyRotation(pw, rotInv) for pw in clippedWcs
                                if len(pw) >= 2]
            if not isFinishing:
                reClipped = []
                for pathLocal in validPathsLocal:
                    reClipped.extend(
                        self.toolpathEngine.slicePathByPlatformZ(
                            pathLocal, rotBack, effectiveWorldSafeZ))
                validPathsLocal = reClipped
            if ipwData is not None and validPathsLocal:
                validPathsLocal = ipwData.filterPathsLocal(
                    validPathsLocal, rotToToolFrame, toolRadius, stepOver)
                ipwData.updateIpwLocal(validPathsLocal, rotToToolFrame, toolRadius)
            if not validPathsLocal:
                continue
            stepSegs, stepPts, outputSegmentId, pointId = self._emitSegments(
                validPathsLocal, axisUnit, feedrate, rotBack,
                outputSegmentId, pointId, axisIndex,
                worldSafeZ=effectiveWorldSafeZ)
            segments.extend(stepSegs)
            allClPoints.extend(stepPts)
        return {
            "stepId": int(stepId),
            "stepType": str(stepType),
            "toolParams": toolParams,
            "motionPolicy": "externalPost",
            "segments": segments,
            "clPoints": allClPoints
        }

    def generateRiserGateStep(self, stepId: int, stepType: str,
                               targetMesh: trimesh.Trimesh,
                               toolParams: Dict[str, Any],
                               stepParam: Dict[str, Any],
                               partSdf: SdfVolume,
                               safeClearance: float,
                               globalMinZ: float,
                               safetyMargin: float,
                               worldSafeZ: float,
                               collisionEngine: Optional[SweptVolumeCollisionEngine] = None) -> Dict[str, Any]:
        toolRadius = float(toolParams.get("diameter", 6.0)) * 0.5
        feedrate = float(stepParam.get("feedrate", 500.0))
        platformSafeZ = float(globalMinZ + toolRadius + safetyMargin)
        effectiveWorldSafeZ = max(worldSafeZ, platformSafeZ)
        removalParams = dict(stepParam)
        removalParams["mode"] = "risergateremoval"
        removalParams.setdefault("stepOver", toolRadius * 1.5)
        removalParams.setdefault("layerStep", toolRadius * 1.5)
        removalParams.setdefault("roughStock", 0.0)
        removalParams["bottomClearance"] = float(toolParams.get("bottomClearance", 0.0))
        strategy = ToolpathStrategyFactory.getStrategy("risergateremoval")
        axisUnit = np.array([0.0, 0.0, 1.0], dtype=float)
        rotToToolFrame = buildRotationFromTo(axisUnit, axisUnit)
        rotBack = rotToToolFrame.T
        sweepTol = float(stepParam.get("sweepTol", toolRadius * 0.1))
        rawPathsLocal = strategy.generate(
            targetMesh, trimesh.Trimesh(), toolRadius, removalParams, safetyMargin)
        if collisionEngine is not None:
            rawPathsLocal = collisionEngine.filterPaths(
                rawPathsLocal, axisUnit, rotBack, sweepTol)
        else:
            rawPathsLocal = [np.asarray(p, dtype=float) for p in rawPathsLocal if len(p) >= 2]
        rawPathsWcs = [applyRotation(np.asarray(p, dtype=float), rotBack)
                       for p in rawPathsLocal if len(p) >= 2]
        clippedWcs = self.toolpathEngine.clipWcsPathsByZ(rawPathsWcs, effectiveWorldSafeZ)
        rotInv = rotToToolFrame
        validPathsLocal = [applyRotation(pw, rotInv) for pw in clippedWcs if len(pw) >= 2]
        reClipped = []
        for pathLocal in validPathsLocal:
            reClipped.extend(
                self.toolpathEngine.slicePathByPlatformZ(
                    pathLocal, rotBack, effectiveWorldSafeZ))
        validPathsLocal = reClipped
        segments = []
        allClPoints = []
        pointId = 0
        outputSegmentId = 0
        if validPathsLocal:
            stepSegs, stepPts, outputSegmentId, pointId = self._emitSegments(
                validPathsLocal, axisUnit, feedrate, rotBack,
                outputSegmentId, pointId, 0,
                worldSafeZ=effectiveWorldSafeZ)
            segments.extend(stepSegs)
            allClPoints.extend(stepPts)
        stepData = {
            "stepId": int(stepId),
            "stepType": str(stepType),
            "toolParams": toolParams,
            "motionPolicy": "externalPost",
            "segments": segments,
            "clPoints": allClPoints
        }
        return self._filterClPointsByPartSdf(stepData, partSdf, safeClearance)

    def generateStep1ShellRemoval(self, toolParams: Dict[str, Any],
                                   stepParam: Dict[str, Any],
                                   moldMesh: trimesh.Trimesh,
                                   partSdf: SdfVolume,
                                   step1Axes: List[np.ndarray],
                                   globalMinZ: float,
                                   safetyMargin: float,
                                   worldSafeZ: float) -> Dict[str, Any]:
        toolRadius = float(toolParams.get("diameter", 6.0)) * 0.5
        feedrate = float(stepParam.get("feedrate", 500.0))
        coarseStepOver = float(stepParam.get("step1StepOver",
                               stepParam.get("stepOver", toolRadius * 1.5)))
        coarseLayerStep = float(stepParam.get("step1LayerStep",
                                stepParam.get("layerStep", toolRadius * 1.5)))
        step1SafeClearance = float(stepParam.get("step1SafeClearance", safetyMargin * 1.5))
        platformSafeZ = float(globalMinZ + toolRadius + safetyMargin)
        effectiveWorldSafeZ = max(worldSafeZ, platformSafeZ)
        coarseParams = dict(stepParam)
        coarseParams["mode"] = "shellRemovalRoughing"
        coarseParams["stepOver"] = coarseStepOver
        coarseParams["layerStep"] = coarseLayerStep
        coarseParams["roughStock"] = float(stepParam.get("roughStock", toolRadius * 0.3))
        coarseParams["step1UseContour"] = bool(stepParam.get("step1UseContour", True))
        coarseParams["step1ContourPasses"] = int(stepParam.get("step1ContourPasses", 2))
        coarseParams["_toolpathEngine"] = self.toolpathEngine
        coarseParams["bottomClearance"] = float(toolParams.get("bottomClearance", 0.0))
        strategy = ToolpathStrategyFactory.getStrategy("shellRemovalRoughing")
        segments = []
        allClPoints = []
        pointId = 0
        outputSegmentId = 0
        for axisIndex, toolAxis in enumerate(step1Axes):
            axisUnit = normalizeVector(np.asarray(toolAxis, dtype=float))
            rotToToolFrame = buildRotationFromTo(
                axisUnit, np.array([0.0, 0.0, 1.0], dtype=float))
            rotBack = rotToToolFrame.T
            rotatedMold = self.rotateMesh(moldMesh, rotToToolFrame)
            rawPathsLocal = strategy.generate(
                rotatedMold, trimesh.Trimesh(), toolRadius, coarseParams, safetyMargin)
            rawPathsLocal = [np.asarray(p, dtype=float) for p in rawPathsLocal if len(p) >= 2]
            rawPathsWcs = [applyRotation(np.asarray(p, dtype=float), rotBack)
                           for p in rawPathsLocal if len(p) >= 2]
            clippedWcs = self.toolpathEngine.clipWcsPathsByZ(rawPathsWcs, effectiveWorldSafeZ)
            rotInv = rotToToolFrame
            validPathsLocal = [applyRotation(pw, rotInv) for pw in clippedWcs if len(pw) >= 2]
            reClipped = []
            for pathLocal in validPathsLocal:
                reClipped.extend(
                    self.toolpathEngine.slicePathByPlatformZ(
                        pathLocal, rotBack, effectiveWorldSafeZ))
            validPathsLocal = reClipped
            if not validPathsLocal:
                continue
            stepSegs, stepPts, outputSegmentId, pointId = self._emitSegments(
                validPathsLocal, axisUnit, feedrate, rotBack,
                outputSegmentId, pointId, axisIndex,
                worldSafeZ=effectiveWorldSafeZ)
            segments.extend(stepSegs)
            allClPoints.extend(stepPts)
        stepData = {
            "stepId": 1,
            "stepType": "shellRemoval",
            "toolParams": toolParams,
            "motionPolicy": "externalPost",
            "segments": segments,
            "clPoints": allClPoints
        }
        return self._filterStep1ByPartSdf(stepData, partSdf, step1SafeClearance)

    def generateJob(self, partStl: str, moldStl: str, gateStl: str, riserStl: str,
                    toolParams: Dict[str, Any], stepParams: List[Dict[str, Any]],
                    axisStrategyParams: Dict[str, Any], wcsId: str = "WCS_MAIN",
                    jobId: Optional[str] = None) -> Dict[str, Any]:
        partMesh = self.loadMesh(partStl)
        moldMesh = self.loadMesh(moldStl)
        gateMesh = self.loadMesh(gateStl)
        riserMesh = self.loadMesh(riserStl)
        meshList = [partMesh, moldMesh, gateMesh, riserMesh]
        globalMinZ = min(
            float(m.bounds[0, 2]) for m in meshList if m is not None and not m.is_empty)
        globalMaxZ = max(
            float(m.bounds[1, 2]) for m in meshList if m is not None and not m.is_empty)
        candidateAxesRaw = axisStrategyParams.get("candidateAxes", [[0.0, 0.0, 1.0]])
        candidateAxes = [normalizeVector(np.asarray(a, dtype=float))
                         for a in candidateAxesRaw]
        safetyMargin = float(toolParams.get("safetyMargin", 0.5))
        toolRadius = float(toolParams.get("diameter", 6.0)) * 0.5
        voxelSize = float(toolParams.get("sdfVoxelSize", max(toolRadius * 0.05, 0.2)))
        backendName = str(toolParams.get("sdfBackend", "auto"))
        bottomSafeOffset = float(toolParams.get("bottomSafeOffset", 1.0))
        stepSafeClearance = float(toolParams.get("stepSafeClearance", safetyMargin))
        gateSafeClearance = float(toolParams.get("gateSafeClearance", safetyMargin * 0.6))
        sweptDiskCount = int(toolParams.get("sweptDiskCount", 16))
        sweptRingCount = int(toolParams.get("sweptRingCount", 6))
        sweptSafeBuffer = float(toolParams.get("sweptSafeBuffer", 2.0))
        worldSafeZ = globalMinZ + bottomSafeOffset
        globalClearanceZ = globalMaxZ + float(toolParams.get("globalRetractHeight", 10.0))
        enableStep1 = bool(axisStrategyParams.get("enableStep1ShellRemoval", True))
        enableStep2 = bool(axisStrategyParams.get("enableStep2RiserRemoval", True))
        enableStep3 = bool(axisStrategyParams.get("enableStep3PartFinishing", True))
        enableStep4 = bool(axisStrategyParams.get("enableStep4GateRemoval", True))
        flatTool = self.toolpathEngine.buildFlatEndMillTool(toolParams)
        sweptClearance = safetyMargin
        partSdf = buildSdfVolume(partMesh, voxelSize, backendName)

        def buildEngine(protectMeshes, clearances):
            sdfList = [buildSdfVolume(m, voxelSize, backendName)
                       for m in protectMeshes]
            return SweptVolumeCollisionEngine(
                flatTool, sdfList, clearances,
                diskCount=sweptDiskCount,
                ringCount=sweptRingCount,
                safeBuffer=sweptSafeBuffer)

        if enableStep1:
            step1Axes = self._selectStep1Axes(axisStrategyParams)
            step1 = self.generateStep1ShellRemoval(
                toolParams, stepParams[0], moldMesh, partSdf,
                step1Axes, globalMinZ, safetyMargin, worldSafeZ)
        else:
            step1 = self._emptyStep(1, "shellRemoval", toolParams)

        if enableStep2:
            engineStep2 = buildEngine([partMesh], [sweptClearance])
            step2 = self.generateRiserGateStep(
                2, "riserRemoval", riserMesh, toolParams, stepParams[1],
                partSdf, stepSafeClearance, globalMinZ, safetyMargin, worldSafeZ,
                collisionEngine=engineStep2)
        else:
            step2 = self._emptyStep(2, "riserRemoval", toolParams)

        if enableStep3:
            finishStock = float(stepParams[2].get("finishStock", 0.03))
            partOuterSdf = buildOffsetSdf(partMesh, -finishStock, voxelSize, backendName)
            engine3 = SweptVolumeCollisionEngine(
                flatTool, [partOuterSdf], [0.0],
                diskCount=sweptDiskCount, ringCount=sweptRingCount,
                safeBuffer=sweptSafeBuffer)
            step3Axes = self.selectAxesGreedyCoverage(
                partMesh, candidateAxes, axisStrategyParams)
            step3ConfigX = dict(stepParams[2])
            step3ConfigX["scanAxis"] = "x"
            step3X = self.generateStepWithAxes(
                3, "partFinishing", partMesh, engine3, toolParams,
                step3ConfigX, step3Axes, globalMinZ, safetyMargin, worldSafeZ)
            step3ConfigY = dict(stepParams[2])
            step3ConfigY["scanAxis"] = "y"
            step3Y = self.generateStepWithAxes(
                3, "partFinishing", partMesh, engine3, toolParams,
                step3ConfigY, step3Axes, globalMinZ, safetyMargin, worldSafeZ)
            step3 = {
                "stepId": 3,
                "stepType": "partFinishing",
                "toolParams": step3X["toolParams"],
                "motionPolicy": "externalPost",
                "segments": step3X["segments"] + step3Y["segments"],
                "clPoints": step3X["clPoints"] + step3Y["clPoints"]
            }
        else:
            step3 = self._emptyStep(3, "partFinishing", toolParams)

        if enableStep4:
            engineStep4 = buildEngine([partMesh], [sweptClearance])
            step4 = self.generateRiserGateStep(
                4, "gateRemoval", gateMesh, toolParams, stepParams[3],
                partSdf, gateSafeClearance, globalMinZ, safetyMargin, worldSafeZ,
                collisionEngine=engineStep4)
        else:
            step4 = self._emptyStep(4, "gateRemoval", toolParams)

        outputData = {
            "version": self.version,
            "wcsId": str(wcsId),
            "globalClearanceZ": float(globalClearanceZ),
            "steps": [step1, step2, step3, step4]
        }
        if jobId is not None:
            outputData["jobId"] = str(jobId)
        return outputData

    def exportClJson(self, clData: Dict[str, Any], outputPath: str) -> None:
        with open(outputPath, "w", encoding="utf-8") as f:
            json.dump(clData, f, ensure_ascii=False, indent=2)
