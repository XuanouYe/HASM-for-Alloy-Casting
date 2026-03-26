import json
from typing import Any, Dict, List, Optional, Tuple
import numpy as np
import trimesh
from .geometryUtils import applyRotation, buildRotationFromTo, concatenateMeshes, normalizeVector, sampleMeshPointsWithNormals
from .toolpathEngine import TrimeshToolpathEngine, PointCloudIPW, MeshCollisionChecker, SolidKeepOutClipper, SafeEnvelope
from .toolpathStrategies import ToolpathStrategyFactory

class FiveAxisCncPathGenerator:
    def __init__(self, version: str = '2.5'):
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
        return trimesh.Trimesh(vertices=rotatedVertices, faces=np.asarray(mesh.faces, dtype=int), process=False)

    def buildClPointDicts(self, positions: np.ndarray, toolAxis: np.ndarray, feedrate: float, segmentId: int, startPointId: int) -> List[Dict[str, Any]]:
        toolAxisUnit = normalizeVector(np.asarray(toolAxis, dtype=float))
        return [
            {
                'pointId': int(startPointId + pointIndex),
                'position': [float(v[0]), float(v[1]), float(v[2])],
                'toolAxis': [float(toolAxisUnit[0]), float(toolAxisUnit[1]), float(toolAxisUnit[2])],
                'feedrate': float(feedrate),
                'segmentId': int(segmentId)
            }
            for pointIndex, v in enumerate(np.asarray(positions, dtype=float))
        ]

    def buildAxisCoverageTable(self, targetMesh: trimesh.Trimesh, candidateAxes: List[np.ndarray], axisStrategyParams: Dict[str, Any]) -> Tuple[List[Dict[str, Any]], int]:
        sampleCount = int(axisStrategyParams.get('step3AxisSampleCount', 12000))
        minNormalDot = float(axisStrategyParams.get('step3MinNormalDot', 0.02))
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
            dotValues = sampleNormals @ axisUnit
            coveredIndices = set(np.where(dotValues >= minNormalDot)[0].tolist())
            axisCoverages.append({'axisIndex': int(axisIndex), 'axis': axisUnit, 'coveredSet': coveredIndices})
        return axisCoverages, len(samplePoints)

    def pickBestAxis(self, axisCoverages: List[Dict[str, Any]], selectedAxes: List[np.ndarray], coveredUniverse: set, diversityDot: float, enforceDiversity: bool) -> Tuple[Optional[np.ndarray], set]:
        bestAxis = None
        bestNewSet = set()
        bestCoverageCount = 0
        for axisData in axisCoverages:
            axisUnit = axisData['axis']
            if enforceDiversity and selectedAxes:
                if max(abs(float(np.dot(axisUnit, sa))) for sa in selectedAxes) > diversityDot:
                    continue
            newCoverageSet = axisData['coveredSet'] - coveredUniverse
            if len(newCoverageSet) > bestCoverageCount:
                bestCoverageCount = len(newCoverageSet)
                bestAxis = axisUnit
                bestNewSet = newCoverageSet
        return bestAxis, bestNewSet

    def selectAxesGreedyCoverage(self, targetMesh: trimesh.Trimesh, candidateAxes: List[np.ndarray], axisStrategyParams: Dict[str, Any]) -> List[np.ndarray]:
        if not candidateAxes:
            return [np.array([0.0, 0.0, 1.0], dtype=float)]
        axisCoverages, totalPoints = self.buildAxisCoverageTable(targetMesh, candidateAxes, axisStrategyParams)
        maxAxisCount = int(axisStrategyParams.get('step3AxisCount', min(8, len(candidateAxes))))
        targetCoverage = float(axisStrategyParams.get('step3TargetCoverage', 0.995))
        diversityDot = float(axisStrategyParams.get('step3AxisDiversityDot', 0.985))
        if totalPoints == 0 or not axisCoverages:
            return [normalizeVector(np.asarray(candidateAxes[0], dtype=float))]
        selectedAxes = []
        coveredUniverse = set()
        while len(selectedAxes) < maxAxisCount:
            bestAxis, bestNewSet = self.pickBestAxis(axisCoverages, selectedAxes, coveredUniverse, diversityDot, True)
            if bestAxis is None:
                bestAxis, bestNewSet = self.pickBestAxis(axisCoverages, selectedAxes, coveredUniverse, diversityDot, False)
            if bestAxis is None or len(bestNewSet) == 0:
                break
            selectedAxes.append(bestAxis)
            coveredUniverse |= bestNewSet
            if len(coveredUniverse) / float(totalPoints) >= targetCoverage:
                break
        if not selectedAxes:
            selectedAxes = [normalizeVector(np.asarray(candidateAxes[0], dtype=float))]
        return selectedAxes

    def _emitSegments(self, localPaths: List[np.ndarray], axisUnit: np.ndarray, feedrate: float, rotBack: np.ndarray, outputSegmentId: int, pointId: int, axisIndex: int) -> Tuple[List[Dict], List[Dict], int, int]:
        segments = []
        allClPoints = []
        for pathLocal in localPaths:
            pathArray = np.asarray(pathLocal, dtype=float)
            if len(pathArray) < 2:
                continue
            finalWcsPath = applyRotation(pathArray, rotBack)
            clPointsWcs = self.buildClPointDicts(finalWcsPath, axisUnit, feedrate, outputSegmentId, pointId)
            pointId += len(clPointsWcs)
            segments.append({
                'segmentId': int(outputSegmentId),
                'axisIndex': int(axisIndex),
                'toolAxis': [float(axisUnit[0]), float(axisUnit[1]), float(axisUnit[2])],
                'pointCount': int(len(clPointsWcs))
            })
            allClPoints.extend(clPointsWcs)
            outputSegmentId += 1
        return segments, allClPoints, outputSegmentId, pointId

    def generateStepWithAxes(self, stepId: int, stepType: str, targetMesh: trimesh.Trimesh, keepOutMesh: trimesh.Trimesh, avoidanceMesh: Optional[trimesh.Trimesh], toolParams: Dict[str, Any], stepParam: Dict[str, Any], candidateAxes: List[np.ndarray], globalMinZ: float, safetyMargin: float, solidClipMesh: Optional[trimesh.Trimesh] = None) -> Dict[str, Any]:
        modeValue = str(stepParam.get('mode', 'dropRaster'))
        feedrate = float(stepParam.get('feedrate', 500.0))
        toolRadius = float(toolParams.get('diameter', 6.0)) * 0.5
        platformSafeZ = float(globalMinZ + toolRadius + safetyMargin)
        stepOver = float(stepParam.get('stepOver', 1.0))
        isFinishing = modeValue.lower() in {'surfacefinishing', 'spf'}
        strategy = ToolpathStrategyFactory.getStrategy(modeValue)
        useIpw = modeValue.lower() in {'zlevelroughing', 'zlr', 'dropraster'}
        ipwData = PointCloudIPW(targetMesh, int(stepParam.get('ipwSampleCount', 50000))) if useIpw else None
        hmGridStep = float(stepParam.get('hmGridStep', max(stepOver * 0.6, toolRadius * 0.5)))
        hmSampleStep = float(stepParam.get('hmSampleStep', max(stepOver * 0.4, toolRadius * 0.35)))
        clearance = float(toolRadius + safetyMargin)

        segments = []
        allClPoints = []
        pointId = 0
        outputSegmentId = 0

        for axisIndex, toolAxis in enumerate(candidateAxes):
            axisUnit = normalizeVector(np.asarray(toolAxis, dtype=float))
            rotToToolFrame = buildRotationFromTo(axisUnit, np.array([0.0, 0.0, 1.0], dtype=float))
            rotBack = rotToToolFrame.T

            rotatedTarget = self.rotateMesh(targetMesh, rotToToolFrame)
            rotatedKeepOut = self.rotateMesh(keepOutMesh, rotToToolFrame)
            rotatedAvoidance = self.rotateMesh(avoidanceMesh, rotToToolFrame) if avoidanceMesh is not None and not avoidanceMesh.is_empty else None
            rotatedSolidClip = self.rotateMesh(solidClipMesh, rotToToolFrame) if solidClipMesh is not None and not solidClipMesh.is_empty else None

            keepOutChecker = MeshCollisionChecker(rotatedKeepOut, toolRadius, safetyMargin)
            hmObstacleMesh = rotatedAvoidance if rotatedAvoidance is not None else rotatedKeepOut
            heightMapLocal = self.toolpathEngine.buildObstacleHeightMapLocal(hmObstacleMesh, hmGridStep, toolRadius, safetyMargin)
            solidClipperLocal = SolidKeepOutClipper(rotatedSolidClip) if rotatedSolidClip is not None else None

            strategyCtx = dict(stepParam)
            strategyCtx['_keepOutChecker'] = keepOutChecker
            strategyCtx['_toolpathEngine'] = self.toolpathEngine
            strategyCtx['_hmSampleStep'] = hmSampleStep

            rawPathsLocal = strategy.generate(rotatedTarget, rotatedKeepOut, toolRadius, strategyCtx, safetyMargin)
            rawPathsLocal = self.toolpathEngine.clipPathsByCollisionChecker(rawPathsLocal, keepOutChecker, hmSampleStep)

            if heightMapLocal is not None:
                rawPathsLocal = self.toolpathEngine.clipPathsByObstacleLocal(rawPathsLocal, heightMapLocal, hmSampleStep, clearance)

            if solidClipperLocal is not None:
                rawPathsLocal = solidClipperLocal.clipPaths(rawPathsLocal, hmSampleStep)

            validPathsLocal = []
            for pathLocal in rawPathsLocal:
                if len(pathLocal) < 2:
                    continue
                if isFinishing:
                    validPathsLocal.append(np.asarray(pathLocal, dtype=float))
                else:
                    validPathsLocal.extend(self.toolpathEngine.slicePathByPlatformZ(pathLocal, rotBack, platformSafeZ))

            if ipwData is not None and validPathsLocal:
                validPathsLocal = ipwData.filterPathsLocal(validPathsLocal, rotToToolFrame, toolRadius, stepOver)
                ipwData.updateIpwLocal(validPathsLocal, rotToToolFrame, toolRadius)

            if not validPathsLocal:
                continue

            stepSegs, stepPts, outputSegmentId, pointId = self._emitSegments(
                validPathsLocal, axisUnit, feedrate, rotBack, outputSegmentId, pointId, axisIndex
            )

            segments.extend(stepSegs)
            allClPoints.extend(stepPts)

        return {
            'stepId': int(stepId),
            'stepType': str(stepType),
            'toolParams': toolParams,
            'motionPolicy': 'externalPost',
            'segments': segments,
            'clPoints': allClPoints
        }

    def generateJob(self, partStl: str, moldStl: str, gateStl: str, riserStl: str, toolParams: Dict[str, Any], stepParams: List[Dict[str, Any]], axisStrategyParams: Dict[str, Any], wcsId: str = 'WCS_MAIN', jobId: Optional[str] = None) -> Dict[str, Any]:
        partMesh = self.loadMesh(partStl)
        moldMesh = self.loadMesh(moldStl)
        gateMesh = self.loadMesh(gateStl)
        riserMesh = self.loadMesh(riserStl)
        meshList = [partMesh, moldMesh, gateMesh, riserMesh]
        globalMinZ = min(float(m.bounds[0, 2]) for m in meshList if m is not None and not m.is_empty)
        candidateAxesRaw = axisStrategyParams.get('candidateAxes', [[0.0, 0.0, 1.0]])
        candidateAxes = [normalizeVector(np.asarray(a, dtype=float)) for a in candidateAxesRaw]
        safetyMargin = float(toolParams.get('safetyMargin', 0.5))

        keepOutMeshStep1 = concatenateMeshes([partMesh, gateMesh, riserMesh])
        step1 = self.generateStepWithAxes(
            1, 'shellRemoval',
            moldMesh, keepOutMeshStep1, None,
            toolParams, stepParams[0], candidateAxes, globalMinZ, safetyMargin,
            solidClipMesh=keepOutMeshStep1
        )

        keepOutMeshStep2 = concatenateMeshes([partMesh, gateMesh])
        step2 = self.generateStepWithAxes(
            2, 'riserRemoval',
            riserMesh, keepOutMeshStep2, None,
            toolParams, stepParams[1], [np.array([0.0, 0.0, 1.0], dtype=float)], globalMinZ, safetyMargin,
            solidClipMesh=keepOutMeshStep2
        )

        step3Axes = self.selectAxesGreedyCoverage(partMesh, candidateAxes, axisStrategyParams)
        step3ConfigX = stepParams[2].copy()
        step3ConfigX['scanAxis'] = 'x'
        step3X = self.generateStepWithAxes(
            3, 'partFinishing',
            partMesh, gateMesh, None,
            toolParams, step3ConfigX, step3Axes, globalMinZ, safetyMargin,
            solidClipMesh=None
        )

        step3ConfigY = stepParams[2].copy()
        step3ConfigY['scanAxis'] = 'y'
        step3Y = self.generateStepWithAxes(
            3, 'partFinishing',
            partMesh, gateMesh, None,
            toolParams, step3ConfigY, step3Axes, globalMinZ, safetyMargin,
            solidClipMesh=None
        )

        step3 = {
            'stepId': 3,
            'stepType': 'partFinishing',
            'toolParams': step3X['toolParams'],
            'motionPolicy': 'externalPost',
            'segments': step3X['segments'] + step3Y['segments'],
            'clPoints': step3X['clPoints'] + step3Y['clPoints']
        }

        step4 = self.generateStepWithAxes(
            4, 'gateRemoval',
            gateMesh, partMesh, None,
            toolParams, stepParams[3], [np.array([0.0, 0.0, 1.0], dtype=float)], globalMinZ, safetyMargin,
            solidClipMesh=partMesh
        )

        outputData = {'version': self.version, 'wcsId': str(wcsId), 'steps': [step1, step2, step3, step4]}
        if jobId is not None:
            outputData['jobId'] = str(jobId)
        return outputData

    def exportClJson(self, clData: Dict[str, Any], outputPath: str) -> None:
        with open(outputPath, 'w', encoding='utf-8') as outputFile:
            json.dump(clData, outputFile, ensure_ascii=False, indent=2)