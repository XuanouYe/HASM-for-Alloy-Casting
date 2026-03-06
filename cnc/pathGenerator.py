import json
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple
import numpy as np
import trimesh
from .coveragePlanner import ShellCoveragePlanner
from .geometryUtils import applyRotation, buildRotationFromTo, normalizeVector
from .toolpathEngine import PointCloudIPW, TrimeshToolpathEngine
from .toolpathStrategies import ToolpathStrategyFactory


class FiveAxisCncPathGenerator:
    def __init__(self, version: str = '2.0'):
        self.version = str(version)
        self.toolpathEngine = TrimeshToolpathEngine()

    def loadMesh(self, stlPath: str) -> trimesh.Trimesh:
        mesh = trimesh.load_mesh(stlPath)
        if isinstance(mesh, trimesh.Scene):
            mesh = trimesh.util.concatenate([geometry for geometry in mesh.geometry.values()])
        return mesh

    def rotateMesh(self, mesh: trimesh.Trimesh, rotMat: np.ndarray) -> trimesh.Trimesh:
        if mesh.is_empty:
            return mesh
        return trimesh.Trimesh(vertices=applyRotation(np.asarray(mesh.vertices, dtype=float), rotMat), faces=np.asarray(mesh.faces, dtype=int), process=False)

    def buildClPointDicts(self, positions: np.ndarray, toolAxis: np.ndarray, feedrate: float, segmentId: int, startPointId: int) -> List[Dict[str, Any]]:
        toolAxis = normalizeVector(toolAxis)
        return [{
            'pointId': int(startPointId + pointIndex),
            'position': [float(position[0]), float(position[1]), float(position[2])],
            'toolAxis': [float(toolAxis[0]), float(toolAxis[1]), float(toolAxis[2])],
            'feedrate': float(feedrate),
            'segmentId': int(segmentId)
        } for pointIndex, position in enumerate(positions)]

    def buildBridgePoints(self, lastPoint: List[float], lastAxis: List[float], nextPoint: List[float], nextAxis: List[float],
                          globalSafeZ: float, feedrate: float, pointId: int, lastSegmentId: int, nextSegmentId: int) -> Tuple[List[Dict[str, Any]], int]:
        retractPoint = {
            'pointId': int(pointId),
            'position': [float(lastPoint[0]), float(lastPoint[1]), float(globalSafeZ)],
            'toolAxis': [float(lastAxis[0]), float(lastAxis[1]), float(lastAxis[2])],
            'feedrate': float(feedrate),
            'segmentId': int(lastSegmentId)
        }
        rotatePoint = {
            'pointId': int(pointId + 1),
            'position': [float(lastPoint[0]), float(lastPoint[1]), float(globalSafeZ)],
            'toolAxis': [float(nextAxis[0]), float(nextAxis[1]), float(nextAxis[2])],
            'feedrate': float(feedrate),
            'segmentId': int(nextSegmentId)
        }
        traversePoint = {
            'pointId': int(pointId + 2),
            'position': [float(nextPoint[0]), float(nextPoint[1]), float(globalSafeZ)],
            'toolAxis': [float(nextAxis[0]), float(nextAxis[1]), float(nextAxis[2])],
            'feedrate': float(feedrate),
            'segmentId': int(nextSegmentId)
        }
        approachPoint = {
            'pointId': int(pointId + 3),
            'position': [float(nextPoint[0]), float(nextPoint[1]), float(nextPoint[2])],
            'toolAxis': [float(nextAxis[0]), float(nextAxis[1]), float(nextAxis[2])],
            'feedrate': float(feedrate),
            'segmentId': int(nextSegmentId)
        }
        return [retractPoint, rotatePoint, traversePoint, approachPoint], pointId + 4

    def generateAxisPath(self, targetMesh: trimesh.Trimesh, keepOutMesh: trimesh.Trimesh, toolParams: Dict[str, Any],
                         stepParam: Dict[str, Any], toolAxis: np.ndarray, globalMinZ: float, safetyMargin: float,
                         ipw: Optional[PointCloudIPW]) -> np.ndarray:
        mode = str(stepParam.get('mode', 'dropRaster'))
        toolRadius = float(toolParams.get('diameter', 6.0)) * 0.5
        platformSafeZ = float(globalMinZ + toolRadius + safetyMargin)
        stepOver = float(stepParam.get('stepOver', 1.0))
        safeHeight = float(stepParam.get('safeHeight', 5.0))
        mapStep = max(float(stepOver * 0.5), float(toolRadius * 0.35), 0.5)
        rotToToolFrame = buildRotationFromTo(toolAxis, np.array([0.0, 0.0, 1.0], dtype=float))
        rotBack = rotToToolFrame.T
        rotatedTarget = self.rotateMesh(targetMesh, rotToToolFrame)
        rotatedKeepOut = self.rotateMesh(keepOutMesh, rotToToolFrame)
        rotatedCombined = rotatedTarget if rotatedKeepOut.is_empty else trimesh.util.concatenate([rotatedTarget, rotatedKeepOut])
        localSafeZ = float(rotatedCombined.bounds[1][2] + safeHeight)
        keepOutMapLocal = self.toolpathEngine.buildObstacleHeightMapLocal(rotatedKeepOut, mapStep, toolRadius, safetyMargin)
        combinedMapLocal = self.toolpathEngine.buildObstacleHeightMapLocal(rotatedCombined, mapStep, toolRadius, safetyMargin)
        strategy = ToolpathStrategyFactory.getStrategy(mode)
        rawPathsLocal = strategy.generate(rotatedTarget, rotatedKeepOut, toolRadius, stepParam, safetyMargin)
        platformPathsLocal = []
        for pathLocal in rawPathsLocal:
            if len(pathLocal) == 0:
                continue
            slicedPaths = self.toolpathEngine.slicePathByPlatformZ(pathLocal, rotBack, platformSafeZ)
            platformPathsLocal.extend(slicedPaths)
        cutSafePathsLocal = self.toolpathEngine.clipPathsByObstacleLocal(platformPathsLocal, keepOutMapLocal, mapStep, 0.0)
        if ipw is not None and cutSafePathsLocal:
            cutSafePathsLocal = ipw.filterPathsLocal(cutSafePathsLocal, rotToToolFrame, toolRadius, stepOver)
            if cutSafePathsLocal:
                ipw.updateIpwLocal(cutSafePathsLocal, rotToToolFrame, toolRadius)
        if not cutSafePathsLocal:
            return np.zeros((0, 3), dtype=float)
        optimizedLocalPath = self.toolpathEngine.optimizePathLinking(cutSafePathsLocal, localSafeZ, stepOver, combinedMapLocal, float(toolRadius + safetyMargin))
        if len(optimizedLocalPath) == 0:
            return np.zeros((0, 3), dtype=float)
        return applyRotation(np.asarray(optimizedLocalPath, dtype=float), rotBack)

    def generateStepWithAxes(self, stepId: int, stepType: str, targetMesh: trimesh.Trimesh, keepOutMesh: trimesh.Trimesh,
                             toolParams: Dict[str, Any], stepParam: Dict[str, Any], candidateAxes: List[np.ndarray],
                             globalMinZ: float, safetyMargin: float) -> Dict[str, Any]:
        feedrate = float(stepParam.get('feedrate', 500.0))
        safeHeight = float(stepParam.get('safeHeight', 5.0))
        combinedOriginal = targetMesh if keepOutMesh.is_empty else trimesh.util.concatenate([targetMesh, keepOutMesh])
        bounds = np.asarray(combinedOriginal.bounds, dtype=float)
        globalSafeZ = float(bounds[1][2] + safeHeight)
        ipw = PointCloudIPW(targetMesh, 60000) if str(stepParam.get('mode', '')).lower() in ['zlevelroughing', 'zlr', 'dropraster'] else None
        segments = []
        allClPoints = []
        pointId = 0
        for axisVec in candidateAxes:
            toolAxis = normalizeVector(np.asarray(axisVec, dtype=float))
            finalWcsPath = self.generateAxisPath(targetMesh, keepOutMesh, toolParams, stepParam, toolAxis, globalMinZ, safetyMargin, ipw)
            if len(finalWcsPath) == 0:
                continue
            segmentId = len(segments)
            if len(allClPoints) > 0:
                lastPoint = allClPoints[-1]['position']
                lastAxis = allClPoints[-1]['toolAxis']
                nextPoint = [float(finalWcsPath[0][0]), float(finalWcsPath[0][1]), float(finalWcsPath[0][2])]
                nextAxis = [float(toolAxis[0]), float(toolAxis[1]), float(toolAxis[2])]
                bridgePoints, pointId = self.buildBridgePoints(lastPoint, lastAxis, nextPoint, nextAxis, globalSafeZ, feedrate, pointId, int(allClPoints[-1]['segmentId']), segmentId)
                allClPoints.extend(bridgePoints)
            clPoints = self.buildClPointDicts(finalWcsPath, toolAxis, feedrate, segmentId, pointId)
            pointId += len(clPoints)
            allClPoints.extend(clPoints)
            segments.append({'segmentId': int(segmentId), 'toolAxis': [float(toolAxis[0]), float(toolAxis[1]), float(toolAxis[2])], 'pointCount': int(len(clPoints))})
        return {'stepId': int(stepId), 'stepType': str(stepType), 'toolParams': toolParams, 'segments': segments, 'clPoints': allClPoints}

    def generateShellRemovalStep(self, stepId: int, targetMesh: trimesh.Trimesh, keepOutMesh: trimesh.Trimesh,
                                 toolParams: Dict[str, Any], stepParam: Dict[str, Any], axisStrategyParams: Dict[str, Any],
                                 globalMinZ: float, safetyMargin: float) -> Dict[str, Any]:
        feedrate = float(stepParam.get('feedrate', 500.0))
        safeHeight = float(stepParam.get('safeHeight', 5.0))
        combinedOriginal = targetMesh if keepOutMesh.is_empty else trimesh.util.concatenate([targetMesh, keepOutMesh])
        bounds = np.asarray(combinedOriginal.bounds, dtype=float)
        globalSafeZ = float(bounds[1][2] + safeHeight)
        toolRadius = float(toolParams.get('diameter', 6.0)) * 0.5
        planner = ShellCoveragePlanner(
            shellMesh=targetMesh,
            keepOutMesh=keepOutMesh,
            toolRadius=toolRadius,
            safetyMargin=safetyMargin,
            sampleCount=int(axisStrategyParams.get('shellCoverageSamples', 8000)),
            accessAngleDeg=float(axisStrategyParams.get('shellAccessAngleDeg', 72.0)),
            minAxisZ=float(axisStrategyParams.get('shellMinAxisZ', 0.02))
        )
        baseAxisCount = int(axisStrategyParams.get('shellBaseAxisCount', 48))
        localAxisCount = int(axisStrategyParams.get('shellLocalAxisCount', 16))
        maxAxes = int(axisStrategyParams.get('shellMaxAxes', 12))
        coverageTarget = float(axisStrategyParams.get('shellCoverageTarget', 0.985))
        minAxisScore = float(axisStrategyParams.get('shellMinAxisScore', 20.0))
        strictCoverage = bool(axisStrategyParams.get('shellCoverageStrict', True))
        ipw = PointCloudIPW(targetMesh, 80000)
        segments = []
        allClPoints = []
        pointId = 0
        selectedAxes = []
        rejectedAxes = []
        while planner.getCoverageRatio() < coverageTarget and len(selectedAxes) + len(rejectedAxes) < maxAxes * 3 and len(selectedAxes) < maxAxes:
            suggested = planner.suggestAxes(baseAxisCount, localAxisCount, selectedAxes + rejectedAxes, topK=8)
            if not suggested:
                break
            bestAxisInfo = suggested[0]
            if float(bestAxisInfo['score']) < minAxisScore:
                break
            toolAxis = normalizeVector(np.asarray(bestAxisInfo['axis'], dtype=float))
            finalWcsPath = self.generateAxisPath(targetMesh, keepOutMesh, toolParams, stepParam, toolAxis, globalMinZ, safetyMargin, ipw)
            if len(finalWcsPath) == 0:
                rejectedAxes.append(toolAxis)
                continue
            segmentId = len(segments)
            if len(allClPoints) > 0:
                lastPoint = allClPoints[-1]['position']
                lastAxis = allClPoints[-1]['toolAxis']
                nextPoint = [float(finalWcsPath[0][0]), float(finalWcsPath[0][1]), float(finalWcsPath[0][2])]
                nextAxis = [float(toolAxis[0]), float(toolAxis[1]), float(toolAxis[2])]
                bridgePoints, pointId = self.buildBridgePoints(lastPoint, lastAxis, nextPoint, nextAxis, globalSafeZ, feedrate, pointId, int(allClPoints[-1]['segmentId']), segmentId)
                allClPoints.extend(bridgePoints)
            clPoints = self.buildClPointDicts(finalWcsPath, toolAxis, feedrate, segmentId, pointId)
            pointId += len(clPoints)
            allClPoints.extend(clPoints)
            segments.append({'segmentId': int(segmentId), 'toolAxis': [float(toolAxis[0]), float(toolAxis[1]), float(toolAxis[2])], 'pointCount': int(len(clPoints))})
            selectedAxes.append(toolAxis)
            planner.updateCoverageByPath(toolAxis, finalWcsPath)
        coverageRatio = planner.getCoverageRatio()
        result = {
            'stepId': int(stepId),
            'stepType': 'shellRemoval',
            'toolParams': toolParams,
            'segments': segments,
            'clPoints': allClPoints,
            'coverageRatio': float(coverageRatio),
            'selectedAxisCount': int(len(selectedAxes)),
            'selectedAxes': [[float(axis[0]), float(axis[1]), float(axis[2])] for axis in selectedAxes]
        }
        if strictCoverage and coverageRatio < coverageTarget:
            raise RuntimeError(f'Shell removal coverage {coverageRatio:.4f} below target {coverageTarget:.4f}. Refine axis limits or shell planner parameters before finishing.')
        return result

    def generateJob(self, partStl: str, moldStl: str, gateStl: str, riserStl: str, toolParams: Dict[str, Any],
                    stepParams: List[Dict[str, Any]], axisStrategyParams: Dict[str, Any], wcsId: str = 'WCS0',
                    jobId: Optional[str] = None) -> Dict[str, Any]:
        partMesh = self.loadMesh(partStl)
        moldMesh = self.loadMesh(moldStl)
        gateMesh = self.loadMesh(gateStl)
        riserMesh = self.loadMesh(riserStl)
        globalMinZ = min([float(mesh.bounds[0][2]) for mesh in [partMesh, moldMesh, gateMesh, riserMesh]])
        candidateAxesRaw = axisStrategyParams.get('candidateAxes', [[0.0, 0.0, 1.0]])
        candidateAxes = [normalizeVector(np.array(item, dtype=float)) for item in candidateAxesRaw if np.asarray(item, dtype=float)[2] >= 0.0]
        if not candidateAxes:
            candidateAxes = [np.array([0.0, 0.0, 1.0], dtype=float)]
        safetyMargin = float(toolParams.get('safetyMargin', 0.5))
        keepOutMeshStep1 = trimesh.util.concatenate([partMesh, gateMesh, riserMesh])
        step1 = self.generateShellRemovalStep(1, moldMesh, keepOutMeshStep1, toolParams, stepParams[0], axisStrategyParams, globalMinZ, safetyMargin)
        targetMeshStep2 = trimesh.util.concatenate([partMesh, riserMesh])
        step2 = self.generateStepWithAxes(2, 'partAndRiserFinishing', targetMeshStep2, gateMesh, toolParams, stepParams[1], candidateAxes, globalMinZ, safetyMargin)
        step3 = self.generateStepWithAxes(3, 'gateRemoval', gateMesh, partMesh, toolParams, stepParams[2], candidateAxes, globalMinZ, safetyMargin)
        output = {'version': self.version, 'wcsId': str(wcsId), 'steps': [step1, step2, step3]}
        if jobId is not None:
            output['jobId'] = str(jobId)
        return output

    def exportClJson(self, clData: Dict[str, Any], outputPath: str) -> None:
        outputFile = Path(outputPath)
        outputFile.parent.mkdir(parents=True, exist_ok=True)
        with open(outputFile, 'w', encoding='utf-8') as fileObj:
            json.dump(clData, fileObj, ensure_ascii=False, indent=2)
