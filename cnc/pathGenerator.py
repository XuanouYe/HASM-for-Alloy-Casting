import json
from typing import Any, Dict, List, Optional, Tuple
import numpy as np
import trimesh
from scipy.spatial import cKDTree
from scipy.spatial.distance import cdist
from .geometryUtils import applyRotation, buildRotationFromTo, concatenateMeshes, normalizeVector, sampleMeshPointsWithNormals
from .toolpathStrategies import ToolpathStrategyFactory

class PointCloudIpw:
    def __init__(self, mesh: trimesh.Trimesh, sampleCount: int = 50000):
        self.points, _ = sampleMeshPointsWithNormals(mesh, sampleCount)
        self.activeMask = np.ones(len(self.points), dtype=bool)

    def getActivePointsWcs(self) -> np.ndarray:
        return self.points[self.activeMask]

    def filterPathsLocal(self, pathsLocal: List[np.ndarray], rotToToolFrame: np.ndarray, toolRadius: float, stepOver: float) -> List[np.ndarray]:
        if not pathsLocal or not np.any(self.activeMask):
            return pathsLocal
        activePointsWcs = self.getActivePointsWcs()
        if len(activePointsWcs) == 0:
            return []
        activePointsLocal = applyRotation(activePointsWcs, rotToToolFrame)
        searchTree = cKDTree(activePointsLocal)
        searchRadius = toolRadius + stepOver * 1.5
        validPaths = []
        for pathLocal in pathsLocal:
            dists, _ = searchTree.query(pathLocal, k=1, distance_upper_bound=searchRadius)
            if np.any(dists != np.inf):
                validPaths.append(pathLocal)
        return validPaths

    def updateIpwLocal(self, pathsLocal: List[np.ndarray], rotToToolFrame: np.ndarray, toolRadius: float) -> None:
        if not pathsLocal or not np.any(self.activeMask):
            return
        activePointsWcs = self.getActivePointsWcs()
        activeIndices = np.where(self.activeMask)[0]
        activePointsLocal = applyRotation(activePointsWcs, rotToToolFrame)
        searchTree = cKDTree(activePointsLocal)
        removeLocalIndices = set()
        for pathLocal in pathsLocal:
            pointHits = searchTree.query_ball_point(pathLocal, r=toolRadius)
            for pathIndex, hitList in enumerate(pointHits):
                pathZ = pathLocal[pathIndex, 2]
                for localIndex in hitList:
                    if activePointsLocal[localIndex, 2] <= pathZ - toolRadius * 0.5:
                        removeLocalIndices.add(localIndex)
        if removeLocalIndices:
            removeGlobalIndices = activeIndices[list(removeLocalIndices)]
            self.activeMask[removeGlobalIndices] = False

class TrimeshToolpathEngine:
    def optimizePathLinking(self, paths: List[np.ndarray], safeZ: float, stepOver: float) -> np.ndarray:
        validPaths = [np.asarray(pathItem, dtype=float) for pathItem in paths if len(pathItem) >= 2]
        if not validPaths:
            return np.empty((0, 3), dtype=float)
        linkedParts = []
        unvisited = list(range(len(validPaths)))
        currentPos = None
        while unvisited:
            if currentPos is None:
                firstPath = validPaths[unvisited.pop(0)]
                linkedParts.append(firstPath)
                currentPos = np.asarray(firstPath[-1], dtype=float)
                continue
            startPoints = np.array([validPaths[pathIndex][0] for pathIndex in unvisited], dtype=float)
            endPoints = np.array([validPaths[pathIndex][-1] for pathIndex in unvisited], dtype=float)
            distStarts = cdist([currentPos], startPoints)[0]
            distEnds = cdist([currentPos], endPoints)[0]
            minStartIndex = int(np.argmin(distStarts))
            minEndIndex = int(np.argmin(distEnds))
            if distStarts[minStartIndex] <= distEnds[minEndIndex]:
                chosenListIndex = minStartIndex
                reverseFlag = False
                minDist = float(distStarts[minStartIndex])
            else:
                chosenListIndex = minEndIndex
                reverseFlag = True
                minDist = float(distEnds[minEndIndex])
            chosenPathIndex = unvisited.pop(chosenListIndex)
            nextPath = validPaths[chosenPathIndex] if not reverseFlag else validPaths[chosenPathIndex][::-1]
            if minDist <= stepOver * 2.1 and abs(currentPos[2] - nextPath[0, 2]) < 1e-3:
                if float(np.linalg.norm(currentPos - nextPath[0])) > 1e-6:
                    linkedParts.append(np.vstack([currentPos, nextPath[0]]))
                if len(nextPath) > 1:
                    linkedParts.append(nextPath[1:])
            else:
                bridgePath = np.vstack([
                    currentPos,
                    np.array([currentPos[0], currentPos[1], safeZ], dtype=float),
                    np.array([nextPath[0, 0], nextPath[0, 1], safeZ], dtype=float),
                    nextPath[0]
                ])
                linkedParts.append(bridgePath)
                if len(nextPath) > 1:
                    linkedParts.append(nextPath[1:])
            currentPos = np.asarray(nextPath[-1], dtype=float)
        nonEmptyParts = [partItem for partItem in linkedParts if len(partItem) > 0]
        if not nonEmptyParts:
            return np.empty((0, 3), dtype=float)
        return np.vstack(nonEmptyParts)

    def slicePathByPlatformZ(self, pathLocal: np.ndarray, rotBack: np.ndarray, platformSafeZ: float) -> List[np.ndarray]:
        pathWcs = applyRotation(pathLocal, rotBack)
        validMask = pathWcs[:, 2] >= platformSafeZ
        validPaths = []
        currentPath = []
        for pointIndex, isValid in enumerate(validMask.tolist()):
            if isValid:
                currentPath.append(pathLocal[pointIndex])
            else:
                if len(currentPath) >= 2:
                    validPaths.append(np.asarray(currentPath, dtype=float))
                currentPath = []
        if len(currentPath) >= 2:
            validPaths.append(np.asarray(currentPath, dtype=float))
        return validPaths

class FiveAxisCncPathGenerator:
    def __init__(self, version: str = '2.4'):
        self.version = str(version)
        self.toolpathEngine = TrimeshToolpathEngine()

    def loadMesh(self, stlPath: str) -> trimesh.Trimesh:
        meshData = trimesh.load_mesh(stlPath)
        if isinstance(meshData, trimesh.Scene):
            return concatenateMeshes([meshItem for meshItem in meshData.geometry.values()])
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
                'position': [float(pointValue[0]), float(pointValue[1]), float(pointValue[2])],
                'toolAxis': [float(toolAxisUnit[0]), float(toolAxisUnit[1]), float(toolAxisUnit[2])],
                'feedrate': float(feedrate),
                'segmentId': int(segmentId)
            }
            for pointIndex, pointValue in enumerate(np.asarray(positions, dtype=float))
        ]

    def buildAxisCoverageTable(self, targetMesh: trimesh.Trimesh, candidateAxes: List[np.ndarray], axisStrategyParams: Dict[str, Any]) -> Tuple[List[Dict[str, Any]], int]:
        sampleCount = int(axisStrategyParams.get('step3AxisSampleCount', 12000))
        minNormalDot = float(axisStrategyParams.get('step3MinNormalDot', 0.02))
        samplePoints, sampleNormals = sampleMeshPointsWithNormals(targetMesh, sampleCount)
        if len(sampleNormals) == 0:
            return [], 0
        axisCoverages = []
        uniqueAxes = []
        for axisItem in candidateAxes:
            axisUnit = normalizeVector(np.asarray(axisItem, dtype=float))
            isDuplicate = False
            for existingAxis in uniqueAxes:
                if abs(float(np.dot(axisUnit, existingAxis))) > 0.999:
                    isDuplicate = True
                    break
            if not isDuplicate:
                uniqueAxes.append(axisUnit)
        for axisIndex, axisUnit in enumerate(uniqueAxes):
            dotValues = sampleNormals @ axisUnit
            coveredIndices = set(np.where(dotValues >= minNormalDot)[0].tolist())
            axisCoverages.append({
                'axisIndex': int(axisIndex),
                'axis': axisUnit,
                'coveredSet': coveredIndices
            })
        return axisCoverages, len(samplePoints)

    def pickBestAxis(self, axisCoverages: List[Dict[str, Any]], selectedAxes: List[np.ndarray], coveredUniverse: set, diversityDot: float, enforceDiversity: bool) -> Tuple[Optional[np.ndarray], set]:
        bestAxis = None
        bestNewSet = set()
        bestCoverageCount = 0
        for axisData in axisCoverages:
            axisUnit = axisData['axis']
            if enforceDiversity and selectedAxes:
                maxSimilarity = max(abs(float(np.dot(axisUnit, selectedAxis))) for selectedAxis in selectedAxes)
                if maxSimilarity > diversityDot:
                    continue
            newCoverageSet = axisData['coveredSet'] - coveredUniverse
            newCoverageCount = len(newCoverageSet)
            if newCoverageCount > bestCoverageCount:
                bestCoverageCount = newCoverageCount
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
            currentCoverageRatio = len(coveredUniverse) / float(totalPoints)
            if currentCoverageRatio >= targetCoverage:
                break
        if not selectedAxes:
            selectedAxes = [normalizeVector(np.asarray(candidateAxes[0], dtype=float))]
        return selectedAxes

    def generateStepWithAxes(self, stepId: int, stepType: str, targetMesh: trimesh.Trimesh, keepOutMesh: trimesh.Trimesh, toolParams: Dict[str, Any], stepParam: Dict[str, Any], candidateAxes: List[np.ndarray], globalMinZ: float, safetyMargin: float) -> Dict[str, Any]:
        modeValue = str(stepParam.get('mode', 'dropRaster'))
        feedrate = float(stepParam.get('feedrate', 500.0))
        toolRadius = float(toolParams.get('diameter', 6.0)) * 0.5
        platformSafeZ = float(globalMinZ + toolRadius + safetyMargin)
        stepOver = float(stepParam.get('stepOver', 1.0))
        enablePathLinking = bool(stepParam.get('enablePathLinking', True))
        isFinishing = modeValue.lower() in {'surfacefinishing', 'spf'}
        strategy = ToolpathStrategyFactory.getStrategy(modeValue)
        useIpw = modeValue.lower() in {'zlevelroughing', 'zlr', 'dropraster'}
        ipwData = PointCloudIpw(targetMesh, int(stepParam.get('ipwSampleCount', 50000))) if useIpw else None
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
            localSafeZ = float(rotatedTarget.bounds[1, 2] + float(stepParam.get('safeHeight', 5.0)) + toolRadius)
            rawPathsLocal = strategy.generate(rotatedTarget, rotatedKeepOut, toolRadius, stepParam, safetyMargin)
            validPathsLocal = []
            for pathLocal in rawPathsLocal:
                if len(pathLocal) < 2:
                    continue
                if isFinishing:
                    validPathsLocal.append(pathLocal)
                else:
                    validPathsLocal.extend(self.toolpathEngine.slicePathByPlatformZ(pathLocal, rotBack, platformSafeZ))
            if ipwData is not None and validPathsLocal:
                validPathsLocal = ipwData.filterPathsLocal(validPathsLocal, rotToToolFrame, toolRadius, stepOver)
                ipwData.updateIpwLocal(validPathsLocal, rotToToolFrame, toolRadius)
            if not validPathsLocal:
                continue
            if enablePathLinking:
                optimizedLocalPath = self.toolpathEngine.optimizePathLinking(validPathsLocal, localSafeZ, stepOver)
                if len(optimizedLocalPath) == 0:
                    continue
                finalWcsPath = applyRotation(optimizedLocalPath, rotBack)
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
            else:
                for pathLocal in validPathsLocal:
                    finalWcsPath = applyRotation(pathLocal, rotBack)
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
        return {
            'stepId': int(stepId),
            'stepType': str(stepType),
            'toolParams': toolParams,
            'segments': segments,
            'clPoints': allClPoints
        }

    def generateJob(self, partStl: str, moldStl: str, gateStl: str, riserStl: str, toolParams: Dict[str, Any], stepParams: List[Dict[str, Any]], axisStrategyParams: Dict[str, Any], wcsId: str = 'WCS_MAIN', jobId: Optional[str] = None) -> Dict[str, Any]:
        partMesh = self.loadMesh(partStl)
        moldMesh = self.loadMesh(moldStl)
        gateMesh = self.loadMesh(gateStl)
        riserMesh = self.loadMesh(riserStl)
        meshList = [partMesh, moldMesh, gateMesh, riserMesh]
        globalMinZ = min(float(meshItem.bounds[0, 2]) for meshItem in meshList if meshItem is not None and not meshItem.is_empty)
        candidateAxesRaw = axisStrategyParams.get('candidateAxes', [[0.0, 0.0, 1.0]])
        candidateAxes = [normalizeVector(np.asarray(axisItem, dtype=float)) for axisItem in candidateAxesRaw]
        safetyMargin = float(toolParams.get('safetyMargin', 0.5))
        keepOutMeshStep1 = concatenateMeshes([partMesh, gateMesh, riserMesh])
        step1 = self.generateStepWithAxes(1, 'shellRemoval', moldMesh, keepOutMeshStep1, toolParams, stepParams[0], candidateAxes, globalMinZ, safetyMargin)
        riserAxes = [np.array([0.0, 0.0, 1.0], dtype=float)]
        keepOutMeshStep2 = concatenateMeshes([partMesh, gateMesh])
        step2 = self.generateStepWithAxes(2, 'riserRemoval', riserMesh, keepOutMeshStep2, toolParams, stepParams[1], riserAxes, globalMinZ, safetyMargin)
        keepOutMeshStep3 = gateMesh
        step3Axes = self.selectAxesGreedyCoverage(partMesh, candidateAxes, axisStrategyParams)
        step3ConfigX = stepParams[2].copy()
        step3ConfigX['scanAxis'] = 'x'
        step3X = self.generateStepWithAxes(3, 'partFinishing', partMesh, keepOutMeshStep3, toolParams, step3ConfigX, step3Axes, globalMinZ, safetyMargin)
        step3ConfigY = stepParams[2].copy()
        step3ConfigY['scanAxis'] = 'y'
        step3Y = self.generateStepWithAxes(3, 'partFinishing', partMesh, keepOutMeshStep3, toolParams, step3ConfigY, step3Axes, globalMinZ, safetyMargin)
        step3 = {
            'stepId': 3,
            'stepType': 'partFinishing',
            'toolParams': step3X['toolParams'],
            'segments': step3X['segments'] + step3Y['segments'],
            'clPoints': step3X['clPoints'] + step3Y['clPoints']
        }
        step4 = self.generateStepWithAxes(4, 'gateRemoval', gateMesh, partMesh, toolParams, stepParams[3], [np.array([0.0, 0.0, 1.0], dtype=float)], globalMinZ, safetyMargin)
        outputData = {
            'version': self.version,
            'wcsId': str(wcsId),
            'steps': [step1, step2, step3, step4]
        }
        if jobId is not None:
            outputData['jobId'] = str(jobId)
        return outputData

    def exportClJson(self, clData: Dict[str, Any], outputPath: str) -> None:
        with open(outputPath, 'w', encoding='utf-8') as outputFile:
            json.dump(clData, outputFile, ensure_ascii=False, indent=2)
