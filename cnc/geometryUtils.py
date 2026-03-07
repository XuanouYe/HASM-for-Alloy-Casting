from typing import Any, List, Tuple
import numpy as np
import trimesh
from scipy.spatial import cKDTree

def normalizeVector(vec: np.ndarray) -> np.ndarray:
    normValue = float(np.linalg.norm(vec))
    if normValue > 0.0:
        return vec / normValue
    return vec

def buildRotationFromTo(fromVec: np.ndarray, toVec: np.ndarray) -> np.ndarray:
    fromUnit = normalizeVector(np.asarray(fromVec, dtype=float))
    toUnit = normalizeVector(np.asarray(toVec, dtype=float))
    crossValue = np.cross(fromUnit, toUnit)
    dotValue = float(np.dot(fromUnit, toUnit))
    sinValue = float(np.linalg.norm(crossValue))
    if sinValue == 0.0:
        if dotValue > 0.0:
            return np.eye(3)
        helperAxis = np.array([1.0, 0.0, 0.0], dtype=float)
        if abs(fromUnit[0]) > 0.9:
            helperAxis = np.array([0.0, 1.0, 0.0], dtype=float)
        axisValue = normalizeVector(np.cross(fromUnit, helperAxis))
        skewMat = np.array([
            [0.0, -axisValue[2], axisValue[1]],
            [axisValue[2], 0.0, -axisValue[0]],
            [-axisValue[1], axisValue[0], 0.0]
        ], dtype=float)
        return np.eye(3) + 2.0 * skewMat @ skewMat
    skewMat = np.array([
        [0.0, -crossValue[2], crossValue[1]],
        [crossValue[2], 0.0, -crossValue[0]],
        [-crossValue[1], crossValue[0], 0.0]
    ], dtype=float)
    rotMat = np.eye(3) + skewMat + skewMat @ skewMat * ((1.0 - dotValue) / (sinValue * sinValue))
    return rotMat

def applyRotation(points: np.ndarray, rotMat: np.ndarray) -> np.ndarray:
    if len(points) == 0:
        return np.asarray(points, dtype=float)
    pointsArray = np.asarray(points, dtype=float)
    return (rotMat @ pointsArray.T).T

def generateHemisphereAxes(numAxes: int, minAxisZ: float = 0.0) -> List[List[float]]:
    if numAxes <= 0:
        return [[0.0, 0.0, 1.0]]
    axesList = []
    goldenAngle = np.pi * (3.0 - np.sqrt(5.0))
    for axisIndex in range(numAxes):
        zValue = 1.0 - axisIndex / float(max(numAxes - 1, 1))
        if zValue < minAxisZ:
            continue
        radiusValue = np.sqrt(max(0.0, 1.0 - zValue * zValue))
        thetaValue = goldenAngle * axisIndex
        xValue = np.cos(thetaValue) * radiusValue
        yValue = np.sin(thetaValue) * radiusValue
        axesList.append([float(xValue), float(yValue), float(zValue)])
    if not axesList:
        axesList.append([0.0, 0.0, 1.0])
    return axesList

def densifyPolyline(pathPoints: np.ndarray, maxStep: float) -> np.ndarray:
    pathArray = np.asarray(pathPoints, dtype=float)
    if len(pathArray) <= 1 or maxStep <= 0.0:
        return pathArray
    densePoints = [pathArray[0]]
    for pointIndex in range(1, len(pathArray)):
        startPoint = pathArray[pointIndex - 1]
        endPoint = pathArray[pointIndex]
        segVec = endPoint - startPoint
        segLen = float(np.linalg.norm(segVec))
        if segLen <= maxStep:
            densePoints.append(endPoint)
            continue
        splitCount = int(np.ceil(segLen / maxStep))
        for splitIndex in range(1, splitCount + 1):
            ratioValue = splitIndex / float(splitCount)
            densePoints.append(startPoint * (1.0 - ratioValue) + endPoint * ratioValue)
    return np.asarray(densePoints, dtype=float)

def splitPolylineByGap(pathPoints: np.ndarray, maxGap: float) -> List[np.ndarray]:
    pathArray = np.asarray(pathPoints, dtype=float)
    if len(pathArray) == 0:
        return []
    if len(pathArray) == 1:
        return [pathArray]
    subPaths = []
    currentPath = [pathArray[0]]
    for pointIndex in range(1, len(pathArray)):
        prevPoint = pathArray[pointIndex - 1]
        nextPoint = pathArray[pointIndex]
        if float(np.linalg.norm(nextPoint - prevPoint)) > maxGap:
            if len(currentPath) >= 2:
                subPaths.append(np.asarray(currentPath, dtype=float))
            currentPath = [nextPoint]
        else:
            currentPath.append(nextPoint)
    if len(currentPath) >= 2:
        subPaths.append(np.asarray(currentPath, dtype=float))
    return subPaths

def createEmptyMesh() -> trimesh.Trimesh:
    return trimesh.Trimesh(
        vertices=np.zeros((0, 3), dtype=float),
        faces=np.zeros((0, 3), dtype=int),
        process=False
    )

def concatenateMeshes(meshList: List[trimesh.Trimesh]) -> trimesh.Trimesh:
    validMeshes = [meshItem for meshItem in meshList if meshItem is not None and not meshItem.is_empty]
    if not validMeshes:
        return createEmptyMesh()
    if len(validMeshes) == 1:
        return validMeshes[0].copy()
    return trimesh.util.concatenate(validMeshes)

def sampleMeshPointsWithNormals(mesh: trimesh.Trimesh, sampleCount: int) -> Tuple[np.ndarray, np.ndarray]:
    if mesh is None or mesh.is_empty or sampleCount <= 0:
        return np.zeros((0, 3), dtype=float), np.zeros((0, 3), dtype=float)
    samplePoints, faceIndices = trimesh.sample.sample_surface(mesh, sampleCount)
    sampleNormals = mesh.face_normals[np.asarray(faceIndices, dtype=int)]
    return np.asarray(samplePoints, dtype=float), np.asarray(sampleNormals, dtype=float)

def deduplicateAxes(axesList: List[Any], angleToleranceDeg: float) -> List[np.ndarray]:
    uniqueAxes = []
    cosTol = float(np.cos(np.radians(angleToleranceDeg)))
    for ax in axesList:
        axVec = np.asarray(ax, dtype=float)
        isDup = False
        for uAx in uniqueAxes:
            if float(np.dot(axVec, uAx)) > cosTol:
                isDup = True
                break
        if not isDup:
            uniqueAxes.append(axVec)
    return uniqueAxes

def nearestDistanceToPath(points: np.ndarray, pathArray: np.ndarray) -> np.ndarray:
    if len(pathArray) == 0 or len(points) == 0:
        return np.full(len(points), np.inf, dtype=float)
    searchTree = cKDTree(pathArray)
    dists, _ = searchTree.query(points, k=1)
    return np.asarray(dists, dtype=float)
