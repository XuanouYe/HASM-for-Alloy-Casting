import numpy as np
import trimesh
from scipy.spatial import cKDTree
from typing import List, Tuple


def normalizeVector(vec: np.ndarray) -> np.ndarray:
    normVal = float(np.linalg.norm(vec))
    if normVal > 0.0:
        return vec / normVal
    return vec


def buildRotationFromTo(fromVec: np.ndarray, toVec: np.ndarray) -> np.ndarray:
    fromVec = normalizeVector(fromVec)
    toVec = normalizeVector(toVec)
    crossVec = np.cross(fromVec, toVec)
    dotVal = float(np.dot(fromVec, toVec))
    sinVal = float(np.linalg.norm(crossVec))
    if sinVal == 0.0:
        if dotVal > 0.0:
            return np.eye(3)
        axisVec = np.array([1.0, 0.0, 0.0], dtype=float)
        if abs(fromVec[0]) > 0.9:
            axisVec = np.array([0.0, 1.0, 0.0], dtype=float)
        crossVec = normalizeVector(np.cross(fromVec, axisVec))
        skewMat = np.array([
            [0.0, -crossVec[2], crossVec[1]],
            [crossVec[2], 0.0, -crossVec[0]],
            [-crossVec[1], crossVec[0], 0.0]
        ], dtype=float)
        return np.eye(3) + 2.0 * (skewMat @ skewMat)
    skewMat = np.array([
        [0.0, -crossVec[2], crossVec[1]],
        [crossVec[2], 0.0, -crossVec[0]],
        [-crossVec[1], crossVec[0], 0.0]
    ], dtype=float)
    return np.eye(3) + skewMat + (skewMat @ skewMat) * ((1.0 - dotVal) / (sinVal * sinVal))


def applyRotation(points: np.ndarray, rotMat: np.ndarray) -> np.ndarray:
    if len(points) == 0:
        return points
    return (rotMat @ points.T).T


def generateHemisphereAxes(numAxes: int, minZ: float = 0.0) -> List[np.ndarray]:
    axes = []
    if numAxes <= 1:
        return [np.array([0.0, 0.0, 1.0], dtype=float)]
    phiVal = np.pi * (3.0 - np.sqrt(5.0))
    for axisIndex in range(numAxes):
        zVal = 1.0 - (axisIndex / float(numAxes - 1))
        if zVal < minZ:
            continue
        radiusVal = np.sqrt(max(0.0, 1.0 - zVal * zVal))
        thetaVal = phiVal * axisIndex
        xVal = np.cos(thetaVal) * radiusVal
        yVal = np.sin(thetaVal) * radiusVal
        axes.append(normalizeVector(np.array([xVal, yVal, zVal], dtype=float)))
    if not axes:
        axes = [np.array([0.0, 0.0, 1.0], dtype=float)]
    return axes


def angleBetweenAxesDeg(axisA: np.ndarray, axisB: np.ndarray) -> float:
    dotVal = float(np.clip(np.dot(normalizeVector(axisA), normalizeVector(axisB)), -1.0, 1.0))
    return float(np.degrees(np.arccos(dotVal)))


def deduplicateAxes(candidateAxes: List[np.ndarray], angleThresholdDeg: float = 12.0) -> List[np.ndarray]:
    uniqueAxes = []
    for axisVec in candidateAxes:
        axisVec = normalizeVector(np.asarray(axisVec, dtype=float))
        if axisVec[2] < 0.0:
            continue
        isDuplicate = False
        for uniqueAxis in uniqueAxes:
            if angleBetweenAxesDeg(axisVec, uniqueAxis) < angleThresholdDeg:
                isDuplicate = True
                break
        if not isDuplicate:
            uniqueAxes.append(axisVec)
    return uniqueAxes


def densifyPolyline(pathPoints: np.ndarray, sampleStep: float) -> np.ndarray:
    if len(pathPoints) < 2:
        return np.asarray(pathPoints, dtype=float)
    densePoints = [np.asarray(pathPoints[0], dtype=float)]
    for pointIndex in range(1, len(pathPoints)):
        startPoint = np.asarray(pathPoints[pointIndex - 1], dtype=float)
        endPoint = np.asarray(pathPoints[pointIndex], dtype=float)
        segLen = float(np.linalg.norm(endPoint - startPoint))
        if segLen <= sampleStep:
            densePoints.append(endPoint)
            continue
        sampleCount = int(np.ceil(segLen / sampleStep))
        tValues = np.linspace(0.0, 1.0, sampleCount + 1)
        for tVal in tValues[1:]:
            densePoints.append((1.0 - tVal) * startPoint + tVal * endPoint)
    return np.asarray(densePoints, dtype=float)


def sampleMeshSurface(mesh: trimesh.Trimesh, sampleCount: int) -> Tuple[np.ndarray, np.ndarray]:
    if mesh.is_empty:
        return np.zeros((0, 3), dtype=float), np.zeros((0, 3), dtype=float)
    try:
        points, faceIndices = trimesh.sample.sample_surface(mesh, sampleCount)
        normals = mesh.face_normals[faceIndices]
        return np.asarray(points, dtype=float), np.asarray(normals, dtype=float)
    except Exception:
        return np.zeros((0, 3), dtype=float), np.zeros((0, 3), dtype=float)


def nearestDistanceToPath(samplePoints: np.ndarray, pathPoints: np.ndarray) -> np.ndarray:
    if len(samplePoints) == 0:
        return np.zeros(0, dtype=float)
    if len(pathPoints) == 0:
        return np.full(len(samplePoints), np.inf, dtype=float)
    tree = cKDTree(pathPoints)
    distances, _ = tree.query(samplePoints, k=1)
    return np.asarray(distances, dtype=float)
