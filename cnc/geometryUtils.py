import numpy as np
import trimesh
from scipy.spatial.transform import Rotation
from shapely.geometry import LineString
from shapely.ops import linemerge

def applyRotation(points: np.ndarray, rotMat: np.ndarray) -> np.ndarray:
    if points is None or len(points) == 0:
        return np.empty((0, 3), dtype=float)
    homo = np.column_stack([points, np.ones(len(points), dtype=float)])
    return (rotMat @ homo.T).T[:, :3]

def buildRotationFromTo(vFrom: np.ndarray, vTo: np.ndarray) -> np.ndarray:
    f = vFrom / np.linalg.norm(vFrom)
    t = vTo / np.linalg.norm(vTo)
    rotAxis = np.cross(f, t)
    axisNorm = np.linalg.norm(rotAxis)
    if axisNorm < 1e-6:
        if np.dot(f, t) < 0:
            ortho = np.array([1.0, 0.0, 0.0])
            if abs(f[0]) > 0.9:
                ortho = np.array([0.0, 1.0, 0.0])
            rotAxis = np.cross(f, ortho)
            rotAxis /= np.linalg.norm(rotAxis)
            rotObj = Rotation.from_rotvec(np.pi * rotAxis)
        else:
            rotObj = Rotation.from_rotvec(np.zeros(3))
    else:
        rotAxis /= axisNorm
        angle = np.arccos(np.clip(np.dot(f, t), -1.0, 1.0))
        rotObj = Rotation.from_rotvec(angle * rotAxis)
    mat = np.eye(4, dtype=float)
    mat[:3, :3] = rotObj.as_matrix()
    return mat

def normalizeVector(vec: np.ndarray) -> np.ndarray:
    norm = np.linalg.norm(vec)
    if norm < 1e-6:
        return np.array([0.0, 0.0, 1.0], dtype=float)
    return vec / norm

def concatenateMeshes(meshes: list) -> trimesh.Trimesh:
    validMeshes = [m for m in meshes if m is not None and not m.is_empty]
    if not validMeshes:
        return trimesh.Trimesh(vertices=[], faces=[], process=False)
    if len(validMeshes) == 1:
        return validMeshes[0].copy()
    return trimesh.util.concatenate(validMeshes)

def sampleMeshPointsWithNormals(mesh: trimesh.Trimesh, count: int) -> tuple:
    if mesh is None or mesh.is_empty:
        return np.empty((0, 3), dtype=float), np.empty((0, 3), dtype=float)
    samples, faceIndices = trimesh.sample.sample_surface(mesh, count)
    normals = mesh.face_normals[faceIndices]
    return np.asarray(samples, dtype=float), np.asarray(normals, dtype=float)

def densifyPolyline(points: np.ndarray, step: float) -> np.ndarray:
    if len(points) < 2:
        return points
    densePoints = []
    for i in range(len(points) - 1):
        p1 = points[i]
        p2 = points[i + 1]
        dist = np.linalg.norm(p2 - p1)
        if dist > step:
            n = int(np.ceil(dist / step))
            for j in range(n):
                t = j / n
                densePoints.append(p1 * (1 - t) + p2 * t)
        else:
            densePoints.append(p1)
    densePoints.append(points[-1])
    return np.asarray(densePoints, dtype=float)

def splitPolylineByGap(points: np.ndarray, maxGap: float) -> list:
    if len(points) < 2:
        return [points]
    segments = []
    currentSeg = [points[0]]
    for i in range(1, len(points)):
        dist = np.linalg.norm(points[i] - points[i - 1])
        if dist > maxGap:
            if len(currentSeg) >= 2:
                segments.append(np.asarray(currentSeg, dtype=float))
            currentSeg = [points[i]]
        else:
            currentSeg.append(points[i])
    if len(currentSeg) >= 2:
        segments.append(np.asarray(currentSeg, dtype=float))
    return segments

def extractConnectedComponents(mesh: trimesh.Trimesh) -> list:
    if mesh.is_empty:
        return []
    return trimesh.graph.split(mesh, only_watertight=False)

def generateHemisphereAxes(count: int, minZ: float) -> list:
    axes = []
    phi = np.pi * (3.0 - np.sqrt(5.0))
    for i in range(count):
        y = 1.0 - (i / float(max(count - 1, 1))) * 2.0
        radius = np.sqrt(max(0.0, 1.0 - y * y))
        theta = phi * i
        x = np.cos(theta) * radius
        z = np.sin(theta) * radius
        if z >= minZ:
            axes.append(normalizeVector(np.array([float(x), float(y), float(z)], dtype=float)))
    if not any(float(np.dot(a, [0.0, 0.0, 1.0])) > 0.99 for a in axes):
        axes.append(np.array([0.0, 0.0, 1.0], dtype=float))
    return axes

def generateSphereAxes(count: int) -> list:
    axes = []
    phi = np.pi * (3.0 - np.sqrt(5.0))
    for i in range(count):
        y = 1.0 - (i / float(max(count - 1, 1))) * 2.0
        radius = np.sqrt(max(0.0, 1.0 - y * y))
        theta = phi * i
        x = np.cos(theta) * radius
        z = np.sin(theta) * radius
        axes.append(normalizeVector(np.array([float(x), float(y), float(z)], dtype=float)))
    return axes

def deduplicateAxes(axes: list, angleDegThreshold: float) -> list:
    cosThresh = float(np.cos(np.radians(angleDegThreshold)))
    result = []
    for axisVec in axes:
        a = normalizeVector(np.asarray(axisVec, dtype=float))
        if not any(abs(float(np.dot(a, r))) > cosThresh for r in result):
            result.append(a)
    return result

def nearestDistanceToPath(points: np.ndarray, pathPoints: np.ndarray) -> np.ndarray:
    from scipy.spatial import cKDTree
    if len(pathPoints) == 0:
        return np.full(len(points), np.inf, dtype=float)
    tree = cKDTree(pathPoints)
    dists, _ = tree.query(points, k=1)
    return np.asarray(dists, dtype=float)

def segmentsToPolylines(segments: np.ndarray, tol: float = 1e-5) -> list:
    if len(segments) == 0:
        return []
    lines = [LineString(seg) for seg in segments if len(seg) == 2]
    if not lines:
        return []
    merged = linemerge(lines)
    if merged.geom_type == "LineString":
        return [np.array(merged.coords)]
    return [np.array(l.coords) for l in merged.geoms]
