from typing import Optional
import numpy as np
import trimesh
from .sdfBackend import createSdfBackend


class SdfVolume:
    def __init__(self, mesh: trimesh.Trimesh, voxelSize: float, backendName: str = "auto"):
        self.voxelSize = float(voxelSize)
        self.isEmpty = mesh is None or mesh.is_empty
        self._backend = None
        if not self.isEmpty:
            self._backend = createSdfBackend(mesh, voxelSize, backendName)
            if self._backend is None:
                self.isEmpty = True

    def query(self, points: np.ndarray) -> np.ndarray:
        pts = np.asarray(points, dtype=float)
        if self.isEmpty or self._backend is None:
            return np.full(len(pts), np.inf, dtype=float)
        return self._backend.query(pts)


def buildSdfVolume(mesh: trimesh.Trimesh, voxelSize: float, backendName: str = "auto") -> SdfVolume:
    return SdfVolume(mesh, voxelSize, backendName)


def buildOffsetSdf(mesh: trimesh.Trimesh, offsetDist: float, voxelSize: float,
                   backendName: str = "auto") -> SdfVolume:
    if mesh is None or mesh.is_empty:
        return SdfVolume(mesh, voxelSize, backendName)
    try:
        offsetMesh = mesh.copy()
        offsetMesh.vertices = mesh.vertices + mesh.vertex_normals * (-offsetDist)
        offsetMesh = trimesh.Trimesh(vertices=offsetMesh.vertices, faces=offsetMesh.faces, process=False)
        return SdfVolume(offsetMesh, voxelSize, backendName)
    except Exception:
        return SdfVolume(mesh, voxelSize, backendName)