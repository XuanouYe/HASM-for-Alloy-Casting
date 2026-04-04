from typing import Optional, Tuple
import numpy as np
from scipy.spatial import cKDTree
import trimesh

class SdfVolume:
    def __init__(self, mesh: trimesh.Trimesh, voxelSize: float):
        self.voxelSize = float(voxelSize)
        self.isEmpty = mesh is None or mesh.is_empty
        self._tree: Optional[cKDTree] = None
        self._samples: Optional[np.ndarray] = None
        self._mesh = mesh
        if not self.isEmpty:
            sampleCount = max(20000, int(float(mesh.area) / (voxelSize * voxelSize) * 4))
            sampleCount = min(sampleCount, 500000)
            pts, _ = trimesh.sample.sample_surface(mesh, sampleCount)
            self._samples = np.asarray(pts, dtype=float)
            self._tree = cKDTree(self._samples)

    def query(self, points: np.ndarray) -> np.ndarray:
        pts = np.asarray(points, dtype=float)
        if self.isEmpty or self._tree is None:
            return np.full(len(pts), np.inf, dtype=float)
        dists, _ = self._tree.query(pts, k=1)
        insideMask = self._mesh.contains(pts)
        signed = np.asarray(dists, dtype=float)
        signed[insideMask] *= -1.0
        return signed

def buildSdfVolume(mesh: trimesh.Trimesh, voxelSize: float) -> SdfVolume:
    return SdfVolume(mesh, voxelSize)

def buildOffsetSdf(mesh: trimesh.Trimesh, offsetDist: float, voxelSize: float) -> SdfVolume:
    if mesh is None or mesh.is_empty:
        return SdfVolume(mesh, voxelSize)
    try:
        offsetMesh = mesh.copy()
        offsetMesh.vertices = mesh.vertices + mesh.vertex_normals * (-offsetDist)
        offsetMesh = trimesh.Trimesh(vertices=offsetMesh.vertices, faces=offsetMesh.faces, process=False)
        return SdfVolume(offsetMesh, voxelSize)
    except Exception:
        return SdfVolume(mesh, voxelSize)
