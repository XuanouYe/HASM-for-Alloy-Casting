from typing import Optional
import numpy as np
import trimesh

_BACKEND_OPEN3D = "open3d"
_BACKEND_PYSDF = "pysdf"
_BACKEND_LEGACY = "legacy"


class _Open3dSdfBackend:
    def __init__(self, mesh: trimesh.Trimesh):
        import open3d as o3d
        o3dMesh = o3d.geometry.TriangleMesh()
        o3dMesh.vertices = o3d.utility.Vector3dVector(np.asarray(mesh.vertices, dtype=np.float32))
        o3dMesh.triangles = o3d.utility.Vector3iVector(np.asarray(mesh.faces, dtype=np.int32))
        o3dMesh.compute_vertex_normals()
        self.scene = o3d.t.geometry.RaycastingScene()
        o3dTMesh = o3d.t.geometry.TriangleMesh.from_legacy(o3dMesh)
        self.scene.add_triangles(o3dTMesh)

    def query(self, points: np.ndarray) -> np.ndarray:
        import open3d as o3d
        pts = np.asarray(points, dtype=np.float32)
        tensor = o3d.core.Tensor(pts, dtype=o3d.core.Dtype.Float32)
        sdResult = self.scene.compute_signed_distance(tensor)
        return sdResult.numpy().astype(float)


class _PysdfSdfBackend:
    def __init__(self, mesh: trimesh.Trimesh):
        from pysdf import SDF
        self.sdf = SDF(
            np.asarray(mesh.vertices, dtype=np.float32),
            np.asarray(mesh.faces, dtype=np.uint32)
        )

    def query(self, points: np.ndarray) -> np.ndarray:
        pts = np.asarray(points, dtype=np.float32)
        return -self.sdf(pts).astype(float)


class _LegacySdfBackend:
    def __init__(self, mesh: trimesh.Trimesh, voxelSize: float):
        from scipy.spatial import cKDTree
        self.mesh = mesh
        sampleCount = max(20000, int(float(mesh.area) / (voxelSize * voxelSize) * 4))
        sampleCount = min(sampleCount, 500000)
        pts, _ = trimesh.sample.sample_surface(mesh, sampleCount)
        self._samples = np.asarray(pts, dtype=float)
        self._tree = cKDTree(self._samples)

    def query(self, points: np.ndarray) -> np.ndarray:
        pts = np.asarray(points, dtype=float)
        dists, _ = self._tree.query(pts, k=1)
        insideMask = self.mesh.contains(pts)
        signed = np.asarray(dists, dtype=float)
        signed[insideMask] *= -1.0
        return signed


def createSdfBackend(mesh: trimesh.Trimesh, voxelSize: float, backendName: str = "auto"):
    if mesh is None or mesh.is_empty:
        return None
    resolved = backendName.lower()
    if resolved == "auto":
        try:
            import open3d
            resolved = _BACKEND_OPEN3D
        except ImportError:
            pass
    if resolved == "auto":
        try:
            import pysdf
            resolved = _BACKEND_PYSDF
        except ImportError:
            pass
    if resolved == "auto":
        resolved = _BACKEND_LEGACY

    if resolved == _BACKEND_OPEN3D:
        try:
            return _Open3dSdfBackend(mesh)
        except Exception:
            resolved = _BACKEND_PYSDF
    if resolved == _BACKEND_PYSDF:
        try:
            return _PysdfSdfBackend(mesh)
        except Exception:
            resolved = _BACKEND_LEGACY
    return _LegacySdfBackend(mesh, voxelSize)