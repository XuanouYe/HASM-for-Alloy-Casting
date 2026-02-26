import numpy as np
import trimesh
from typing import List, Optional
from shapely import ops
from shapely.geometry.base import BaseGeometry
from dataModel import SupportRegionResult


class SupportRegionDetector:
    def __init__(self, config: Optional[dict] = None):
        self.config = config or {}
        self.supportAngle = float(self.config.get("supportAngle", 45.0))
        self.layerHeight = float(self.config.get("layerHeight", 0.2))
        self.smoothHeight = float(self.config.get("smoothHeight", 0.4))
        self.areaEps = float(self.config.get("areaEps", 1e-6))

    def calculateSupportRegions(self, mesh: trimesh.Trimesh,
                                sliceHeights: Optional[np.ndarray] = None) -> SupportRegionResult:
        if sliceHeights is None:
            sliceHeights = self._generateSliceHeights(mesh)
        layerPaths = self._sliceMeshToLayers(mesh, sliceHeights)
        layerGeoms = [self._path2DToShapely(p) for p in layerPaths]
        layersBelow = int(round(self.smoothHeight / self.layerHeight))
        maxDistFromLowerLayer = self._calculateMaxBridgeDistance()
        supportGeoms = []
        for layerIdx in range(1, len(layerGeoms)):
            cur = layerGeoms[layerIdx]
            if cur is None or cur.is_empty:
                supportGeoms.append(None)
                continue
            mergedBelow = self._getMergedGeomsBelow(layerGeoms, layerIdx, layersBelow, maxDistFromLowerLayer)
            if mergedBelow is None or mergedBelow.is_empty:
                supportGeoms.append(cur)
                continue
            diff = cur.difference(mergedBelow)
            if diff.is_empty or diff.area < self.areaEps:
                supportGeoms.append(None)
            else:
                supportGeoms.append(diff)
        totalSupportArea = sum(g.area for g in supportGeoms if g is not None and not g.is_empty)
        return SupportRegionResult(
            layerGeoms=supportGeoms,
            sliceHeights=sliceHeights,
            totalSupportArea=float(totalSupportArea)
        )

    def _generateSliceHeights(self, mesh: trimesh.Trimesh) -> np.ndarray:
        zMin, zMax = mesh.bounds[:, 2]
        return np.arange(zMin + self.layerHeight / 2, zMax, self.layerHeight)

    def _sliceMeshToLayers(self, mesh: trimesh.Trimesh, heights: np.ndarray) -> List:
        out = []
        for h in heights:
            s = mesh.section(plane_origin=[0, 0, float(h)], plane_normal=[0, 0, 1])
            if s is None:
                out.append(None)
                continue
            xform = np.eye(4)
            xform[2, 3] = -float(h)
            p2d, _ = s.to_2D(to_2D=xform, normal=[0, 0, 1])
            out.append(p2d)
        return out

    def _calculateMaxBridgeDistance(self) -> float:
        return np.tan(np.deg2rad(self.supportAngle)) * self.layerHeight

    def _path2DToShapely(self, path2d) -> Optional[BaseGeometry]:
        if path2d is None or path2d.is_empty:
            return None
        polys = list(path2d.polygons_full)
        if len(polys) == 0:
            return None
        return ops.unary_union(polys)

    def _getMergedGeomsBelow(self, layerGeoms: List[Optional[BaseGeometry]],
                             currentLayerIdx: int, layersBelow: int,
                             offsetDist: float) -> Optional[BaseGeometry]:
        merged = None
        for offset in range(1, min(layersBelow + 1, currentLayerIdx + 1)):
            g = layerGeoms[currentLayerIdx - offset]
            if g is None or g.is_empty:
                continue
            expanded = g.buffer(offsetDist * offset)
            if merged is None:
                merged = expanded
            else:
                merged = merged.union(expanded)
        return merged


def calculateSupportRegions(moldShell: trimesh.Trimesh, config: dict) -> SupportRegionResult:
    detector = SupportRegionDetector(config)
    return detector.calculateSupportRegions(moldShell)


if __name__ == "__main__":
    config = {
        "supportAngle": 45.0,
        "layerHeight": 0.5,
        "smoothHeight": 1.0,
        "areaEps": 1e-4
    }
    stlPath = "testModels/hollow.cylinder.down.stl"
    mesh = trimesh.load_mesh(stlPath)
    result = calculateSupportRegions(mesh, config)
    validLayers = sum(1 for g in result.layerGeoms if g is not None)
    print(f"Found support regions on {validLayers} layers.")
    print(f"Total support area: {result.totalSupportArea:.4f}")
