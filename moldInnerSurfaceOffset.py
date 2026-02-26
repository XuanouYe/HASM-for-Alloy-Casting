import numpy as np
import trimesh
from typing import Optional, List
from shapely.geometry import Polygon, MultiPolygon
from shapely.geometry.base import BaseGeometry
from moldSupportRegionDetector import SupportRegionDetector


class InnerSurfaceOffset:
    def __init__(self, config: dict):
        self.config = config or {}
        self.detector = SupportRegionDetector(self.config)
        self.supportAngle = float(self.config.get("supportAngle", 45.0))
        self.layerHeight = float(self.config.get("layerHeight", 0.2))
        self.areaEps = float(self.config.get("areaEps", 1e-6))
        self.maxIterations = int(self.config.get("maxIterations", 10))
        defaultOffset = float(
            self.config.get("maxBridgeDistance", np.tan(np.deg2rad(self.supportAngle)) * self.layerHeight))
        self.offsetDistance = float(self.config.get("offsetDistance", defaultOffset))

    def removeInnerSurfaceOverhangs(self, mesh: trimesh.Trimesh) -> trimesh.Trimesh:
        currentMesh = mesh
        iterationCount = 0
        while iterationCount < self.maxIterations:
            sliceHeights = self.detector._generateSliceHeights(currentMesh)
            layerPaths = self.detector._sliceMeshToLayers(currentMesh, sliceHeights)
            layerGeoms = [self.detector._path2DToShapely(p) for p in layerPaths]
            supportResult = self.detector.calculateSupportRegions(currentMesh, sliceHeights)
            supportGeoms = supportResult.layerGeoms

            validLayers = sum(1 for r in supportGeoms if r is not None and not r.is_empty)
            if validLayers == 0:
                break

            zMin = float(currentMesh.bounds[0, 2])
            removalVolumes = []

            for supportIdx, supportGeom in enumerate(supportGeoms):
                if supportGeom is None or supportGeom.is_empty or supportGeom.area < self.areaEps:
                    continue
                layerIdx = supportIdx + 1
                if layerIdx <= 0 or layerIdx >= len(layerGeoms):
                    continue
                curGeom = layerGeoms[layerIdx]
                if curGeom is None or curGeom.is_empty:
                    continue
                overhangGeom = supportGeom.intersection(curGeom)
                if overhangGeom.is_empty or overhangGeom.area < self.areaEps:
                    continue
                zBottom = zMin + (layerIdx * self.layerHeight)
                transitionVolume = self._createTransitionVolume(overhangGeom, zBottom, self.layerHeight)
                if transitionVolume is not None:
                    removalVolumes.append(transitionVolume)

            if not removalVolumes:
                break

            mergedRemoval = trimesh.util.concatenate(removalVolumes)
            resultMesh = currentMesh.difference(mergedRemoval)
            if not isinstance(resultMesh, trimesh.Trimesh):
                break
            vertexChange = abs(resultMesh.vertices.shape[0] - currentMesh.vertices.shape[0])
            if vertexChange < 3:
                break
            currentMesh = resultMesh
            iterationCount += 1

        return currentMesh

    def _createTransitionVolume(self, overhangPolygon: BaseGeometry, zBottom: float, height: float) -> Optional[trimesh.Trimesh]:
        if overhangPolygon is None or overhangPolygon.is_empty or overhangPolygon.area < self.areaEps:
            return None
        poly = overhangPolygon.buffer(float(self.offsetDistance))
        if poly.is_empty or poly.area < self.areaEps:
            return None
        return self._shapelyToTrimesh(poly, zBottom, height)

    def _shapelyToTrimesh(self, polygon: BaseGeometry, zBottom: float, height: float) -> Optional[trimesh.Trimesh]:
        if polygon is None or polygon.is_empty:
            return None
        if isinstance(polygon, Polygon):
            polys = [polygon]
        elif isinstance(polygon, MultiPolygon):
            polys = list(polygon.geoms)
        else:
            return None
        meshes = []
        for poly in polys:
            if (not poly.is_valid) or poly.area < self.areaEps:
                continue
            m = trimesh.creation.extrude_polygon(poly, height=float(height))
            m.apply_translation([0.0, 0.0, float(zBottom)])
            meshes.append(m)
        if not meshes:
            return None
        return meshes[0] if len(meshes) == 1 else trimesh.util.concatenate(meshes)


def removeInnerSurfaceOverhangs(mesh: trimesh.Trimesh, config: dict) -> trimesh.Trimesh:
    processor = InnerSurfaceOffset(config)
    return processor.removeInnerSurfaceOverhangs(mesh)


if __name__ == "__main__":
    from geometryAdapters import loadMeshFromFile, exportMeshToStl
    config = {
        "supportAngle": 45.0,
        "layerHeight": 0.2,
        "smoothHeight": 0.4,
        "areaEps": 1e-4,
        "maxIterations": 20
    }
    stlPath = "testModels/hollow.cylinder.left.stl"
    mesh = loadMeshFromFile(stlPath)
    print(f"Loaded mesh: {mesh.vertices.shape[0]} vertices, {mesh.faces.shape[0]} faces")
    processedMesh = removeInnerSurfaceOverhangs(mesh, config)
    outputPath = stlPath.replace(".stl", ".offset.stl")
    exportMeshToStl(processedMesh, outputPath)
    print(f"Saved to: {outputPath}")
