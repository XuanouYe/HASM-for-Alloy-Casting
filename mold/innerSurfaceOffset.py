import numpy as np
import trimesh
import pyvista as pv
from typing import Optional, List
from shapely.geometry import Polygon, MultiPolygon
from shapely.geometry.base import BaseGeometry
from supportRegionDetector import SupportRegionDetector
from geometryAdapters import loadMeshFromFile


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
        currentMesh = mesh.copy()
        currentMesh.merge_vertices()
        currentMesh.fix_normals()
        if not currentMesh.is_watertight:
            trimesh.repair.fill_holes(currentMesh)

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

                try:
                    overhangGeom = supportGeom.intersection(curGeom)
                except Exception:
                    continue

                if overhangGeom.is_empty or overhangGeom.area < self.areaEps:
                    continue
                zBottom = zMin + (layerIdx * self.layerHeight)
                transitionVolume = self._createTransitionVolume(overhangGeom, zBottom, self.layerHeight)
                if transitionVolume is not None and transitionVolume.is_volume:
                    removalVolumes.append(transitionVolume)

            if not removalVolumes:
                break

            resultMesh = currentMesh
            for vol in removalVolumes:
                if not resultMesh.is_volume:
                    resultMesh.merge_vertices()
                    resultMesh.fix_normals()
                    if not resultMesh.is_watertight:
                        trimesh.repair.fill_holes(resultMesh)

                if resultMesh.is_volume and vol.is_volume:
                    try:
                        resultMesh = resultMesh.difference(vol)
                    except Exception:
                        continue

            if not isinstance(resultMesh, trimesh.Trimesh):
                break

            vertexChange = abs(resultMesh.vertices.shape[0] - currentMesh.vertices.shape[0])
            if vertexChange < 3:
                break

            currentMesh = resultMesh
            iterationCount += 1

        return currentMesh

    def _createTransitionVolume(self, overhangPolygon: BaseGeometry, zBottom: float, height: float) -> Optional[
        trimesh.Trimesh]:
        if overhangPolygon is None or overhangPolygon.is_empty or overhangPolygon.area < self.areaEps:
            return None
        try:
            poly = overhangPolygon.buffer(float(self.offsetDistance)).buffer(0)
        except Exception:
            return None

        if poly.is_empty or poly.area < self.areaEps:
            return None
        return self._shapelyToTrimesh(poly, zBottom, height)

    def _shapelyToTrimesh(self, polygon: BaseGeometry, zBottom: float, height: float) -> Optional[trimesh.Trimesh]:
        if polygon is None or polygon.is_empty:
            return None
        try:
            polygon = polygon.buffer(0)
        except Exception:
            pass

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
            try:
                m = trimesh.creation.extrude_polygon(poly, height=float(height))
                m.apply_translation([0.0, 0.0, float(zBottom)])
                m.merge_vertices()
                m.fix_normals()
                if not m.is_watertight:
                    trimesh.repair.fill_holes(m)
                if m.is_volume:
                    meshes.append(m)
            except Exception:
                continue

        if not meshes:
            return None

        if len(meshes) == 1:
            return meshes[0]
        else:
            res = meshes[0]
            for m in meshes[1:]:
                if res.is_volume and m.is_volume:
                    try:
                        res = res.union(m)
                    except Exception:
                        pass
            return res


def removeInnerSurfaceOverhangs(mesh: trimesh.Trimesh, config: dict) -> trimesh.Trimesh:
    processor = InnerSurfaceOffset(config)
    return processor.removeInnerSurfaceOverhangs(mesh)


def visualizeInnerSurfaceOffset(meshPath, configDict):
    originalMesh = loadMeshFromFile(meshPath)
    processedMesh = removeInnerSurfaceOverhangs(originalMesh, configDict)

    originalPvMesh = pv.wrap(originalMesh)
    processedPvMesh = pv.wrap(processedMesh)

    visualizationPlotter = pv.Plotter(shape=(1, 2))

    visualizationPlotter.subplot(0, 0)
    visualizationPlotter.add_mesh(originalPvMesh, color="lightblue", show_edges=True, opacity=0.5)

    visualizationPlotter.subplot(0, 1)
    visualizationPlotter.add_mesh(processedPvMesh, color="lightgreen", show_edges=True, opacity=0.5)

    visualizationPlotter.link_views()
    visualizationPlotter.show()


if __name__ == "__main__":
    offsetConfig = {
        "supportAngle": 45.0,
        "layerHeight": 0.2,
        "smoothHeight": 0.4,
        "areaEps": 1e-4,
        "maxIterations": 20
    }
    testFilePath = "../testModels/cylinder.mold.stl"
    visualizeInnerSurfaceOffset(testFilePath, offsetConfig)
