import numpy as np
import trimesh
import pyvista as pv
from typing import List, Optional
from shapely.geometry import Polygon, MultiPolygon
from shapely.geometry.base import BaseGeometry
from shapely import ops
from mold.supportRegionDetector import SupportRegionDetector
from mold.moldGenerator import MoldGenerator
from geometryAdapters import loadMeshFromFile


def decomposeToPolygons(geom: BaseGeometry) -> List[Polygon]:
    if geom is None or geom.is_empty:
        return []
    if isinstance(geom, Polygon):
        return [geom]
    if isinstance(geom, MultiPolygon):
        return [g for g in geom.geoms if not g.is_empty]
    if hasattr(geom, 'geoms'):
        result = []
        for g in geom.geoms:
            result.extend(decomposeToPolygons(g))
        return result
    return []


def buildCavityColumnMask(cavityLayerGeoms: List[Optional[BaseGeometry]]) -> Optional[BaseGeometry]:
    parts = []
    for geom in cavityLayerGeoms:
        if geom is not None and not geom.is_empty:
            parts.append(geom.buffer(0))
    if not parts:
        return None
    result = ops.unary_union(parts).buffer(0)
    return result if not result.is_empty else None


class InnerCavityOffsetPlanner:
    def __init__(self, supportAngle: float, layerHeight: float,
                 areaEps: float, minWallThickness: float):
        self.supportAngle = supportAngle
        self.layerHeight = layerHeight
        self.areaEps = areaEps
        self.minWallThickness = minWallThickness
        self.lateralStep = float(np.tan(np.deg2rad(supportAngle)) * layerHeight)

    def computeOffsetLayers(
        self,
        supportLayerGeoms: List[Optional[BaseGeometry]],
        solidLayerGeoms: List[Optional[BaseGeometry]],
        cavityLayerGeoms: List[Optional[BaseGeometry]]
    ) -> List[Optional[BaseGeometry]]:
        n = len(supportLayerGeoms)
        deltaAccum = [None] * n
        cavityColumnMask = buildCavityColumnMask(cavityLayerGeoms)

        for k in range(n):
            sk = supportLayerGeoms[k]
            if sk is None or sk.is_empty or sk.area < self.areaEps:
                continue
            for j in range(k, -1, -1):
                dist = (k - j) * self.lateralStep
                rjk = sk.buffer(dist).buffer(0) if dist > 0 else sk.buffer(0)
                if rjk is None or rjk.is_empty:
                    continue
                if cavityColumnMask is not None and not cavityColumnMask.is_empty:
                    rjk = rjk.intersection(cavityColumnMask).buffer(0)
                if rjk is None or rjk.is_empty:
                    continue
                if deltaAccum[j] is None:
                    deltaAccum[j] = rjk
                else:
                    deltaAccum[j] = deltaAccum[j].union(rjk).buffer(0)

        offsetCavityLayerGeoms = []
        for j in range(n):
            cj = cavityLayerGeoms[j]
            if cj is None or cj.is_empty:
                offsetCavityLayerGeoms.append(cj)
                continue
            solidJ = solidLayerGeoms[j] if j < len(solidLayerGeoms) else None
            if solidJ is not None and not solidJ.is_empty:
                bj = solidJ.buffer(-self.minWallThickness).buffer(0)
                if bj is None or bj.is_empty:
                    bj = cj
            else:
                bj = cj
            if deltaAccum[j] is not None and not deltaAccum[j].is_empty:
                candidate = cj.union(deltaAccum[j]).buffer(0)
            else:
                candidate = cj
            if bj is not None and not bj.is_empty:
                result = candidate.intersection(bj).buffer(0)
            else:
                result = candidate
            if result is None or result.is_empty or result.area < cj.area - self.areaEps:
                result = cj
            offsetCavityLayerGeoms.append(result)

        return offsetCavityLayerGeoms


class InnerCavityVolumeBuilder:
    def __init__(self, layerHeight: float, areaEps: float, booleanEngine: str):
        self.layerHeight = layerHeight
        self.areaEps = areaEps
        self.booleanEngine = booleanEngine
        self.zEps = 0.005

    def buildCutVolume(
        self,
        originalCavityLayerGeoms: List[Optional[BaseGeometry]],
        offsetCavityLayerGeoms: List[Optional[BaseGeometry]],
        sliceHeights: List[float],
        cavityColumnMask: Optional[BaseGeometry] = None
    ) -> Optional[trimesh.Trimesh]:
        extrudedLayers = []
        for j in range(len(sliceHeights)):
            origGeom = originalCavityLayerGeoms[j]
            offGeom = offsetCavityLayerGeoms[j]
            if origGeom is None or origGeom.is_empty:
                continue
            if offGeom is None or offGeom.is_empty:
                continue
            deltaGeom = offGeom.difference(origGeom).buffer(0)
            if deltaGeom is None or deltaGeom.is_empty or deltaGeom.area < self.areaEps:
                continue
            if cavityColumnMask is not None and not cavityColumnMask.is_empty:
                deltaGeom = deltaGeom.intersection(cavityColumnMask).buffer(0)
            if deltaGeom is None or deltaGeom.is_empty or deltaGeom.area < self.areaEps:
                continue
            extruded = self.shapelyToExtrudedMesh(
                deltaGeom,
                float(sliceHeights[j]) - self.zEps,
                self.layerHeight + 2 * self.zEps
            )
            if extruded is not None:
                extrudedLayers.append(extruded)
        if not extrudedLayers:
            return None
        if len(extrudedLayers) == 1:
            return extrudedLayers[0]
        return trimesh.boolean.union(extrudedLayers, engine=self.booleanEngine, check_volume=False)

    def shapelyToExtrudedMesh(
        self, geom: BaseGeometry, zBottom: float, height: float
    ) -> Optional[trimesh.Trimesh]:
        if geom is None or geom.is_empty:
            return None
        geom = geom.buffer(0)
        polys = decomposeToPolygons(geom)
        meshes = []
        for poly in polys:
            if not poly.is_valid or poly.area < self.areaEps:
                continue
            m = trimesh.creation.extrude_polygon(poly, height=float(height))
            m.apply_translation([0.0, 0.0, float(zBottom)])
            meshes.append(m)
        if not meshes:
            return None
        if len(meshes) == 1:
            return meshes[0]
        return trimesh.boolean.union(meshes, engine=self.booleanEngine, check_volume=False)


def ensureTrimesh(meshOrScene) -> trimesh.Trimesh:
    if isinstance(meshOrScene, trimesh.Scene):
        return meshOrScene.dump(concatenate=True)
    return meshOrScene


class InnerSurfaceOffset:
    def __init__(self, config: dict):
        self.config = config or {}
        self.supportAngle = float(self.config.get("supportAngle", 45.0))
        self.layerHeight = float(self.config.get("layerHeight", 0.2))
        self.areaEps = float(self.config.get("areaEps", 1e-4))
        self.minWallThickness = float(self.config.get("minWallThickness", 1.0))
        self.booleanEngine = self.config.get("booleanEngine", "manifold")
        self.detector = SupportRegionDetector(self.config)

    def removeInnerSurfaceOverhangs(self, moldMesh: trimesh.Trimesh) -> trimesh.Trimesh:
        workMesh = moldMesh.copy()
        sliceHeights = self.detector.generateSliceHeights(workMesh)
        layerPaths = self.detector.sliceMeshToLayers(workMesh, sliceHeights)
        solidLayerGeoms = [self.detector.path2DToShapely(p) for p in layerPaths]
        cavityLayerGeoms = [self.detector.extractCavities(p) for p in layerPaths]
        supportResult = self.detector.calculateSupportRegions(workMesh, sliceHeights)
        supportLayerGeoms = supportResult.layerGeoms
        if len(supportLayerGeoms) < len(sliceHeights):
            supportLayerGeoms = [None] + list(supportLayerGeoms)
        cavityColumnMask = buildCavityColumnMask(cavityLayerGeoms)
        planner = InnerCavityOffsetPlanner(
            supportAngle=self.supportAngle,
            layerHeight=self.layerHeight,
            areaEps=self.areaEps,
            minWallThickness=self.minWallThickness
        )
        offsetCavityLayerGeoms = planner.computeOffsetLayers(
            supportLayerGeoms, solidLayerGeoms, cavityLayerGeoms
        )
        builder = InnerCavityVolumeBuilder(
            layerHeight=self.layerHeight,
            areaEps=self.areaEps,
            booleanEngine=self.booleanEngine
        )
        cutVolumeMesh = builder.buildCutVolume(
            cavityLayerGeoms, offsetCavityLayerGeoms, list(sliceHeights), cavityColumnMask
        )
        if cutVolumeMesh is None:
            return workMesh
        result = trimesh.boolean.difference(
            [workMesh, cutVolumeMesh],
            engine=self.booleanEngine,
            check_volume=False
        )
        return ensureTrimesh(result)


def removeInnerSurfaceOverhangs(moldMesh: trimesh.Trimesh, config: dict) -> trimesh.Trimesh:
    processor = InnerSurfaceOffset(config)
    return processor.removeInnerSurfaceOverhangs(moldMesh)


def moldToCasting(moldMesh: trimesh.Trimesh, booleanEngine: str) -> trimesh.Trimesh:
    hull = trimesh.convex.convex_hull(moldMesh)
    casting = trimesh.boolean.difference([hull, moldMesh], engine=booleanEngine, check_volume=False)
    return ensureTrimesh(casting)


def addCoordinateAxes(plotter: pv.Plotter, mesh: trimesh.Trimesh):
    bounds = pv.wrap(mesh).bounds
    axisLength = max(bounds[1] - bounds[0], bounds[3] - bounds[2], bounds[5] - bounds[4]) * 0.4
    origin = np.array([bounds[0], bounds[2], bounds[4]])
    for direction, color, label in [
        (np.array([1, 0, 0]), "red", "X"),
        (np.array([0, 1, 0]), "green", "Y"),
        (np.array([0, 0, 1]), "blue", "Z"),
    ]:
        endPoint = origin + direction * axisLength
        arrow = pv.Arrow(start=origin, direction=direction, scale=axisLength,
                         tip_length=0.25, tip_radius=0.05, shaft_radius=0.02)
        plotter.add_mesh(arrow, color=color)
        plotter.add_point_labels([endPoint], [label], text_color=color,
                                 font_size=12, bold=True, show_points=False)


def visualizeInnerSurfaceOffset(castingMeshPath: str, configDict: dict):
    booleanEngine = configDict.get("booleanEngine", "manifold")
    castingMesh = ensureTrimesh(loadMeshFromFile(castingMeshPath))
    moldGen = MoldGenerator(configDict)
    originalMoldMesh = moldGen.generateMoldShell(castingMesh)
    processedMoldMesh = removeInnerSurfaceOverhangs(originalMoldMesh, configDict)
    visualizationPlotter = pv.Plotter(shape=(1, 3))
    visualizationPlotter.subplot(0, 0)
    visualizationPlotter.add_text("原始铸件", position="upper_edge", font_size=10)
    visualizationPlotter.add_mesh(pv.wrap(castingMesh), color="lightblue", show_edges=True, opacity=0.6)
    addCoordinateAxes(visualizationPlotter, castingMesh)
    visualizationPlotter.subplot(0, 1)
    visualizationPlotter.add_text("原始模具", position="upper_edge", font_size=10)
    visualizationPlotter.add_mesh(pv.wrap(originalMoldMesh), color="lightyellow", show_edges=True, opacity=0.6)
    addCoordinateAxes(visualizationPlotter, originalMoldMesh)
    visualizationPlotter.subplot(0, 2)
    visualizationPlotter.add_text("处理后模具（内腔悬垂修正）", position="upper_edge", font_size=10)
    visualizationPlotter.add_mesh(pv.wrap(processedMoldMesh), color="lightcoral", show_edges=True, opacity=0.6)
    addCoordinateAxes(visualizationPlotter, processedMoldMesh)
    visualizationPlotter.link_views()
    visualizationPlotter.show()


if __name__ == "__main__":
    offsetConfig = {
        "supportAngle": 45.0,
        "layerHeight": 0.2,
        "areaEps": 1e-4,
        "minWallThickness": 1.0,
        "booleanEngine": "manifold",
        "boundingBoxOffset": 2.0
    }
    testFilePath = "../testModels/cylinder.up.stl"
    visualizeInnerSurfaceOffset(testFilePath, offsetConfig)
