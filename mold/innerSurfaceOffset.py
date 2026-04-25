import numpy as np
import trimesh
import pyvista as pv
from typing import List, Optional
from shapely.geometry import Polygon, MultiPolygon, Point, LineString
from shapely.geometry.base import BaseGeometry
from shapely import ops
from mold.supportRegionDetector import SupportRegionDetector
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

def samplePrintLines(cavityPoly: Polygon, axis: np.ndarray, numLines: int) -> List[LineString]:
    coords = np.array(cavityPoly.exterior.coords)
    centered = coords - coords.mean(axis=0)
    perp = np.array([-axis[1], axis[0]])
    perpProj = centered @ perp
    pMin, pMax = perpProj.min(), perpProj.max()
    centroidXy = np.array([cavityPoly.centroid.x, cavityPoly.centroid.y])
    lines = []
    for t in np.linspace(0.15, 0.85, numLines):
        offset = pMin + t * (pMax - pMin)
        lineOrigin = centroidXy + offset * perp
        tVals = np.linspace(-1e6, 1e6, 2)
        p0 = lineOrigin + tVals[0] * axis
        p1 = lineOrigin + tVals[1] * axis
        scanLine = LineString([p0, p1])
        clipped = scanLine.intersection(cavityPoly)
        if clipped.is_empty:
            continue
        if clipped.geom_type == 'LineString':
            lines.append(clipped)
        elif clipped.geom_type == 'MultiLineString':
            lines.extend(list(clipped.geoms))
    return lines

def isCantilever(cavityPoly: Polygon, aboveCavityGeom: Optional[BaseGeometry], belowSolidGeom: Optional[BaseGeometry], lateralStep: float, pillarRadius: float, areaEps: float, bridgeLineSamples: int = 7) -> bool:
    if aboveCavityGeom is not None and not aboveCavityGeom.is_empty:
        allowedZone = aboveCavityGeom.buffer(lateralStep).buffer(0)
        if not allowedZone.is_empty:
            exceeding = cavityPoly.difference(allowedZone).buffer(0)
            if exceeding.is_empty or exceeding.area < areaEps:
                return False
    area = cavityPoly.area
    if area < areaEps:
        return False
    if belowSolidGeom is None or belowSolidGeom.is_empty:
        return True
    perimeter = cavityPoly.length
    circularity = 4.0 * np.pi * area / (perimeter ** 2) if perimeter > 0 else 0.0
    coords = np.array(cavityPoly.exterior.coords)
    if len(coords) < 3:
        return True
    centered = coords - coords.mean(axis=0)
    cov = np.cov(centered.T)
    if cov.ndim < 2 or np.allclose(cov, 0):
        return True
    eigVals, eigVecs = np.linalg.eigh(cov)
    sortIdx = np.argsort(eigVals)[::-1]
    eigVals = eigVals[sortIdx]
    eigVecs = eigVecs[:, sortIdx]
    aspectRatio = eigVals[0] / (eigVals[1] + 1e-12)
    if circularity > 0.65 or aspectRatio < 2.5:
        centroid = cavityPoly.centroid
        sampleR = max(np.sqrt(area / np.pi) * 0.7, pillarRadius)
        sampleCircle = Point(centroid.x, centroid.y).buffer(sampleR)
        supportArea = sampleCircle.intersection(belowSolidGeom).area
        coverRatio = supportArea / sampleCircle.area if sampleCircle.area > 0 else 0.0
        return coverRatio < 0.5
    axis = eigVecs[:, 0]
    printLines = samplePrintLines(cavityPoly, axis, bridgeLineSamples)
    if not printLines:
        projections = centered @ axis
        topMask = projections >= np.percentile(projections, 85)
        botMask = projections <= np.percentile(projections, 15)
        ptA = coords[topMask].mean(axis=0)
        ptB = coords[botMask].mean(axis=0)
        pillarA = Point(float(ptA[0]), float(ptA[1])).buffer(pillarRadius)
        pillarB = Point(float(ptB[0]), float(ptB[1])).buffer(pillarRadius)
        contactA = pillarA.intersects(belowSolidGeom)
        contactB = pillarB.intersects(belowSolidGeom)
        return not (contactA and contactB)
    bridgeCount = 0
    cantileverCount = 0
    unsupportedCount = 0
    for line in printLines:
        lineCoords = list(line.coords)
        if len(lineCoords) < 2:
            continue
        ptA = lineCoords[0]
        ptB = lineCoords[-1]
        circleA = Point(ptA[0], ptA[1]).buffer(pillarRadius)
        circleB = Point(ptB[0], ptB[1]).buffer(pillarRadius)
        contactA = circleA.intersects(belowSolidGeom)
        contactB = circleB.intersects(belowSolidGeom)
        if contactA and contactB:
            bridgeCount += 1
        elif contactA ^ contactB:
            cantileverCount += 1
        else:
            unsupportedCount += 1
    total = bridgeCount + cantileverCount + unsupportedCount
    if total == 0:
        return True
    if bridgeCount == total:
        return False
    if cantileverCount > 0:
        return True
    cantileverRatio = (cantileverCount + unsupportedCount) / total
    return cantileverRatio >= 0.5

class InnerCavityOffsetPlanner:
    def __init__(self, supportAngle: float, layerHeight: float, areaEps: float, minWallThickness: float, pillarRadius: float, bridgeLineSamples: int = 7):
        self.supportAngle = supportAngle
        self.layerHeight = layerHeight
        self.areaEps = areaEps
        self.minWallThickness = minWallThickness
        self.pillarRadius = pillarRadius
        self.bridgeLineSamples = bridgeLineSamples
        self.detector = SupportRegionDetector({"supportAngle": self.supportAngle, "layerHeight": self.layerHeight})
        self.lateralStep = self.detector.calculateMaxBridgeDistance()

    def computeOffsetLayers(self, cavityLayerGeoms: List[Optional[BaseGeometry]], solidLayerGeoms: List[Optional[BaseGeometry]]) -> List[Optional[BaseGeometry]]:
        n = len(cavityLayerGeoms)
        deltaAccum = [None] * n
        cavityColumnMask = buildCavityColumnMask(cavityLayerGeoms)
        for k in range(1, n):
            ckPrev = cavityLayerGeoms[k - 1]
            ck = cavityLayerGeoms[k]
            belowSolid = solidLayerGeoms[k - 1] if k > 0 else None
            if ckPrev is None or ckPrev.is_empty or ckPrev.area < self.areaEps:
                continue
            if ck is None or ck.is_empty:
                overhangRegion = ckPrev.buffer(0)
            else:
                allowedShrink = ck.buffer(self.lateralStep).buffer(0)
                if allowedShrink is None or allowedShrink.is_empty:
                    overhangRegion = ckPrev.buffer(0)
                else:
                    overhangRegion = ckPrev.difference(allowedShrink).buffer(0)
            if overhangRegion is None or overhangRegion.is_empty or overhangRegion.area < self.areaEps:
                continue
            components = decomposeToPolygons(overhangRegion)
            activeParts = []
            for comp in components:
                if comp.area < self.areaEps:
                    continue
                if isCantilever(comp, ck, belowSolid, self.lateralStep, self.pillarRadius, self.areaEps, self.bridgeLineSamples):
                    activeParts.append(comp)
            if not activeParts:
                continue
            activeSk = ops.unary_union(activeParts).buffer(0)
            if activeSk is None or activeSk.is_empty:
                continue
            for j in range(k - 1, -1, -1):
                dist = (k - 1 - j) * self.lateralStep
                rjk = activeSk.buffer(dist).buffer(0) if dist > 0 else activeSk.buffer(0)
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

    def buildCutVolume(self, originalCavityLayerGeoms: List[Optional[BaseGeometry]], offsetCavityLayerGeoms: List[Optional[BaseGeometry]], sliceHeights: List[float], cavityColumnMask: Optional[BaseGeometry] = None) -> Optional[trimesh.Trimesh]:
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
            extruded = self.shapelyToExtrudedMesh(deltaGeom, float(sliceHeights[j]) - self.zEps, self.layerHeight + 2 * self.zEps)
            if extruded is not None:
                extrudedLayers.append(extruded)
        if not extrudedLayers:
            return None
        if len(extrudedLayers) == 1:
            return extrudedLayers[0]
        return trimesh.boolean.union(extrudedLayers, engine=self.booleanEngine, check_volume=False)

    def shapelyToExtrudedMesh(self, geom: BaseGeometry, zBottom: float, height: float) -> Optional[trimesh.Trimesh]:
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
        self.pillarRadius = float(self.config.get("pillarRadius", self.layerHeight))
        self.bridgeLineSamples = int(self.config.get("bridgeLineSamples", 7))
        self.detector = SupportRegionDetector(self.config)

    def removeInnerSurfaceOverhangs(self, moldMesh: trimesh.Trimesh) -> trimesh.Trimesh:
        workMesh = moldMesh.copy()
        sliceHeights = self.detector.generateSliceHeights(workMesh)
        layerPaths = self.detector.sliceMeshToLayers(workMesh, sliceHeights)
        solidLayerGeoms = [self.detector.path2DToShapely(p) for p in layerPaths]
        cavityLayerGeoms = [self.detector.extractCavities(p) for p in layerPaths]
        cavityColumnMask = buildCavityColumnMask(cavityLayerGeoms)
        planner = InnerCavityOffsetPlanner(supportAngle=self.supportAngle, layerHeight=self.layerHeight, areaEps=self.areaEps, minWallThickness=self.minWallThickness, pillarRadius=self.pillarRadius, bridgeLineSamples=self.bridgeLineSamples)
        offsetCavityLayerGeoms = planner.computeOffsetLayers(cavityLayerGeoms, solidLayerGeoms)
        builder = InnerCavityVolumeBuilder(layerHeight=self.layerHeight, areaEps=self.areaEps, booleanEngine=self.booleanEngine)
        cutVolumeMesh = builder.buildCutVolume(cavityLayerGeoms, offsetCavityLayerGeoms, list(sliceHeights), cavityColumnMask)
        if cutVolumeMesh is None:
            return workMesh
        result = trimesh.boolean.difference([workMesh, cutVolumeMesh], engine=self.booleanEngine, check_volume=False)
        return ensureTrimesh(result)

def removeInnerSurfaceOverhangs(moldMesh: trimesh.Trimesh, config: dict) -> trimesh.Trimesh:
    processor = InnerSurfaceOffset(config)
    return processor.removeInnerSurfaceOverhangs(moldMesh)

def moldToCasting(moldMesh: trimesh.Trimesh, booleanEngine: str) -> trimesh.Trimesh:
    hull = trimesh.convex.convex_hull(moldMesh)
    casting = trimesh.boolean.difference([hull, moldMesh], engine=booleanEngine, check_volume=False)
    return ensureTrimesh(casting)

def visualizeInnerSurfaceOffset(meshPath: str, configDict: dict):
    booleanEngine = configDict.get("booleanEngine", "manifold")
    originalMoldMesh = loadMeshFromFile(meshPath)
    processedMoldMesh = removeInnerSurfaceOverhangs(originalMoldMesh, configDict)
    originalCasting = moldToCasting(originalMoldMesh, booleanEngine)
    processedCasting = moldToCasting(processedMoldMesh, booleanEngine)
    visualizationPlotter = pv.Plotter(shape=(1, 2))
    visualizationPlotter.subplot(0, 0)
    visualizationPlotter.add_text("原始铸件", position="upper_edge", font_size=10)
    visualizationPlotter.add_mesh(pv.wrap(originalCasting), color="lightblue", show_edges=True, opacity=1.0)
    visualizationPlotter.subplot(0, 1)
    visualizationPlotter.add_text("处理后铸件（内腔扩张）", position="upper_edge", font_size=10)
    visualizationPlotter.add_mesh(pv.wrap(processedCasting), color="lightcoral", show_edges=True, opacity=1.0)
    visualizationPlotter.link_views()
    visualizationPlotter.show()

if __name__ == "__main__":
    offsetConfig = {
        "supportAngle": 45.0,
        "layerHeight": 0.2,
        "areaEps": 1e-4,
        "minWallThickness": 1.0,
        "booleanEngine": "manifold",
        "pillarRadius": 0.2,
        "bridgeLineSamples": 20
    }
    testFilePath = "../testModels/cylinder.up.stl"
    visualizeInnerSurfaceOffset(testFilePath, offsetConfig)