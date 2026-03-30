from abc import ABC, abstractmethod
from typing import Any, Dict, List, Optional, Tuple
import numpy as np
import scipy.ndimage as nd
import trimesh
from shapely.geometry import LineString, MultiPolygon
from shapely.ops import unary_union
from shapely.validation import make_valid


def repairMesh(mesh: trimesh.Trimesh) -> trimesh.Trimesh:
    if mesh is None or mesh.is_empty:
        return mesh
    m = mesh.copy()
    trimesh.repair.fix_winding(m)
    trimesh.repair.fix_normals(m)
    trimesh.repair.fill_holes(m)
    m.remove_degenerate_faces()
    m.remove_duplicate_faces()
    m.remove_infinite_values()
    m.remove_unreferenced_vertices()
    return m


def cleanPolygon(polys: List[Any]) -> Any:
    cleanPolys = []
    for polyItem in polys:
        fixedPoly = polyItem
        if not fixedPoly.is_valid:
            fixedPoly = make_valid(fixedPoly)
        fixedPoly = fixedPoly.buffer(0)
        if not fixedPoly.is_empty:
            cleanPolys.append(fixedPoly)
    if not cleanPolys:
        return MultiPolygon()
    return unary_union(cleanPolys)


def robustSection(mesh: trimesh.Trimesh, zValue: float, tolerance: float = 0.05) -> Optional[Tuple[Any, Any]]:
    offsets = [0.0, -tolerance, tolerance, -tolerance * 2, tolerance * 2]
    for offset in offsets:
        try:
            sectionResult = mesh.section(
                plane_origin=[0.0, 0.0, zValue + offset],
                plane_normal=[0.0, 0.0, 1.0]
            )
            if sectionResult is None:
                continue
            slice2d, to3dMat = sectionResult.to_2D()
            polys = slice2d.polygons_full
            if polys:
                return polys, to3dMat
        except Exception:
            continue
    return None


def extractLineStrings(geomValue: Any) -> List[Any]:
    if geomValue is None or geomValue.is_empty:
        return []
    geomType = geomValue.geom_type
    if geomType == 'LineString':
        return [geomValue]
    if geomType == 'MultiLineString':
        return [l for l in geomValue.geoms if l.geom_type == 'LineString']
    if geomType == 'GeometryCollection':
        return [l for l in geomValue.geoms if l.geom_type == 'LineString']
    return []


class IToolpathStrategy(ABC):
    @abstractmethod
    def generate(self, targetMesh: trimesh.Trimesh, keepOutMesh: trimesh.Trimesh, toolRadius: float,
                 params: Dict[str, Any], safetyMargin: float) -> List[np.ndarray]:
        pass


class ZLevelRoughingStrategy(IToolpathStrategy):
    def generate(self, targetMesh: trimesh.Trimesh, keepOutMesh: trimesh.Trimesh, toolRadius: float,
                 params: Dict[str, Any], safetyMargin: float) -> List[np.ndarray]:
        if targetMesh is None or targetMesh.is_empty:
            return []
        repairedTarget = repairMesh(targetMesh)
        repairedKeepOut = repairMesh(keepOutMesh) if keepOutMesh is not None and not keepOutMesh.is_empty else keepOutMesh
        stepOver = float(params.get('stepOver', 1.0))
        layerStep = float(params.get('layerStep', 1.0))
        roughStock = float(params.get('roughStock', 0.0))
        boundsArray = np.asarray(repairedTarget.bounds, dtype=float)
        zMin = float(boundsArray[0, 2])
        zMax = float(boundsArray[1, 2])
        zLevels = np.arange(zMax - layerStep, zMin, -layerStep, dtype=float)
        allPaths = []
        for zValue in zLevels:
            sectionData = robustSection(repairedTarget, zValue)
            if sectionData is None:
                continue
            targetPolys, to3dMat = sectionData
            machinablePoly = cleanPolygon(targetPolys).buffer(-(toolRadius + roughStock))
            if machinablePoly.is_empty:
                continue
            keepOutPoly = MultiPolygon()
            if repairedKeepOut is not None and not repairedKeepOut.is_empty:
                keepOutData = robustSection(repairedKeepOut, zValue)
                if keepOutData is not None:
                    keepOutPolys, _ = keepOutData
                    keepOutPoly = cleanPolygon(keepOutPolys).buffer(toolRadius + safetyMargin + roughStock)
            safePoly = machinablePoly.difference(keepOutPoly.buffer(0))
            if safePoly.is_empty:
                continue
            polyGeoms = list(safePoly.geoms) if safePoly.geom_type == 'MultiPolygon' else [safePoly]
            for polyItem in polyGeoms:
                if polyItem.is_empty:
                    continue
                minX, minY, maxX, maxY = polyItem.bounds
                yList = np.arange(minY, maxY + stepOver * 0.5, stepOver, dtype=float)
                reverseFlag = False
                for yValue in yList:
                    scanLine = LineString([(minX - 1.0, yValue), (maxX + 1.0, yValue)])
                    interResult = scanLine.intersection(polyItem)
                    lineItems = extractLineStrings(interResult)
                    for lineItem in lineItems:
                        coordArray = np.asarray(lineItem.coords, dtype=float)
                        if len(coordArray) < 2:
                            continue
                        if reverseFlag:
                            coordArray = coordArray[::-1]
                        coordHomo = np.column_stack([coordArray, np.zeros(len(coordArray)), np.ones(len(coordArray))])
                        path3d = (to3dMat @ coordHomo.T).T[:, :3]
                        if len(path3d) >= 2:
                            allPaths.append(path3d)
                    reverseFlag = not reverseFlag
        return allPaths


def generateDropCutterPaths(targetMesh: trimesh.Trimesh, keepOutMesh: trimesh.Trimesh, toolRadius: float,
                            params: Dict[str, Any], safetyMargin: float) -> List[np.ndarray]:
    if targetMesh is None or targetMesh.is_empty:
        return []

    scanAxis = str(params.get('scanAxis', 'x')).lower()
    stepOver = float(params.get('stepOver', 0.5))
    projectionStep = float(params.get('projectionStep', 0.5))
    finishStock = float(params.get('finishStock', 0.0))

    padValue = toolRadius + max(finishStock, safetyMargin) + stepOver
    b = targetMesh.bounds
    xMin, yMin = float(b[0, 0]) - padValue, float(b[0, 1]) - padValue
    xMax, yMax = float(b[1, 0]) + padValue, float(b[1, 1]) + padValue
    bottomZ = float(b[0, 2]) - padValue

    gridRes = min(projectionStep, stepOver, toolRadius) * 0.3
    gridRes = max(gridRes, 0.05)

    nx = int(np.ceil((xMax - xMin) / gridRes))
    ny = int(np.ceil((yMax - yMin) / gridRes))
    if nx < 2 or ny < 2:
        return []

    def buildZMap(mesh: trimesh.Trimesh, offset: float) -> np.ndarray:
        zMap = np.full((nx, ny), bottomZ, dtype=float)
        if mesh is None or mesh.is_empty:
            return zMap
        area = float(mesh.area)
        cnt = min(int(area / (gridRes * 0.5) ** 2), 2000000)
        try:
            pts, _ = trimesh.sample.sample_surface(mesh, cnt)
        except Exception:
            pts = np.asarray(mesh.vertices, dtype=float)
        pts = np.vstack([mesh.vertices, pts])
        ix = np.clip(((pts[:, 0] - xMin) / gridRes).astype(int), 0, nx - 1)
        iy = np.clip(((pts[:, 1] - yMin) / gridRes).astype(int), 0, ny - 1)
        np.maximum.at(zMap, (ix, iy), pts[:, 2] + offset)
        valid = zMap > bottomZ
        filled = nd.maximum_filter(zMap, size=3)
        zMap[~valid] = filled[~valid]
        return zMap

    zTarget = buildZMap(targetMesh, finishStock)
    zKeepOut = buildZMap(keepOutMesh, 0.0)

    cells = int(np.ceil(toolRadius / gridRes))
    kSize = 2 * cells + 1
    y, x = np.ogrid[-cells:cells + 1, -cells:cells + 1]
    distSq = x ** 2 + y ** 2
    maskR = distSq <= (toolRadius / gridRes) ** 2

    kernelTarget = np.full((kSize, kSize), -np.inf, dtype=float)
    kernelTarget[maskR] = np.sqrt(np.maximum(toolRadius ** 2 - distSq[maskR] * (gridRes ** 2), 0.0))

    rSafe = toolRadius + safetyMargin
    cellsSafe = int(np.ceil(rSafe / gridRes))
    kSizeSafe = 2 * cellsSafe + 1
    yS, xS = np.ogrid[-cellsSafe:cellsSafe + 1, -cellsSafe:cellsSafe + 1]
    distSqSafe = xS ** 2 + yS ** 2
    maskRSafe = distSqSafe <= (rSafe / gridRes) ** 2
    kernelKeepOut = np.full((kSizeSafe, kSizeSafe), -np.inf, dtype=float)
    kernelKeepOut[maskRSafe] = np.sqrt(np.maximum(rSafe ** 2 - distSqSafe[maskRSafe] * (gridRes ** 2), 0.0))

    clTarget = nd.grey_dilation(zTarget, structure=kernelTarget)
    clKeepOut = nd.grey_dilation(zKeepOut, structure=kernelKeepOut)
    clFinal = np.maximum(clTarget, clKeepOut)

    targetMask = zTarget > bottomZ
    targetFootprint = nd.binary_dilation(targetMask, structure=maskR)

    paths = []
    if scanAxis == 'x':
        scanVals = np.arange(float(b[0, 1]), float(b[1, 1]) + stepOver * 0.5, stepOver)
        travelVals = np.arange(float(b[0, 0]) - toolRadius, float(b[1, 0]) + toolRadius, projectionStep)
    else:
        scanVals = np.arange(float(b[0, 0]), float(b[1, 0]) + stepOver * 0.5, stepOver)
        travelVals = np.arange(float(b[0, 1]) - toolRadius, float(b[1, 1]) + toolRadius, projectionStep)

    reverse = False
    for scan in scanVals:
        tVals = travelVals[::-1] if reverse else travelVals
        reverse = not reverse
        seg = []
        for t in tVals:
            xVal = t if scanAxis == 'x' else scan
            yVal = scan if scanAxis == 'x' else t
            ix = int(round((xVal - xMin) / gridRes))
            iy = int(round((yVal - yMin) / gridRes))
            if 0 <= ix < nx and 0 <= iy < ny:
                if targetFootprint[ix, iy]:
                    zVal = float(clFinal[ix, iy])
                    if clTarget[ix, iy] >= clKeepOut[ix, iy] - 1e-3:
                        seg.append([float(xVal), float(yVal), zVal])
                        continue
            if len(seg) >= 2:
                paths.append(np.array(seg, dtype=float))
            seg = []
        if len(seg) >= 2:
            paths.append(np.array(seg, dtype=float))

    return paths


class DropRasterStrategy(IToolpathStrategy):
    def generate(self, targetMesh: trimesh.Trimesh, keepOutMesh: trimesh.Trimesh, toolRadius: float,
                 params: Dict[str, Any], safetyMargin: float) -> List[np.ndarray]:
        params['finishStock'] = float(params.get('roughStock', 0.5))
        return generateDropCutterPaths(targetMesh, keepOutMesh, toolRadius, params, safetyMargin)


class SurfaceProjectionFinishingStrategy(IToolpathStrategy):
    def generate(self, targetMesh: trimesh.Trimesh, keepOutMesh: trimesh.Trimesh, toolRadius: float,
                 params: Dict[str, Any], safetyMargin: float) -> List[np.ndarray]:
        params['finishStock'] = float(params.get('finishStock', 0.03))
        return generateDropCutterPaths(targetMesh, keepOutMesh, toolRadius, params, safetyMargin)


class ToolpathStrategyFactory:
    strategies = {
        'zlevelroughing':          ZLevelRoughingStrategy,
        'zlr':                     ZLevelRoughingStrategy,
        'dropraster':              DropRasterStrategy,
        'surfacefinishing':        SurfaceProjectionFinishingStrategy,
        'spf':                     SurfaceProjectionFinishingStrategy,
        'isoplanarpatchfinishing': SurfaceProjectionFinishingStrategy,
        'ipf':                     SurfaceProjectionFinishingStrategy,
    }

    @classmethod
    def getStrategy(cls, mode: str) -> IToolpathStrategy:
        return cls.strategies.get(str(mode).lower(), DropRasterStrategy)()
