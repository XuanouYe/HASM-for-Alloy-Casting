from abc import ABC, abstractmethod
from typing import Any, Dict, List
import numpy as np
import scipy.ndimage as nd
import trimesh
from shapely.geometry import LineString, MultiPolygon, Polygon
from shapely.ops import unary_union, polygonize
from shapely.validation import make_valid


class IToolpathStrategy(ABC):
    @abstractmethod
    def generate(self, targetMesh: trimesh.Trimesh, keepOutMesh: trimesh.Trimesh,
                 toolRadius: float, params: Dict[str, Any],
                 safetyMargin: float) -> List[np.ndarray]:
        pass


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


def robustSectionWith2d(mesh: trimesh.Trimesh, zValue: float,
                        tolerance: float = 0.05) -> Any:
    def tryOne(zVal: float):
        try:
            sectionResult = mesh.section(
                plane_origin=[0.0, 0.0, zVal],
                plane_normal=[0.0, 0.0, 1.0])
            if sectionResult is None:
                return None, None
            slice2d, to3dMat = sectionResult.to_2D()
            polys = slice2d.polygons_full
            if not polys:
                return None, None
            return polys, to3dMat
        except Exception:
            return None, None

    polys, to3dMat = tryOne(zValue)
    if polys is not None:
        return polys, to3dMat
    polys, to3dMat = tryOne(zValue - tolerance)
    if polys is not None:
        return polys, to3dMat
    return tryOne(zValue + tolerance)


def sectionInSharedFrame(mesh: trimesh.Trimesh, zValue: float,
                         to3dMat: np.ndarray, tolerance: float = 0.05) -> Any:
    invMat = np.linalg.inv(to3dMat)
    def tryOne(zVal: float):
        try:
            sec = mesh.section(
                plane_origin=[0.0, 0.0, zVal],
                plane_normal=[0.0, 0.0, 1.0])
            if sec is None:
                return None
            verts3d = np.asarray(sec.vertices, dtype=float)
            if len(verts3d) == 0:
                return None
            hom = np.column_stack([verts3d, np.ones(len(verts3d))])
            verts2d = (invMat @ hom.T).T[:, :2]
            lineList = []
            for entity in sec.entities:
                pts = entity.points
                for k in range(len(pts) - 1):
                    lineList.append([verts2d[pts[k]].tolist(),
                                     verts2d[pts[k + 1]].tolist()])
            if not lineList:
                return None
            mls = unary_union([LineString(seg) for seg in lineList])
            polys = list(polygonize(mls))
            return polys if polys else None
        except Exception:
            return None

    result = tryOne(zValue)
    if result is not None:
        return result
    result = tryOne(zValue - tolerance)
    if result is not None:
        return result
    return tryOne(zValue + tolerance)


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


def _rasterFillPoly(polyItem: Any, stepOver: float, to3dMat: np.ndarray,
                    zValue: float, reverseStart: bool) -> List[np.ndarray]:
    paths = []
    if polyItem.is_empty:
        return paths
    minX, minY, maxX, maxY = polyItem.bounds
    yList = np.arange(minY, maxY + stepOver * 0.5, stepOver, dtype=float)
    reverseFlag = reverseStart
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
            coordHomo = np.column_stack(
                [coordArray, np.zeros(len(coordArray)),
                 np.ones(len(coordArray))])
            path3d = (to3dMat @ coordHomo.T).T[:, :3]
            if len(path3d) >= 2:
                paths.append(path3d)
        reverseFlag = not reverseFlag
    return paths


def _contourPaths(poly: Any, to3dMat: np.ndarray) -> List[np.ndarray]:
    paths = []
    boundary = poly.boundary
    if boundary is None or boundary.is_empty:
        return paths
    geomType = boundary.geom_type
    lineGeoms = []
    if geomType in ('LineString', 'LinearRing'):
        lineGeoms = [boundary]
    elif geomType == 'MultiLineString':
        lineGeoms = list(boundary.geoms)
    elif geomType == 'GeometryCollection':
        lineGeoms = [g for g in boundary.geoms if g.geom_type in ('LineString', 'LinearRing')]
    for lineGeom in lineGeoms:
        coords2d = np.asarray(lineGeom.coords, dtype=float)
        if len(coords2d) < 2:
            continue
        coordHomo = np.column_stack(
            [coords2d, np.zeros(len(coords2d)), np.ones(len(coords2d))])
        path3d = (to3dMat @ coordHomo.T).T[:, :3]
        if len(path3d) >= 2:
            paths.append(path3d)
    return paths


class ZLevelRoughingStrategy(IToolpathStrategy):
    def generate(self, targetMesh: trimesh.Trimesh, keepOutMesh: trimesh.Trimesh,
                 toolRadius: float, params: Dict[str, Any],
                 safetyMargin: float) -> List[np.ndarray]:
        if targetMesh.is_empty:
            return []
        stepOver = float(params.get('stepOver', 1.0))
        layerStep = float(params.get('layerStep', 1.0))
        roughStock = float(params.get('roughStock', 0.0))
        bottomClearance = float(params.get('bottomClearance', 0.0))
        boundsArray = np.asarray(targetMesh.bounds, dtype=float)
        zMin = float(boundsArray[0, 2])
        zMax = float(boundsArray[1, 2])
        localSafeZ = zMin + bottomClearance if bottomClearance > 0.0 else -np.inf
        zLevels = np.arange(zMax - layerStep, zMin, -layerStep, dtype=float)
        allPaths = []
        for zValue in zLevels:
            if zValue < localSafeZ:
                continue
            targetPolys, to3dMat = robustSectionWith2d(targetMesh, zValue)
            if not targetPolys or to3dMat is None:
                continue
            machinablePoly = cleanPolygon(targetPolys).buffer(-(toolRadius + roughStock))
            keepOutPoly = MultiPolygon()
            if keepOutMesh is not None and not keepOutMesh.is_empty:
                keepOutPolys = sectionInSharedFrame(keepOutMesh, zValue, to3dMat)
                if keepOutPolys:
                    keepOutPoly = cleanPolygon(keepOutPolys).buffer(
                        toolRadius + safetyMargin + roughStock)
            if machinablePoly.is_empty:
                continue
            safePoly = machinablePoly.difference(keepOutPoly.buffer(0))
            if safePoly.is_empty:
                continue
            polyGeoms = (list(safePoly.geoms)
                         if safePoly.geom_type == 'MultiPolygon' else [safePoly])
            reverseFlag = False
            for polyItem in polyGeoms:
                if polyItem.is_empty:
                    continue
                rasterPaths = _rasterFillPoly(polyItem, stepOver, to3dMat, zValue, reverseFlag)
                allPaths.extend(rasterPaths)
                reverseFlag = not reverseFlag
        return allPaths


class RiserGateRemovalStrategy(IToolpathStrategy):
    def generate(self, targetMesh: trimesh.Trimesh, keepOutMesh: trimesh.Trimesh,
                 toolRadius: float, params: Dict[str, Any],
                 safetyMargin: float) -> List[np.ndarray]:
        if targetMesh is None or targetMesh.is_empty:
            return []
        stepOver = float(params.get('stepOver', toolRadius * 1.5))
        layerStep = float(params.get('layerStep', toolRadius * 1.5))
        roughStock = float(params.get('roughStock', 0.0))
        bottomClearance = float(params.get('bottomClearance', 0.0))
        boundsArray = np.asarray(targetMesh.bounds, dtype=float)
        zMin = float(boundsArray[0, 2])
        zMax = float(boundsArray[1, 2])
        localSafeZ = zMin + bottomClearance if bottomClearance > 0.0 else -np.inf
        zLevels = np.arange(zMax, zMin - layerStep * 0.1, -layerStep, dtype=float)
        if len(zLevels) == 0:
            zLevels = np.array([zMax, zMin], dtype=float)
        allPaths = []
        for zValue in zLevels:
            if zValue < localSafeZ:
                continue
            targetPolys, to3dMat = robustSectionWith2d(targetMesh, zValue)
            if not targetPolys or to3dMat is None:
                continue
            outerUnion = cleanPolygon(targetPolys)
            if outerUnion.is_empty:
                continue
            machinablePoly = outerUnion.buffer(-(toolRadius + roughStock))
            if machinablePoly.is_empty:
                machinablePoly = outerUnion.buffer(-roughStock) if roughStock > 0 else outerUnion
            if machinablePoly.is_empty:
                allPaths.extend(_contourPaths(outerUnion, to3dMat))
                continue
            polyGeoms = (list(machinablePoly.geoms)
                         if machinablePoly.geom_type == 'MultiPolygon' else [machinablePoly])
            reverseFlag = False
            for polyItem in polyGeoms:
                if polyItem.is_empty:
                    continue
                rasterPaths = _rasterFillPoly(polyItem, stepOver, to3dMat, zValue, reverseFlag)
                if rasterPaths:
                    allPaths.extend(rasterPaths)
                else:
                    allPaths.extend(_contourPaths(polyItem, to3dMat))
                reverseFlag = not reverseFlag
        return allPaths


def generateDropCutterPaths(targetMesh: trimesh.Trimesh, keepOutMesh: trimesh.Trimesh,
                            toolRadius: float, params: Dict[str, Any],
                            safetyMargin: float) -> List[np.ndarray]:
    if targetMesh is None or targetMesh.is_empty:
        return []

    scanAxis = str(params.get('scanAxis', 'x')).lower()
    stepOver = float(params.get('stepOver', 0.5))
    projectionStep = float(params.get('projectionStep', 0.5))
    finishStock = float(params.get('finishStock', 0.0))

    padValue = toolRadius + max(finishStock, safetyMargin) + stepOver
    b = targetMesh.bounds
    xMin = float(b[0, 0]) - padValue
    yMin = float(b[0, 1]) - padValue
    xMax = float(b[1, 0]) + padValue
    yMax = float(b[1, 1]) + padValue
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
        pts, _ = trimesh.sample.sample_surface(mesh, cnt)
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
    kernelTarget[maskR] = 0.0

    rSafe = toolRadius + safetyMargin
    cellsSafe = int(np.ceil(rSafe / gridRes))
    kSizeSafe = 2 * cellsSafe + 1
    yS, xS = np.ogrid[-cellsSafe:cellsSafe + 1, -cellsSafe:cellsSafe + 1]
    distSqSafe = xS ** 2 + yS ** 2
    maskRSafe = distSqSafe <= (rSafe / gridRes) ** 2
    kernelKeepOut = np.full((kSizeSafe, kSizeSafe), -np.inf, dtype=float)
    kernelKeepOut[maskRSafe] = np.sqrt(
        np.maximum(rSafe ** 2 - distSqSafe[maskRSafe] * (gridRes ** 2), 0.0))

    clTarget = nd.grey_dilation(zTarget, structure=kernelTarget)
    clKeepOut = nd.grey_dilation(zKeepOut, structure=kernelKeepOut)
    clFinal = np.maximum(clTarget, clKeepOut)

    targetMask = zTarget > bottomZ
    targetFootprint = nd.binary_dilation(targetMask, structure=maskR)

    paths = []
    if scanAxis == 'x':
        scanVals = np.arange(float(b[0, 1]), float(b[1, 1]) + stepOver * 0.5, stepOver)
        travelVals = np.arange(float(b[0, 0]) - toolRadius,
                               float(b[1, 0]) + toolRadius, projectionStep)
    else:
        scanVals = np.arange(float(b[0, 0]), float(b[1, 0]) + stepOver * 0.5, stepOver)
        travelVals = np.arange(float(b[0, 1]) - toolRadius,
                               float(b[1, 1]) + toolRadius, projectionStep)

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
            inBounds = 0 <= ix < nx and 0 <= iy < ny
            if inBounds and targetFootprint[ix, iy]:
                blockedByKeepOut = clTarget[ix, iy] < clKeepOut[ix, iy] - 1e-3
                if blockedByKeepOut:
                    if len(seg) >= 2:
                        paths.append(np.array(seg, dtype=float))
                    seg = []
                else:
                    zVal = float(clFinal[ix, iy])
                    seg.append([float(xVal), float(yVal), zVal])
        if len(seg) >= 2:
            paths.append(np.array(seg, dtype=float))

    return paths


def generateShellRemovalPaths(targetMesh: trimesh.Trimesh,
                               keepOutMesh: trimesh.Trimesh,
                               toolRadius: float,
                               params: Dict[str, Any],
                               safetyMargin: float) -> List[np.ndarray]:
    if targetMesh is None or targetMesh.is_empty:
        return []

    stepOver = float(params.get('stepOver', toolRadius * 1.5))
    layerStep = float(params.get('layerStep', toolRadius * 1.5))
    roughStock = float(params.get('roughStock', toolRadius * 0.3))
    bottomClearance = float(params.get('bottomClearance', 0.0))
    useContour = bool(params.get('step1UseContour', True))
    contourPasses = int(params.get('step1ContourPasses', 2))

    boundsArray = np.asarray(targetMesh.bounds, dtype=float)
    zMin = float(boundsArray[0, 2])
    zMax = float(boundsArray[1, 2])
    localSafeZ = zMin + bottomClearance if bottomClearance > 0.0 else -np.inf

    zLevels = np.arange(zMax, zMin - layerStep * 0.1, -layerStep, dtype=float)
    if len(zLevels) == 0:
        zLevels = np.array([zMax, zMin], dtype=float)
    elif zLevels[-1] > zMin + layerStep * 0.5:
        zLevels = np.append(zLevels, zMin)

    allPaths = []
    contourOffset = toolRadius
    rasterOffset = toolRadius + roughStock

    for zValue in zLevels:
        if zValue < localSafeZ:
            continue

        targetPolys, to3dMat = robustSectionWith2d(targetMesh, zValue)
        if not targetPolys or to3dMat is None:
            continue

        outerUnion = cleanPolygon(targetPolys)
        if outerUnion.is_empty:
            continue

        keepOutPoly = MultiPolygon()
        if keepOutMesh is not None and not keepOutMesh.is_empty:
            keepOutPolys2d = sectionInSharedFrame(keepOutMesh, zValue, to3dMat)
            if keepOutPolys2d:
                keepOutPoly = cleanPolygon(keepOutPolys2d).buffer(
                    toolRadius + safetyMargin)

        fillablePoly = outerUnion.buffer(-rasterOffset)
        if fillablePoly.is_empty:
            fillablePoly = outerUnion.buffer(-contourOffset)
        if fillablePoly.is_empty:
            continue

        if not keepOutPoly.is_empty:
            fillablePoly = fillablePoly.difference(keepOutPoly)
            if fillablePoly.is_empty:
                continue

        if useContour:
            for passIdx in range(contourPasses):
                offsetDist = contourOffset + passIdx * stepOver
                contourRegion = outerUnion.buffer(-offsetDist)
                if contourRegion.is_empty:
                    break
                if not keepOutPoly.is_empty:
                    contourRegion = contourRegion.difference(keepOutPoly)
                    if contourRegion.is_empty:
                        break
                if contourRegion.geom_type == 'Polygon':
                    contourPolys = [Polygon(contourRegion.exterior)]
                elif contourRegion.geom_type == 'MultiPolygon':
                    contourPolys = [Polygon(p.exterior) for p in contourRegion.geoms]
                elif contourRegion.geom_type == 'GeometryCollection':
                    contourPolys = [Polygon(g.exterior) for g in contourRegion.geoms
                                    if g.geom_type == 'Polygon']
                else:
                    contourPolys = []
                for contourPoly in contourPolys:
                    allPaths.extend(_contourPaths(contourPoly, to3dMat))

        polyGeoms = (list(fillablePoly.geoms)
                     if fillablePoly.geom_type == 'MultiPolygon' else [fillablePoly])
        reverseFlag = False
        for polyItem in polyGeoms:
            rasterPaths = _rasterFillPoly(polyItem, stepOver, to3dMat, zValue, reverseFlag)
            allPaths.extend(rasterPaths)
            reverseFlag = not reverseFlag

    return allPaths


class DropRasterStrategy(IToolpathStrategy):
    def generate(self, targetMesh: trimesh.Trimesh, keepOutMesh: trimesh.Trimesh,
                 toolRadius: float, params: Dict[str, Any],
                 safetyMargin: float) -> List[np.ndarray]:
        params['finishStock'] = float(params.get('roughStock', 0.5))
        return generateDropCutterPaths(targetMesh, keepOutMesh, toolRadius, params, safetyMargin)


class SurfaceProjectionFinishingStrategy(IToolpathStrategy):
    def generate(self, targetMesh: trimesh.Trimesh, keepOutMesh: trimesh.Trimesh,
                 toolRadius: float, params: Dict[str, Any],
                 safetyMargin: float) -> List[np.ndarray]:
        params['finishStock'] = float(params.get('finishStock', 0.03))
        return generateDropCutterPaths(targetMesh, keepOutMesh, toolRadius, params, safetyMargin)


class IsoPlanarPatchFinishingStrategy(IToolpathStrategy):
    def generate(self, targetMesh: trimesh.Trimesh, keepOutMesh: trimesh.Trimesh,
                 toolRadius: float, params: Dict[str, Any],
                 safetyMargin: float) -> List[np.ndarray]:
        params['finishStock'] = float(params.get('finishStock', 0.03))
        return generateDropCutterPaths(targetMesh, keepOutMesh, toolRadius, params, safetyMargin)


class ShellRemovalRoughingStrategy(IToolpathStrategy):
    def generate(self, targetMesh: trimesh.Trimesh, keepOutMesh: trimesh.Trimesh,
                 toolRadius: float, params: Dict[str, Any],
                 safetyMargin: float) -> List[np.ndarray]:
        return generateShellRemovalPaths(
            targetMesh, keepOutMesh, toolRadius, params, safetyMargin)


class ToolpathStrategyFactory:
    strategies = {
        'zlevelroughing': ZLevelRoughingStrategy,
        'zlr': ZLevelRoughingStrategy,
        'dropraster': DropRasterStrategy,
        'surfacefinishing': SurfaceProjectionFinishingStrategy,
        'spf': SurfaceProjectionFinishingStrategy,
        'isoplanarpatchfinishing': IsoPlanarPatchFinishingStrategy,
        'ippf': IsoPlanarPatchFinishingStrategy,
        'shellremovalroughing': ShellRemovalRoughingStrategy,
        'srr': ShellRemovalRoughingStrategy,
        'risergateremoval': RiserGateRemovalStrategy,
        'rgr': RiserGateRemovalStrategy,
    }

    @classmethod
    def getStrategy(cls, mode: str) -> IToolpathStrategy:
        return cls.strategies.get(str(mode).lower(), DropRasterStrategy)()
