from abc import ABC, abstractmethod
from typing import Any, Dict, List, Optional, Tuple
import numpy as np
import scipy.ndimage as nd
import trimesh
from shapely.geometry import GeometryCollection, LineString, MultiPolygon, Polygon
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


def robustSection(mesh: trimesh.Trimesh, zValue: float, tolerance: float = 0.05) -> Any:
    def trySection(zVal: float):
        try:
            sectionResult = mesh.section(
                plane_origin=[0.0, 0.0, zVal],
                plane_normal=[0.0, 0.0, 1.0])
            if sectionResult is None:
                return None
            slice2d, _ = sectionResult.to_2D()
            polys = slice2d.polygons_full
            return polys if polys else None
        except Exception:
            return None

    result = trySection(zValue)
    if result is not None:
        return result
    result = trySection(zValue - tolerance)
    if result is not None:
        return result
    return trySection(zValue + tolerance)


def robustSectionWithMat(mesh: trimesh.Trimesh, zValue: float,
                          tolerance: float = 0.05) -> Tuple[Optional[Any], Optional[np.ndarray]]:
    def trySectionWithMat(zVal: float):
        try:
            sec = mesh.section(
                plane_origin=[0.0, 0.0, zVal],
                plane_normal=[0.0, 0.0, 1.0])
            if sec is None:
                return None, None
            sl2d, mat = sec.to_2D()
            polys = sl2d.polygons_full
            if not polys:
                return None, None
            return polys, mat
        except Exception:
            return None, None

    polys, mat = trySectionWithMat(zValue)
    if polys is not None:
        return polys, mat
    polys, mat = trySectionWithMat(zValue - tolerance)
    if polys is not None:
        return polys, mat
    return trySectionWithMat(zValue + tolerance)


def robustSectionIn2d(mesh: trimesh.Trimesh, zValue: float,
                      to3dMat: np.ndarray, tolerance: float = 0.05) -> Any:
    invMat = np.linalg.inv(to3dMat)

    def trySectionIn2d(zVal: float):
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

    result = trySectionIn2d(zValue)
    if result is not None:
        return result
    result = trySectionIn2d(zValue - tolerance)
    if result is not None:
        return result
    return trySectionIn2d(zValue + tolerance)


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
            targetPolys, to3dMat = robustSectionWithMat(targetMesh, zValue)
            if targetPolys is None or to3dMat is None:
                continue
            machinablePoly = cleanPolygon(targetPolys).buffer(-(toolRadius + roughStock))
            keepOutPoly = MultiPolygon()
            if keepOutMesh is not None and not keepOutMesh.is_empty:
                keepOutPolys2d = robustSectionIn2d(keepOutMesh, zValue, to3dMat)
                if keepOutPolys2d:
                    keepOutPoly = cleanPolygon(keepOutPolys2d).buffer(
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
                minX, minY, maxX, maxY = polyItem.bounds
                yList = np.arange(minY, maxY + stepOver * 0.5, stepOver, dtype=float)
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
                            allPaths.append(path3d)
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
            targetPolys, to3dMat = robustSectionWithMat(targetMesh, zValue)
            if targetPolys is None or to3dMat is None:
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
    return generateStep1LayerPaths(targetMesh, keepOutMesh, toolRadius, params, safetyMargin)


def sliceMeshAtZ(mesh: trimesh.Trimesh, zValue: float,
                 tolerance: float = 0.05) -> Optional[Any]:
    if mesh is None or mesh.is_empty:
        return None

    def trySlice(zTry: float) -> Optional[Any]:
        try:
            section = mesh.section(
                plane_origin=[0.0, 0.0, zTry],
                plane_normal=[0.0, 0.0, 1.0])
            if section is None or len(section.vertices) == 0:
                return None
            path2d, _ = section.to_2D()
            vertices2d = np.asarray(path2d.vertices, dtype=float)
            lineList = []
            for entity in path2d.entities:
                pointIds = np.asarray(entity.points, dtype=int)
                if len(pointIds) < 2:
                    continue
                ringCoords = vertices2d[pointIds]
                if np.linalg.norm(ringCoords[0] - ringCoords[-1]) > 1e-9:
                    ringCoords = np.vstack([ringCoords, ringCoords[0]])
                polyItem = Polygon(ringCoords)
                if not polyItem.is_empty and polyItem.area > 1e-9:
                    lineList.append(polyItem)
            if not lineList:
                return None
            merged = make_valid(unary_union(lineList))
            return merged if not merged.is_empty else None
        except Exception:
            return None

    for zTry in [zValue, zValue - tolerance, zValue + tolerance]:
        result = trySlice(zTry)
        if result is not None:
            return result
    return None


def buildLayerSafePoly(moldMesh: trimesh.Trimesh, keepOutMesh: trimesh.Trimesh,
                       zValue: float, toolRadius: float, roughStock: float,
                       safetyMargin: float, tolerance: float) -> Tuple[Any, Any]:
    moldPoly = sliceMeshAtZ(moldMesh, zValue, tolerance)
    if moldPoly is None or moldPoly.is_empty:
        return MultiPolygon(), MultiPolygon()
    keepOutExpanded = MultiPolygon()
    if keepOutMesh is not None and not keepOutMesh.is_empty:
        keepOutRaw = sliceMeshAtZ(keepOutMesh, zValue, tolerance)
        if keepOutRaw is not None and not keepOutRaw.is_empty:
            keepOutExpanded = make_valid(keepOutRaw.buffer(toolRadius + safetyMargin))
    insetPoly = moldPoly.buffer(-(toolRadius + roughStock))
    if insetPoly.is_empty:
        insetPoly = moldPoly.buffer(-toolRadius)
    if not keepOutExpanded.is_empty:
        insetPoly = insetPoly.difference(keepOutExpanded)
    return make_valid(insetPoly), keepOutExpanded


def generateZigzagPaths(fillPoly: Any, stepOver: float,
                        reverseDir: bool = False) -> List[np.ndarray]:
    if fillPoly is None or fillPoly.is_empty or stepOver <= 0.0:
        return []
    minX, minY, maxX, maxY = fillPoly.bounds
    yLevels = np.arange(minY + stepOver * 0.5, maxY + stepOver * 0.25, stepOver, dtype=float)
    allPaths: List[np.ndarray] = []
    reverseFlag = bool(reverseDir)
    for yValue in yLevels:
        scanLine = LineString([(minX - 1.0, yValue), (maxX + 1.0, yValue)])
        intersection = fillPoly.intersection(scanLine)
        lineItems = extractLineStrings(intersection)
        if not lineItems:
            continue
        sortedLines = sorted(lineItems, key=lambda ln: float(ln.bounds[0]), reverse=reverseFlag)
        for lineGeom in sortedLines:
            coords = np.asarray(lineGeom.coords, dtype=float)
            if len(coords) < 2:
                continue
            if reverseFlag:
                coords = coords[::-1]
            path3d = np.column_stack([coords[:, 0], coords[:, 1], np.zeros(len(coords))])
            allPaths.append(path3d.astype(float))
        reverseFlag = not reverseFlag
    return allPaths


def generateContourPaths(fillPoly: Any, toolRadius: float,
                         stepOver: float, passes: int = 2) -> List[np.ndarray]:
    if fillPoly is None or fillPoly.is_empty:
        return []
    contourPaths: List[np.ndarray] = []
    for i in range(max(1, int(passes))):
        offsetDist = float(i) * float(stepOver)
        contourPoly = fillPoly.buffer(-offsetDist)
        if contourPoly.is_empty:
            break
        polyList = list(contourPoly.geoms) if contourPoly.geom_type == "MultiPolygon" else [contourPoly]
        for polyItem in polyList:
            if polyItem.is_empty:
                continue
            coords = np.asarray(polyItem.exterior.coords, dtype=float)
            if len(coords) < 2:
                continue
            contourPaths.append(np.column_stack([coords[:, 0], coords[:, 1], np.zeros(len(coords))]))
    return contourPaths


def connectPathsAvoidingObstacle(rawPaths: List[np.ndarray], safePoly: Any,
                                 directThreshold: float) -> List[np.ndarray]:
    if not rawPaths:
        return []
    mergedPaths: List[np.ndarray] = []
    currentSeg = np.asarray(rawPaths[0], dtype=float).copy()
    for idx in range(1, len(rawPaths)):
        nextSeg = np.asarray(rawPaths[idx], dtype=float)
        if len(nextSeg) < 2:
            continue
        endPt = currentSeg[-1]
        startPt = nextSeg[0]
        dist = float(np.linalg.norm(startPt[:2] - endPt[:2]))
        canLink = False
        if dist <= float(directThreshold):
            linkLine = LineString([(float(endPt[0]), float(endPt[1])),
                                   (float(startPt[0]), float(startPt[1]))])
            canLink = bool(safePoly.covers(linkLine))
        if canLink:
            if np.linalg.norm(currentSeg[-1] - nextSeg[0]) > 1e-9:
                currentSeg = np.vstack([currentSeg, nextSeg[0]])
            currentSeg = np.vstack([currentSeg, nextSeg[1:]])
        else:
            if len(currentSeg) >= 2:
                mergedPaths.append(currentSeg)
            currentSeg = nextSeg.copy()
    if len(currentSeg) >= 2:
        mergedPaths.append(currentSeg)
    return mergedPaths


class ShapelyLayerIpw:
    def __init__(self):
        self.machinedUnion: Any = GeometryCollection()

    def getRemainingPoly(self, targetPoly: Any) -> Any:
        if targetPoly is None or targetPoly.is_empty:
            return MultiPolygon()
        if self.machinedUnion.is_empty:
            return targetPoly
        return make_valid(targetPoly.difference(self.machinedUnion))

    def isFullyCovered(self, targetPoly: Any, threshold: float = 0.90) -> bool:
        if targetPoly is None or targetPoly.is_empty:
            return True
        totalArea = float(targetPoly.area)
        if totalArea <= 1e-9:
            return True
        remaining = self.getRemainingPoly(targetPoly)
        coveredRatio = 1.0 - float(remaining.area) / totalArea
        return coveredRatio >= float(threshold)

    def markMachined(self, machinedPoly: Any) -> None:
        if machinedPoly is None or machinedPoly.is_empty:
            return
        if self.machinedUnion.is_empty:
            self.machinedUnion = make_valid(machinedPoly)
            return
        self.machinedUnion = make_valid(unary_union([self.machinedUnion, machinedPoly]))

    def projectAndMark(self, pathsLocal: List[np.ndarray], toolRadius: float) -> None:
        if not pathsLocal:
            return
        sweepPolys = []
        for pathItem in pathsLocal:
            pathArr = np.asarray(pathItem, dtype=float)
            if len(pathArr) < 2:
                continue
            sweep = LineString(pathArr[:, :2]).buffer(float(toolRadius))
            if not sweep.is_empty:
                sweepPolys.append(sweep)
        if sweepPolys:
            self.markMachined(unary_union(sweepPolys))


def generateStep1LayerPaths(moldMesh: trimesh.Trimesh, keepOutMesh: trimesh.Trimesh,
                            toolRadius: float, params: Dict[str, Any],
                            safetyMargin: float,
                            layerIpw: Optional[ShapelyLayerIpw] = None) -> List[np.ndarray]:
    if moldMesh is None or moldMesh.is_empty:
        return []
    stepOver = float(params.get("stepOver", toolRadius * 1.5))
    layerStep = float(params.get("layerStep", toolRadius * 1.2))
    roughStock = float(params.get("roughStock", toolRadius * 0.3))
    useContour = bool(params.get("useContour", params.get("step1UseContour", True)))
    contourPasses = int(params.get("contourPasses", params.get("step1ContourPasses", 2)))
    tolerance = float(params.get("tolerance", params.get("sectionTolerance", 0.05)))
    ipwThreshold = float(params.get("ipwThreshold", 0.90))
    directThreshold = float(params.get("directThreshold", stepOver * 2.5))
    bottomClearance = float(params.get("bottomClearance", 0.0))
    boundsArray = np.asarray(moldMesh.bounds, dtype=float)
    zMin = float(boundsArray[0, 2])
    zMax = float(boundsArray[1, 2])
    zStart = zMax - layerStep * 0.5
    zLevels = np.arange(zStart, zMin - layerStep * 0.1, -layerStep, dtype=float)
    if len(zLevels) == 0 or zLevels[-1] > zMin + layerStep * 0.1:
        zLevels = np.append(zLevels, zMin)
    localMinZ = zMin + bottomClearance if bottomClearance > 0.0 else -np.inf
    allPaths: List[np.ndarray] = []
    for zVal in zLevels:
        if zVal < localMinZ:
            continue
        safePoly, _ = buildLayerSafePoly(
            moldMesh, keepOutMesh, float(zVal), toolRadius, roughStock, safetyMargin, tolerance)
        if safePoly.is_empty:
            continue
        activePoly = safePoly
        if layerIpw is not None:
            if layerIpw.isFullyCovered(safePoly, ipwThreshold):
                continue
            activePoly = layerIpw.getRemainingPoly(safePoly)
            if activePoly.is_empty:
                continue
        rawPaths: List[np.ndarray] = []
        if useContour:
            rawPaths.extend(generateContourPaths(activePoly, toolRadius, stepOver, contourPasses))
        polyList = list(activePoly.geoms) if activePoly.geom_type == "MultiPolygon" else [activePoly]
        reverseDir = False
        for polyItem in polyList:
            if polyItem.is_empty:
                continue
            zigzagPaths = generateZigzagPaths(polyItem, stepOver, reverseDir)
            if not zigzagPaths:
                zigzagPaths = generateContourPaths(polyItem, toolRadius, stepOver, contourPasses)
            rawPaths.extend(zigzagPaths)
            reverseDir = not reverseDir
        if not rawPaths:
            continue
        connectedPaths = connectPathsAvoidingObstacle(rawPaths, safePoly, directThreshold)
        if not connectedPaths:
            continue
        for pathItem in connectedPaths:
            pathZ = np.asarray(pathItem, dtype=float).copy()
            pathZ[:, 2] = float(zVal)
            allPaths.append(pathZ)
        if layerIpw is not None:
            layerIpw.projectAndMark(connectedPaths, toolRadius)
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
