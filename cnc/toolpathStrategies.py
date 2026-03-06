from abc import ABC, abstractmethod
from typing import Any, Dict, List
import numpy as np
import trimesh
from shapely.geometry import LineString, MultiPolygon, Polygon
from shapely.ops import unary_union
from shapely.validation import make_valid


class IToolpathStrategy(ABC):
    @abstractmethod
    def generate(self, targetMesh: trimesh.Trimesh, keepOutMesh: trimesh.Trimesh, toolRadius: float,
                 params: Dict[str, Any], safetyMargin: float) -> List[np.ndarray]:
        pass

    def cleanPolygon(self, polygons: List[Any]) -> Any:
        cleaned = []
        for polygon in polygons:
            if polygon is None:
                continue
            if not polygon.is_valid:
                polygon = make_valid(polygon)
            polygon = polygon.buffer(0)
            if not polygon.is_empty:
                cleaned.append(polygon)
        if not cleaned:
            return MultiPolygon()
        return unary_union(cleaned)

    def path2dToPolygons(self, slice2d: Any) -> List[Polygon]:
        polygons = []
        if hasattr(slice2d, 'polygons_full'):
            try:
                for polygon in slice2d.polygons_full:
                    if polygon is not None and not polygon.is_empty:
                        polygons.append(polygon)
            except Exception:
                pass
        if polygons:
            return polygons
        if hasattr(slice2d, 'polygons_closed'):
            try:
                for coords in slice2d.polygons_closed:
                    if len(coords) >= 3:
                        polygon = Polygon(coords)
                        if not polygon.is_empty:
                            polygons.append(polygon)
            except Exception:
                pass
        return polygons

    def robustSection(self, mesh: trimesh.Trimesh, zVal: float, tolerance: float = 0.05) -> List[Polygon]:
        sliceResult = mesh.section(plane_origin=[0, 0, zVal], plane_normal=[0, 0, 1])
        if sliceResult is None:
            sliceResult = mesh.section(plane_origin=[0, 0, zVal - tolerance], plane_normal=[0, 0, 1])
        if sliceResult is None:
            sliceResult = mesh.section(plane_origin=[0, 0, zVal + tolerance], plane_normal=[0, 0, 1])
        if sliceResult is None:
            return []
        slice2d, _ = sliceResult.to_2D()
        return self.path2dToPolygons(slice2d)


class ZLevelRoughingStrategy(IToolpathStrategy):
    def generate(self, targetMesh: trimesh.Trimesh, keepOutMesh: trimesh.Trimesh, toolRadius: float,
                 params: Dict[str, Any], safetyMargin: float) -> List[np.ndarray]:
        if targetMesh.is_empty:
            return []
        stepOver = float(params.get('stepOver', 1.0))
        layerStep = float(params.get('layerStep', 1.0))
        projectionStep = float(params.get('projectionStep', 0.5))
        bounds = np.asarray(targetMesh.bounds, dtype=float)
        zMin = float(bounds[0][2])
        zMax = float(bounds[1][2])
        zLevels = np.arange(zMax - layerStep, zMin, -layerStep, dtype=float)
        paths = []
        for zVal in zLevels:
            targetPolygons = self.robustSection(targetMesh, float(zVal))
            if not targetPolygons:
                continue
            targetSliceResult = targetMesh.section(plane_origin=[0, 0, float(zVal)], plane_normal=[0, 0, 1])
            if targetSliceResult is None:
                continue
            _, to3dMatrix = targetSliceResult.to_2D()
            machinableUnion = self.cleanPolygon(targetPolygons)
            machinablePoly = machinableUnion.buffer(-toolRadius)
            if machinablePoly.is_empty:
                continue
            keepOutProjected = MultiPolygon()
            keepOutCurrent = MultiPolygon()
            if not keepOutMesh.is_empty:
                keepOutTop = float(keepOutMesh.bounds[1][2])
                zLevelsKeepOut = np.arange(float(zVal), keepOutTop + 0.1, projectionStep, dtype=float)
                projectedPolygons = []
                for keepOutZ in zLevelsKeepOut:
                    slicePolygons = self.robustSection(keepOutMesh, float(keepOutZ))
                    if slicePolygons:
                        projectedPolygons.extend(slicePolygons)
                if projectedPolygons:
                    keepOutProjected = self.cleanPolygon(projectedPolygons).buffer(toolRadius + safetyMargin)
                currentPolygons = self.robustSection(keepOutMesh, float(zVal))
                if currentPolygons:
                    keepOutCurrent = self.cleanPolygon(currentPolygons).buffer(toolRadius + safetyMargin + stepOver)
            safePoly = machinablePoly.buffer(0)
            if not keepOutProjected.is_empty:
                safePoly = machinablePoly.difference(keepOutProjected).buffer(0)
            if safePoly.is_empty and not keepOutCurrent.is_empty:
                safePoly = machinablePoly.difference(keepOutCurrent).buffer(0)
            if safePoly.is_empty:
                continue
            geometryList = safePoly.geoms if safePoly.geom_type == 'MultiPolygon' else [safePoly]
            for geometry in geometryList:
                if geometry.is_empty:
                    continue
                minX, minY, maxX, maxY = geometry.bounds
                yValues = np.arange(minY, maxY + stepOver * 0.5, stepOver, dtype=float)
                reverseFlag = False
                for yVal in yValues:
                    scanLine = LineString([(minX - 1.0, yVal), (maxX + 1.0, yVal)])
                    intersection = scanLine.intersection(geometry)
                    if intersection.is_empty:
                        reverseFlag = not reverseFlag
                        continue
                    lineList = intersection.geoms if intersection.geom_type == 'MultiLineString' else [intersection]
                    for lineObj in lineList:
                        if lineObj.geom_type != 'LineString':
                            continue
                        coords2d = list(lineObj.coords)
                        if len(coords2d) < 2:
                            continue
                        if reverseFlag:
                            coords2d.reverse()
                        coordsHomo = np.column_stack((np.asarray(coords2d, dtype=float), np.zeros(len(coords2d)), np.ones(len(coords2d))))
                        coords3d = (to3dMatrix @ coordsHomo.T).T[:, :3]
                        paths.append(np.asarray(coords3d, dtype=float))
                    reverseFlag = not reverseFlag
        return paths


class WaterlineStrategy(IToolpathStrategy):
    def generate(self, targetMesh: trimesh.Trimesh, keepOutMesh: trimesh.Trimesh, toolRadius: float,
                 params: Dict[str, Any], safetyMargin: float) -> List[np.ndarray]:
        if targetMesh.is_empty:
            return []
        layerStep = float(params.get('layerStep', 0.5))
        bounds = np.asarray(targetMesh.bounds, dtype=float)
        zMin = float(bounds[0][2])
        zMax = float(bounds[1][2])
        zLevels = np.arange(zMin, zMax + layerStep, layerStep, dtype=float)
        paths = []
        for zVal in zLevels:
            sliceResult = targetMesh.section(plane_origin=[0, 0, float(zVal)], plane_normal=[0, 0, 1])
            if sliceResult is None:
                continue
            slice2d, to3dMatrix = sliceResult.to_2D()
            polygons = self.path2dToPolygons(slice2d)
            if not polygons:
                continue
            unionPoly = self.cleanPolygon(polygons)
            offsetPoly = unionPoly.buffer(toolRadius)
            if offsetPoly.is_empty:
                continue
            geometryList = offsetPoly.geoms if offsetPoly.geom_type == 'MultiPolygon' else [offsetPoly]
            for geometry in geometryList:
                coords2d = np.asarray(geometry.exterior.coords, dtype=float)
                if len(coords2d) < 2:
                    continue
                coordsHomo = np.column_stack((coords2d, np.zeros(len(coords2d)), np.ones(len(coords2d))))
                coords3d = (to3dMatrix @ coordsHomo.T).T[:, :3]
                paths.append(np.asarray(coords3d, dtype=float))
        return paths


class DropRasterStrategy(IToolpathStrategy):
    def generate(self, targetMesh: trimesh.Trimesh, keepOutMesh: trimesh.Trimesh, toolRadius: float,
                 params: Dict[str, Any], safetyMargin: float) -> List[np.ndarray]:
        if targetMesh.is_empty:
            return []
        bounds = np.asarray(targetMesh.bounds, dtype=float)
        stepOver = float(params.get('stepOver', 1.0))
        safeHeight = float(params.get('safeHeight', 5.0))
        angleThreshold = float(params.get('angleThreshold', 1.047))
        minZNormal = float(np.cos(angleThreshold))
        xMin, yMin, _ = bounds[0]
        xMax, yMax, zMax = bounds[1]
        xValues = np.arange(xMin, xMax + stepOver, stepOver, dtype=float)
        yValues = np.arange(yMin, yMax + stepOver, stepOver, dtype=float)
        zStart = float(zMax + safeHeight)
        paths = []
        for yVal in yValues:
            rayOrigins = np.column_stack((xValues, np.full(len(xValues), yVal), np.full(len(xValues), zStart)))
            rayDirections = np.tile([0.0, 0.0, -1.0], (len(xValues), 1))
            locations, rayIndices, triIndices = targetMesh.ray.intersects_location(ray_origins=rayOrigins, ray_directions=rayDirections)
            linePoints = []
            for rayIndex in range(len(xValues)):
                hitMask = np.where(rayIndices == rayIndex)[0]
                if len(hitMask) == 0:
                    continue
                topHit = hitMask[np.argmax(locations[hitMask, 2])]
                triIndex = int(triIndices[topHit])
                normalVec = targetMesh.face_normals[triIndex]
                if normalVec[2] >= minZNormal:
                    linePoints.append([float(xValues[rayIndex]), float(yVal), float(locations[topHit, 2])])
            if len(linePoints) >= 2:
                paths.append(np.asarray(linePoints, dtype=float))
        return paths


class ToolpathStrategyFactory:
    strategyMap = {
        'zlevelroughing': ZLevelRoughingStrategy(),
        'zlr': ZLevelRoughingStrategy(),
        'waterline': WaterlineStrategy(),
        'wl': WaterlineStrategy(),
        'dropraster': DropRasterStrategy()
    }

    @classmethod
    def getStrategy(cls, mode: str) -> IToolpathStrategy:
        return cls.strategyMap.get(mode.lower(), DropRasterStrategy())
