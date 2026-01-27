"""
支撑区域检测模块
基于CuraEngine投影法实现悬垂检测
"""

import numpy as np
import trimesh
from typing import List, Tuple
from shapely.geometry import Polygon, MultiPolygon
from shapely import ops


class SupportRegionDetector:
    def __init__(self, config=None):
        self.config = config or {}
        self.supportAngle = float(self.config.get("supportAngle", 45.0))
        self.layerHeight = float(self.config.get("layerHeight", 0.2))
        self.smoothHeight = float(self.config.get("smoothHeight", 0.4))

    def calculateSupportRegions(self, mesh: trimesh.Trimesh, sliceHeights: List[float] = None) -> List[
        trimesh.path.Path2D]:
        """
        计算需要支撑的区域
        输入: 三角网格模型
        输出: 每层需要支撑的2D轮廓列表
        """
        if sliceHeights is None:
            sliceHeights = self._generateSliceHeights(mesh)

        layerOutlines = self._sliceMeshToLayers(mesh, sliceHeights)
        supportRegionsPerLayer = []

        layersBelow = int(round(self.smoothHeight / self.layerHeight))
        maxDistFromLowerLayer = self._calculateMaxBridgeDistance()

        for layerIdx in range(1, len(layerOutlines)):
            basicOverhang = self._computeBasicOverhang(
                layerOutlines,
                layerIdx,
                layersBelow,
                maxDistFromLowerLayer
            )
            supportRegionsPerLayer.append(basicOverhang)

        return supportRegionsPerLayer

    def _generateSliceHeights(self, mesh: trimesh.Trimesh) -> np.ndarray:
        """生成切片高度序列"""
        zMin, zMax = mesh.bounds[:, 2]
        numLayers = int(np.ceil((zMax - zMin) / self.layerHeight))
        return np.linspace(zMin + self.layerHeight / 2, zMax, numLayers)

    def _sliceMeshToLayers(self, mesh: trimesh.Trimesh, heights: np.ndarray) -> List[trimesh.path.Path2D]:
        """
        将网格按指定高度切片为2D轮廓
        参考CuraEngine中逐层切片的实现
        """
        layerOutlines = []

        for height in heights:
            try:
                sliceResult = mesh.section(plane_origin=[0, 0, height], plane_normal=[0, 0, 1])

                if sliceResult is None:
                    layerOutlines.append(None)
                    continue

                path2d, _ = sliceResult.to_planar()
                layerOutlines.append(path2d)

            except Exception:
                layerOutlines.append(None)

        return layerOutlines

    def _calculateMaxBridgeDistance(self) -> float:
        """
        计算最大可桥接距离
        对应CuraEngine中的 max_dist_from_lower_layer = tan(support_angle) * layer_height
        """
        angleRad = np.deg2rad(self.supportAngle)
        tanAngle = np.tan(angleRad)
        return tanAngle * self.layerHeight

    def _computeBasicOverhang(
            self,
            layerOutlines: List[trimesh.path.Path2D],
            layerIdx: int,
            layersBelow: int,
            maxDistFromLowerLayer: float
    ) -> trimesh.path.Path2D:
        """
        计算基础悬垂区域
        对应CuraEngine的computeBasicAndFullOverhang函数核心逻辑

        算法原理:
        1. 获取当前层轮廓 outlines
        2. 获取下方若干层的并集并外扩 maxDist，得到 outlinesBelow
        3. basicOverhang = outlines - outlinesBelow
        """
        currentOutline = layerOutlines[layerIdx]

        if currentOutline is None:
            return None

        mergedBelow = self._getMergedOutlinesBelow(
            layerOutlines,
            layerIdx,
            layersBelow,
            maxDistFromLowerLayer
        )

        if mergedBelow is None:
            return currentOutline

        overhangPolygons = self._subtractPolygons(currentOutline, mergedBelow)

        return overhangPolygons

    def _getMergedOutlinesBelow(
            self,
            layerOutlines: List[trimesh.path.Path2D],
            currentLayerIdx: int,
            layersBelow: int,
            offsetDist: float
    ) -> trimesh.path.Path2D:
        """
        获取下方多层轮廓的并集（带移动平均平滑）
        对应CuraEngine中的多层投影合并逻辑
        """
        if currentLayerIdx == 0:
            return None

        mergedPath = None

        for offset in range(1, min(layersBelow + 1, currentLayerIdx + 1)):
            belowIdx = currentLayerIdx - offset
            belowOutline = layerOutlines[belowIdx]

            if belowOutline is None:
                continue

            expandedOutline = self._offsetPolygon(belowOutline, offsetDist * offset)

            if mergedPath is None:
                mergedPath = expandedOutline
            else:
                mergedPath = self._unionPolygons(mergedPath, expandedOutline)

        return mergedPath

    def _offsetPolygon(self, path: trimesh.path.Path2D, distance: float) -> trimesh.path.Path2D:
        """多边形外扩操作"""
        try:
            if hasattr(path, 'buffer'):
                buffered = path.buffer(distance)
                return buffered

            polygons = []
            for entity in path.entities:
                if hasattr(entity, 'points'):
                    coords = path.vertices[entity.points]
                    if len(coords) >= 3:
                        polygons.append(Polygon(coords))

            if not polygons:
                return None

            multiPoly = ops.unary_union(polygons)
            buffered = multiPoly.buffer(distance)

            return self._shapelyToPath2D(buffered)

        except Exception:
            return path

    def _unionPolygons(self, path1: trimesh.path.Path2D, path2: trimesh.path.Path2D) -> trimesh.path.Path2D:
        """多边形并集操作"""
        try:
            poly1 = self._path2DToShapely(path1)
            poly2 = self._path2DToShapely(path2)

            if poly1 is None or poly2 is None:
                return path1 if path2 is None else path2

            union = ops.unary_union([poly1, poly2])
            return self._shapelyToPath2D(union)

        except Exception:
            return path1

    def _subtractPolygons(self, path1: trimesh.path.Path2D, path2: trimesh.path.Path2D) -> trimesh.path.Path2D:
        """多边形差集操作（path1 - path2）"""
        try:
            poly1 = self._path2DToShapely(path1)
            poly2 = self._path2DToShapely(path2)

            if poly1 is None:
                return None
            if poly2 is None:
                return path1

            difference = poly1.difference(poly2)
            return self._shapelyToPath2D(difference)

        except Exception:
            return path1

    def _path2DToShapely(self, path: trimesh.path.Path2D):
        """根据多边形方向判断外轮廓和孔洞"""
        try:
            if path is None:
                return None

            exteriors = []
            holes = []

            for entity in path.entities:
                if hasattr(entity, 'points'):
                    coords = path.vertices[entity.points]
                    if len(coords) >= 3:
                        poly = Polygon(coords)
                        if poly.exterior.is_ccw:
                            exteriors.append(poly)
                        else:
                            holes.append(poly)

            if not exteriors:
                return None

            result = ops.unary_union(exteriors)

            for hole in holes:
                result = result.difference(hole)

            return result

        except Exception:
            return None

    def _shapelyToPath2D(self, geom):
        """将shapely几何对象转换为trimesh.Path2D"""
        try:
            if geom is None or geom.is_empty:
                return None

            vertices = []
            entities = []

            polygons = []
            if isinstance(geom, Polygon):
                polygons = [geom]
            elif isinstance(geom, MultiPolygon):
                polygons = list(geom.geoms)

            for poly in polygons:
                coords = np.array(poly.exterior.coords[:-1])
                startIdx = len(vertices)
                vertices.extend(coords)
                entities.append(trimesh.path.entities.Line(
                    points=list(range(startIdx, startIdx + len(coords)))
                ))

            if not vertices:
                return None

            return trimesh.path.Path2D(
                vertices=np.array(vertices),
                entities=entities
            )

        except Exception:
            return None


def calculateSupportRegions(moldShell: trimesh.Trimesh, config: dict):
    detector = SupportRegionDetector(config)
    supportRegions = detector.calculateSupportRegions(moldShell)
    validRegions = [region for region in supportRegions if region is not None]
    return validRegions
