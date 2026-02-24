import numpy as np
import trimesh
from typing import List, Optional, Tuple
from shapely import ops
from shapely.geometry import Polygon, MultiPolygon
from shapely.geometry.base import BaseGeometry
import vtk


class SupportRegionDetector:
    def __init__(self, config: Optional[dict] = None):
        self.config = config or {}
        self.supportAngle = float(self.config.get("supportAngle", 45.0))
        self.layerHeight = float(self.config.get("layerHeight", 0.2))
        self.smoothHeight = float(self.config.get("smoothHeight", 0.4))
        self.areaEps = float(self.config.get("areaEps", 1e-6))

    def calculateSupportRegions(self, mesh: trimesh.Trimesh,
                                sliceHeights: Optional[List[float]] = None) -> List[Optional[BaseGeometry]]:
        if sliceHeights is None:
            sliceHeights = self._generateSliceHeights(mesh)
        layerPaths = self._sliceMeshToLayers(mesh, sliceHeights)
        layerGeoms = [self._path2DToShapely(p) for p in layerPaths]
        layersBelow = int(round(self.smoothHeight / self.layerHeight))
        maxDistFromLowerLayer = self._calculateMaxBridgeDistance()
        supportRegionsPerLayer = []
        for layerIdx in range(1, len(layerGeoms)):
            cur = layerGeoms[layerIdx]
            if cur is None or cur.is_empty:
                supportRegionsPerLayer.append(None)
                continue
            mergedBelow = self._getMergedGeomsBelow(layerGeoms, layerIdx, layersBelow, maxDistFromLowerLayer)
            if mergedBelow is None or mergedBelow.is_empty:
                supportRegionsPerLayer.append(cur)
                continue
            diff = cur.difference(mergedBelow)
            if diff.is_empty or diff.area < self.areaEps:
                supportRegionsPerLayer.append(None)
            else:
                supportRegionsPerLayer.append(diff)
        return supportRegionsPerLayer

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


def calculateSupportRegions(moldShell: trimesh.Trimesh, config: dict) -> List[BaseGeometry]:
    detector = SupportRegionDetector(config)
    supportRegions = detector.calculateSupportRegions(moldShell)
    return [r for r in supportRegions if r is not None]


class SupportVisualizer:
    def __init__(self, windowSize: Tuple[int, int] = (1200, 800)):
        self.windowSize = windowSize

    def _shapelyPolygonToTrimesh(self, polygon: BaseGeometry, zBottom: float, height: float) -> Optional[
        trimesh.Trimesh]:
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
            if not poly.is_valid or poly.area < 1e-9:
                continue
            try:
                mesh = trimesh.creation.extrude_polygon(poly, height=height)
                mesh.apply_translation([0, 0, zBottom])
                meshes.append(mesh)
            except Exception:
                continue
        if not meshes:
            return None
        if len(meshes) == 1:
            return meshes[0]
        return trimesh.util.concatenate(meshes)

    def _createVTKMeshActor(self, mesh: trimesh.Trimesh, color: Tuple[float, float, float],
                            opacity: float) -> vtk.vtkActor:
        vertices = mesh.vertices
        faces = mesh.faces

        vtkPoints = vtk.vtkPoints()
        for v in vertices:
            vtkPoints.InsertNextPoint(v)

        vtkCells = vtk.vtkCellArray()
        for f in faces:
            triangle = vtk.vtkTriangle()
            triangle.GetPointIds().SetId(0, f[0])
            triangle.GetPointIds().SetId(1, f[1])
            triangle.GetPointIds().SetId(2, f[2])
            vtkCells.InsertNextCell(triangle)

        polyData = vtk.vtkPolyData()
        polyData.SetPoints(vtkPoints)
        polyData.SetPolys(vtkCells)

        normals = vtk.vtkPolyDataNormals()
        normals.SetInputData(polyData)
        normals.ComputePointNormalsOn()
        normals.Update()

        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(normals.GetOutputPort())

        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(color)
        actor.GetProperty().SetOpacity(opacity)

        return actor

    def visualize(self, mesh: trimesh.Trimesh, supportRegions: List[Optional[BaseGeometry]], config: dict) -> None:
        print("Generating visualization data...")

        renderer = vtk.vtkRenderer()
        renderer.SetBackground(1.0, 1.0, 1.0)

        meshActor = self._createVTKMeshActor(mesh, color=(0.8, 0.8, 0.8), opacity=0.3)
        meshActor.GetProperty().SetEdgeVisibility(True)
        meshActor.GetProperty().SetEdgeColor(0.2, 0.2, 0.2)
        renderer.AddActor(meshActor)

        layerHeight = float(config.get("layerHeight", 0.2))
        zMin = mesh.bounds[0, 2]

        allSupportMeshes = []

        print(f"Processing {len(supportRegions)} layers for support visualization...")

        firstLayerZ = zMin + layerHeight / 2.0

        for i, region in enumerate(supportRegions):
            if region is None:
                continue

            currentZ = firstLayerZ + (i * layerHeight)

            supportMesh = self._shapelyPolygonToTrimesh(region, zBottom=currentZ, height=layerHeight)
            if supportMesh:
                allSupportMeshes.append(supportMesh)

        if allSupportMeshes:
            print(f"Merging {len(allSupportMeshes)} support volumes...")
            combinedSupport = trimesh.util.concatenate(allSupportMeshes)
            supportActor = self._createVTKMeshActor(combinedSupport, color=(1.0, 0.2, 0.2), opacity=1.0)
            renderer.AddActor(supportActor)
        else:
            print("No support regions detected to visualize.")

        textActor = vtk.vtkTextActor()
        textActor.SetInput("Grey: Mold Shell\nRed: Support Overhangs (>45 deg)")
        textActor.GetTextProperty().SetColor(0, 0, 0)
        textActor.GetTextProperty().SetFontSize(14)
        textActor.SetPosition(10, 10)
        renderer.AddViewProp(textActor)

        renderWindow = vtk.vtkRenderWindow()
        renderWindow.AddRenderer(renderer)
        renderWindow.SetSize(self.windowSize[0], self.windowSize[1])
        renderWindow.SetWindowName("Liquid Metal Mold Support Visualization")

        interactor = vtk.vtkRenderWindowInteractor()
        interactor.SetRenderWindow(renderWindow)

        axes = vtk.vtkAxesActor()
        axesWidget = vtk.vtkOrientationMarkerWidget()
        axesWidget.SetOrientationMarker(axes)
        axesWidget.SetInteractor(interactor)
        axesWidget.SetEnabled(1)
        axesWidget.InteractiveOff()

        renderer.ResetCamera()
        renderer.GetActiveCamera().Azimuth(30)
        renderer.GetActiveCamera().Elevation(30)

        interactor.Initialize()
        renderWindow.Render()
        interactor.Start()


def main():
    config = {
        "supportAngle": 45.0,
        "layerHeight": 0.5,
        "smoothHeight": 1.0,
        "areaEps": 1e-4
    }

    stlPath = "testModels/hollow.cylinder.down.stl"
    mesh = trimesh.load_mesh(stlPath)
    detector = SupportRegionDetector(config)
    supportRegions = detector.calculateSupportRegions(mesh)
    validLayers = sum(1 for r in supportRegions if r is not None)
    print(f"Found support regions on {validLayers} layers.")
    visualizer = SupportVisualizer()
    visualizer.visualize(mesh, supportRegions, config)


if __name__ == "__main__":
    main()
