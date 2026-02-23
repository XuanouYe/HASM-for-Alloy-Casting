import numpy as np
import trimesh
from typing import Optional, Tuple
from shapely.geometry import Polygon, MultiPolygon
from shapely.geometry.base import BaseGeometry
import vtk
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
            print(f"\n=== Iteration {iterationCount + 1} ===")

            sliceHeights = self.detector._generateSliceHeights(currentMesh)
            layerPaths = self.detector._sliceMeshToLayers(currentMesh, sliceHeights)
            layerGeoms = [self.detector._path2DToShapely(p) for p in layerPaths]
            supportRegions = self.detector.calculateSupportRegions(currentMesh, sliceHeights)

            validLayers = sum(1 for r in supportRegions if r is not None and not r.is_empty)
            print(f"Found {validLayers} layers with support regions")

            if validLayers == 0:
                print("No more support regions detected. Converged.")
                break

            zMin = float(currentMesh.bounds[0, 2])
            removalVolumes = []

            for supportIdx, supportGeom in enumerate(supportRegions):
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
                print("No removal volumes generated. Stopping.")
                break

            print(f"Generated {len(removalVolumes)} removal volumes")
            mergedRemoval = trimesh.util.concatenate(removalVolumes)

            try:
                resultMesh = currentMesh.difference(mergedRemoval)
                if not isinstance(resultMesh, trimesh.Trimesh):
                    print("Boolean operation returned invalid result. Stopping.")
                    break

                vertexChange = abs(resultMesh.vertices.shape[0] - currentMesh.vertices.shape[0])
                print(f"Mesh updated: {resultMesh.vertices.shape[0]} vertices (change: {vertexChange})")

                if vertexChange < 3:
                    print("Mesh change too small. Converged.")
                    break

                currentMesh = resultMesh
                iterationCount += 1

            except Exception as e:
                print(f"Boolean operation failed: {e}")
                break

        if iterationCount >= self.maxIterations:
            print(f"\nReached maximum iterations ({self.maxIterations})")

        return currentMesh

    def _createTransitionVolume(self, overhangPolygon: BaseGeometry, zBottom: float, height: float) -> Optional[
        trimesh.Trimesh]:
        if overhangPolygon is None or overhangPolygon.is_empty or overhangPolygon.area < self.areaEps:
            return None

        poly = overhangPolygon.buffer(float(self.offsetDistance))
        if poly.is_empty or poly.area < self.areaEps:
            return None

        slab = self._shapelyToTrimesh(poly, zBottom, height)
        return slab

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
            try:
                m = trimesh.creation.extrude_polygon(poly, height=float(height))
                m.apply_translation([0.0, 0.0, float(zBottom)])
                meshes.append(m)
            except Exception:
                continue

        if not meshes:
            return None
        return meshes[0] if len(meshes) == 1 else trimesh.util.concatenate(meshes)


class OffsetVisualizer:
    def __init__(self, windowSize: Tuple[int, int] = (1200, 800)):
        self.windowSize = windowSize

    def _createVTKMeshActor(self, mesh: trimesh.Trimesh, color: Tuple[float, float, float],
                            opacity: float) -> vtk.vtkActor:
        vertices = mesh.vertices
        faces = mesh.faces

        vtkPoints = vtk.vtkPoints()
        for v in vertices:
            vtkPoints.InsertNextPoint(float(v[0]), float(v[1]), float(v[2]))

        vtkCells = vtk.vtkCellArray()
        for f in faces:
            tri = vtk.vtkTriangle()
            tri.GetPointIds().SetId(0, int(f[0]))
            tri.GetPointIds().SetId(1, int(f[1]))
            tri.GetPointIds().SetId(2, int(f[2]))
            vtkCells.InsertNextCell(tri)

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

    def visualizeComparison(self, originalMesh: trimesh.Trimesh, processedMesh: trimesh.Trimesh) -> None:
        renderer = vtk.vtkRenderer()
        renderer.SetBackground(0.95, 0.95, 0.95)

        originalActor = self._createVTKMeshActor(originalMesh, (0.8, 0.2, 0.2), 0.35)
        originalActor.GetProperty().SetEdgeVisibility(True)
        renderer.AddActor(originalActor)

        processedActor = self._createVTKMeshActor(processedMesh, (0.2, 0.6, 0.8), 0.85)
        processedActor.GetProperty().SetEdgeVisibility(True)
        renderer.AddActor(processedActor)

        textActor = vtk.vtkTextActor()
        textActor.SetInput("Red: Original\nBlue: Processed")
        textActor.GetTextProperty().SetColor(0, 0, 0)
        textActor.GetTextProperty().SetFontSize(14)
        textActor.SetPosition(10, 10)
        renderer.AddViewProp(textActor)

        renderWindow = vtk.vtkRenderWindow()
        renderWindow.AddRenderer(renderer)
        renderWindow.SetSize(self.windowSize[0], self.windowSize[1])
        renderWindow.SetWindowName("Inner Surface Offset Comparison")

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
        "layerHeight": 0.2,
        "smoothHeight": 0.4,
        "areaEps": 1e-4,
        "maxIterations": 20
    }

    stlPath = "testModels/hollow.cylinder.down.stl"
    mesh = trimesh.load_mesh(stlPath)
    print(f"Loaded mesh: {mesh.vertices.shape[0]} vertices, {mesh.faces.shape[0]} faces")
    print(f"Original mesh watertight: {mesh.is_watertight}")

    print("\n=== Initial Detection ===")
    detector = SupportRegionDetector(config)
    initialSupportRegions = detector.calculateSupportRegions(mesh)
    initialValidLayers = sum(1 for r in initialSupportRegions if r is not None)
    print(f"Initial support regions: {initialValidLayers} layers")

    print("\n=== Processing ===")
    offsetProcessor = InnerSurfaceOffset(config)
    processedMesh = offsetProcessor.removeInnerSurfaceOverhangs(mesh)

    print("\n=== Final Verification ===")
    finalSupportRegions = detector.calculateSupportRegions(processedMesh)
    finalValidLayers = sum(1 for r in finalSupportRegions if r is not None)

    print(f"Processed mesh: {processedMesh.vertices.shape[0]} vertices, {processedMesh.faces.shape[0]} faces")
    print(f"Processed mesh watertight: {processedMesh.is_watertight}")
    print(f"Final support regions: {finalValidLayers} layers")
    print(f"Total reduction: {initialValidLayers - finalValidLayers} layers")

    outputPath = stlPath.replace(".stl", ".offset.stl")
    processedMesh.export(outputPath)
    print(f"\nSaved to: {outputPath}")

    print("\n=== Visualization ===")
    visualizer = OffsetVisualizer()
    visualizer.visualizeComparison(mesh, processedMesh)


if __name__ == "__main__":
    main()