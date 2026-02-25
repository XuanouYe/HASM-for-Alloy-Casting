import trimesh
import numpy as np
from typing import Tuple, Optional, Dict
import vtk


class AccessibilityAnalyzer:
    def __init__(self, meshPath: str):
        self.meshPath = meshPath
        self.mesh = trimesh.load_mesh(meshPath)
        self.mesh.fix_normals()
        self.zMin = self.mesh.bounds[0, 2]

    def _generateHemisphereDirections(self, normal: np.ndarray, numRays: int = 64) -> np.ndarray:
        goldenRatio = (1 + np.sqrt(5)) / 2
        indices = np.arange(numRays)
        theta = 2 * np.pi * indices / goldenRatio
        phi = np.arccos(1 - 2 * (indices + 0.5) / numRays)
        x = np.sin(phi) * np.cos(theta)
        y = np.sin(phi) * np.sin(theta)
        z = np.cos(phi)
        fibDirections = np.stack([x, y, z], axis=-1)

        numHorizontal = 90
        thetaH = np.linspace(0, 2 * np.pi, numHorizontal, endpoint=False)
        xH = np.cos(thetaH)
        yH = np.sin(thetaH)
        zH = np.zeros_like(thetaH)
        horizDirections = np.stack([xH, yH, zH], axis=-1)

        allDirections = np.vstack([fibDirections, horizDirections])

        normalNormalized = normal / np.linalg.norm(normal)
        dotProducts = allDirections @ normalNormalized

        validMask = dotProducts >= 1e-6
        return allDirections[validMask]

    def _checkPlaneIntersection(self, origins: np.ndarray, directions: np.ndarray) -> np.ndarray:
        hits = directions[:, 2] < -1e-6
        return hits

    def _exportData(self, points, unmachinableMask, filePath="accessibility_scores.csv"):
        data = np.column_stack([points, unmachinableMask.astype(int)])
        header = "x,y,z,unmachining"
        np.savetxt(filePath, data, delimiter=",", header=header, comments="", fmt="%.6f")
        print(f"Data exported to {filePath}")

    def analyze(
            self,
            numSamples: int = 10000,
            raysPerPoint: int = 180,
            batchSize: int = 100,
            normalOffset: float = 1e-6,
            exportCsv: bool = False
    ) -> Dict:
        points, faceIndices = trimesh.sample.sample_surface(self.mesh, numSamples)
        normals = self.mesh.face_normals[faceIndices]
        unmachinableMask = np.zeros(numSamples, dtype=bool)

        for i in range(numSamples):
            point = points[i]
            normal = normals[i]

            if abs(point[2] - self.zMin) < 1.0:
                unmachinableMask[i] = True
                continue

            rayDirections = self._generateHemisphereDirections(normal, raysPerPoint)
            numValidRays = len(rayDirections)

            if numValidRays == 0:
                unmachinableMask[i] = True
                continue

            rayOrigins = np.tile(point + normal * normalOffset, (numValidRays, 1))
            hitCount = 0

            for batchStart in range(0, numValidRays, batchSize):
                batchEnd = min(batchStart + batchSize, numValidRays)
                batchOrigins = rayOrigins[batchStart:batchEnd]
                batchDirections = rayDirections[batchStart:batchEnd]

                meshHits = self.mesh.ray.intersects_any(batchOrigins, batchDirections)
                planeHits = self._checkPlaneIntersection(batchOrigins, batchDirections)

                totalHits = np.logical_or(meshHits, planeHits)
                hitCount += np.sum(totalHits)

            if hitCount >= numValidRays:
                unmachinableMask[i] = True

        unmachinablePoints = points[unmachinableMask]
        unmachinablePercent = (len(unmachinablePoints) / len(points)) * 100

        if exportCsv:
            self._exportData(points, unmachinableMask)

        return {
            "mesh": self.mesh,
            "totalPoints": len(points),
            "unmachinablePointsCount": len(unmachinablePoints),
            "unmachinablePercent": unmachinablePercent,
            "points": points,
            "unmachinablePoints": unmachinablePoints,
            "unmachinableMask": unmachinableMask
        }


class MoldVisualizer:
    def __init__(self, windowSize: Tuple[int, int] = (1200, 800)):
        self.windowSize = windowSize

    def _createVTKMeshActor(self, mesh: trimesh.Trimesh, opacity: float = 0.3) -> vtk.vtkActor:
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
        normals.ComputeCellNormalsOn()
        normals.Update()

        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(normals.GetOutputPort())

        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(0.8, 0.8, 0.8)
        actor.GetProperty().SetOpacity(opacity)
        actor.GetProperty().SetEdgeVisibility(True)
        actor.GetProperty().SetEdgeColor(0.2, 0.2, 0.2)
        actor.GetProperty().SetLineWidth(1.0)
        return actor

    def _createSpheresActor(self, points: np.ndarray, radius: float = 1.0,
                            color: Tuple[float, float, float] = (1.0, 0.0, 0.0),
                            opacity: float = 0.7) -> Optional[vtk.vtkActor]:
        if points is None or len(points) == 0:
            return None

        if len(points) > 2000:
            step = len(points) // 2000
            points = points[::step]

        sphereSource = vtk.vtkSphereSource()
        sphereSource.SetRadius(radius)
        sphereSource.SetPhiResolution(8)
        sphereSource.SetThetaResolution(8)

        vtkPoints = vtk.vtkPoints()
        for pt in points:
            vtkPoints.InsertNextPoint(pt)

        polyData = vtk.vtkPolyData()
        polyData.SetPoints(vtkPoints)

        glyph = vtk.vtkGlyph3D()
        glyph.SetSourceConnection(sphereSource.GetOutputPort())
        glyph.SetInputData(polyData)
        glyph.SetScaleModeToDataScalingOff()
        glyph.Update()

        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(glyph.GetOutputPort())

        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(color)
        actor.GetProperty().SetOpacity(opacity)
        return actor

    def _createTextActor(self, text: str, position: Tuple[int, int],
                         color: Tuple[float, float, float] = (0, 0, 0)) -> vtk.vtkTextActor:
        textActor = vtk.vtkTextActor()
        textActor.SetInput(text)
        textActor.GetTextProperty().SetColor(color)
        textActor.GetTextProperty().SetFontSize(14)
        textActor.SetPosition(position[0], position[1])
        return textActor

    def show(self, resultData: Dict) -> None:
        print("Creating VTK visualization...")
        renderer = vtk.vtkRenderer()
        renderer.SetBackground(1.0, 1.0, 1.0)

        meshActor = self._createVTKMeshActor(resultData['mesh'], opacity=0.3)
        renderer.AddActor(meshActor)

        unmachinableActor = self._createSpheresActor(
            resultData['unmachinablePoints'],
            radius=0.6,
            color=(1.0, 0.0, 0.0),
            opacity=0.8
        )
        if unmachinableActor:
            renderer.AddActor(unmachinableActor)

        statsText = f"Total Points: {resultData['totalPoints']:,}\n"
        statsText += f"Unmachining Points: {resultData['unmachinablePointsCount']:,} ({resultData['unmachinablePercent']:.1f}%)\n"
        statsText += "(Red spheres = Inaccessible Regions)"

        statsActor = self._createTextActor(statsText, (10, 10))
        renderer.AddViewProp(statsActor)

        renderWindow = vtk.vtkRenderWindow()
        renderWindow.AddRenderer(renderer)
        renderWindow.SetSize(self.windowSize[0], self.windowSize[1])
        renderWindow.SetWindowName("5-Axis CNC Mold Accessibility Analysis")

        interactor = vtk.vtkRenderWindowInteractor()
        interactor.SetRenderWindow(renderWindow)

        axes = vtk.vtkAxesActor()
        axesWidget = vtk.vtkOrientationMarkerWidget()
        axesWidget.SetOrientationMarker(axes)
        axesWidget.SetInteractor(interactor)
        axesWidget.SetViewport(0.0, 0.0, 0.2, 0.2)
        axesWidget.SetEnabled(1)
        axesWidget.InteractiveOff()

        renderer.ResetCamera()
        renderer.GetActiveCamera().Azimuth(30)
        renderer.GetActiveCamera().Elevation(30)
        renderer.GetActiveCamera().Dolly(1.2)
        renderer.ResetCameraClippingRange()

        interactor.Initialize()
        renderWindow.Render()
        interactor.Start()


def analyzeMoldAccessibility(
        meshPath: str,
        numSamples: int = 10000,
        raysPerPoint: int = 64,
        exportCsv: bool = True,
        visualize: bool = True
) -> Dict:
    analyzer = AccessibilityAnalyzer(meshPath)
    results = analyzer.analyze(
        numSamples=numSamples,
        raysPerPoint=raysPerPoint,
        exportCsv=exportCsv
    )

    print(f"Total sampled points:       {results['totalPoints']}")
    print(f"Unmachining points:         {results['unmachinablePointsCount']}")
    print(f"Unmachining percentage:     {results['unmachinablePercent']:.2f}%")

    if visualize:
        visualizer = MoldVisualizer()
        visualizer.show(results)
    return results


def main():
    stlFilePath = "testModels/hollow.cylinder.down.stl"

    results = analyzeMoldAccessibility(
        meshPath    = stlFilePath,
        numSamples  = 5000,
        raysPerPoint= 90,
        exportCsv   = False,
        visualize   = True
    )

if __name__ == "__main__":
    main()
