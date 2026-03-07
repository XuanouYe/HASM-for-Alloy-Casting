from typing import Dict, List, Optional, Set, Tuple
import trimesh
import vtk


class VtkInteractorStyle(vtk.vtkInteractorStyleTrackballCamera):
    def __init__(self, visualizer):
        self.AddObserver('KeyPressEvent', self.keyPressEvent)
        self.visualizer = visualizer

    def keyPressEvent(self, obj, event):
        keyValue = self.GetInteractor().GetKeySym()
        if keyValue.isdigit():
            stepValue = int(keyValue)
            if stepValue == 0:
                for actorItem in self.visualizer.stepActors:
                    actorItem.SetVisibility(1)
            else:
                stepIndex = stepValue - 1
                for actorIndex, actorItem in enumerate(self.visualizer.stepActors):
                    actorItem.SetVisibility(1 if actorIndex == stepIndex else 0)
        elif keyValue in ['c', 'C']:
            if self.visualizer.collisionActor is not None:
                isVisible = self.visualizer.collisionActor.GetVisibility()
                self.visualizer.collisionActor.SetVisibility(1 - isVisible)
        self.GetInteractor().GetRenderWindow().Render()


class PathVisualizer:
    def __init__(self):
        self.windowSize = (1200, 800)
        self.stepActors = []
        self.collisionActor = None

    def buildPolyData(self, mesh: trimesh.Trimesh) -> vtk.vtkPolyData:
        vtkPoints = vtk.vtkPoints()
        for vertexItem in mesh.vertices:
            vtkPoints.InsertNextPoint(vertexItem[0], vertexItem[1], vertexItem[2])
        vtkCells = vtk.vtkCellArray()
        for faceItem in mesh.faces:
            triCell = vtk.vtkTriangle()
            triCell.GetPointIds().SetId(0, int(faceItem[0]))
            triCell.GetPointIds().SetId(1, int(faceItem[1]))
            triCell.GetPointIds().SetId(2, int(faceItem[2]))
            vtkCells.InsertNextCell(triCell)
        polyData = vtk.vtkPolyData()
        polyData.SetPoints(vtkPoints)
        polyData.SetPolys(vtkCells)
        return polyData

    def createVtkMeshActor(self, mesh: trimesh.Trimesh, colorValue: Tuple[float, float, float], opacityValue: float) -> vtk.vtkActor:
        polyData = self.buildPolyData(mesh)
        normalsFilter = vtk.vtkPolyDataNormals()
        normalsFilter.SetInputData(polyData)
        normalsFilter.ComputePointNormalsOn()
        normalsFilter.Update()
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(normalsFilter.GetOutputPort())
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(colorValue[0], colorValue[1], colorValue[2])
        actor.GetProperty().SetOpacity(opacityValue)
        return actor

    def buildImplicitDistance(self, mesh: trimesh.Trimesh) -> vtk.vtkImplicitPolyDataDistance:
        polyData = self.buildPolyData(mesh)
        distanceFunc = vtk.vtkImplicitPolyDataDistance()
        distanceFunc.SetInput(polyData)
        return distanceFunc

    def evaluatePointCollisions(self, points: List[List[float]], distanceFunc: vtk.vtkImplicitPolyDataDistance):
        normalLines = []
        collisionLines = []
        if not points:
            return normalLines, collisionLines
        for pointIndex in range(1, len(points)):
            pointA = points[pointIndex - 1]
            pointB = points[pointIndex]
            distA = float(distanceFunc.FunctionValue(pointA))
            distB = float(distanceFunc.FunctionValue(pointB))
            if distA < 0.0 or distB < 0.0:
                collisionLines.append((pointA, pointB))
            else:
                normalLines.append((pointA, pointB))
        return normalLines, collisionLines

    def createLinesActor(self, lines: List[Tuple[List[float], List[float]]], colorValue: Tuple[float, float, float], lineWidth: float) -> vtk.vtkActor:
        vtkPoints = vtk.vtkPoints()
        vtkLines = vtk.vtkCellArray()
        pointIndex = 0
        for lineItem in lines:
            pointA, pointB = lineItem
            vtkPoints.InsertNextPoint(pointA[0], pointA[1], pointA[2])
            vtkPoints.InsertNextPoint(pointB[0], pointB[1], pointB[2])
            lineCell = vtk.vtkLine()
            lineCell.GetPointIds().SetId(0, pointIndex)
            lineCell.GetPointIds().SetId(1, pointIndex + 1)
            vtkLines.InsertNextCell(lineCell)
            pointIndex += 2
        polyData = vtk.vtkPolyData()
        polyData.SetPoints(vtkPoints)
        polyData.SetLines(vtkLines)
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputData(polyData)
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(colorValue[0], colorValue[1], colorValue[2])
        actor.GetProperty().SetLineWidth(lineWidth)
        return actor

    def visualize(self, targetMesh: trimesh.Trimesh, clData: Dict, stepCollisionMeshes: Optional[List[trimesh.Trimesh]] = None,
                  collisionCheckStepTypes: Optional[Set[str]] = None) -> None:
        renderer = vtk.vtkRenderer()
        renderer.SetBackground(1.0, 1.0, 1.0)
        renderer.AddActor(self.createVtkMeshActor(targetMesh, (0.7, 0.7, 0.7), 0.45))
        colors = [(1.0, 0.0, 0.0), (1.0, 0.6, 0.0), (0.0, 0.8, 0.0), (0.0, 0.2, 1.0), (0.6, 0.0, 0.8)]
        allCollisionLines = []
        self.stepActors = []
        distanceFuncs = []
        if stepCollisionMeshes is not None:
            for meshItem in stepCollisionMeshes:
                distanceFuncs.append(self.buildImplicitDistance(meshItem))
        for stepIndex, stepItem in enumerate(clData.get('steps', [])):
            pointsList = [pointItem['position'] for pointItem in stepItem.get('clPoints', [])]
            stepType = str(stepItem.get('stepType', ''))
            if collisionCheckStepTypes is not None and stepType in collisionCheckStepTypes and stepIndex < len(distanceFuncs):
                normalLines, collisionLines = self.evaluatePointCollisions(pointsList, distanceFuncs[stepIndex])
            else:
                normalLines = [(pointsList[i - 1], pointsList[i]) for i in range(1, len(pointsList))]
                collisionLines = []
            stepActor = self.createLinesActor(normalLines, colors[stepIndex % len(colors)], 2.0)
            self.stepActors.append(stepActor)
            renderer.AddActor(stepActor)
            allCollisionLines.extend(collisionLines)
        if allCollisionLines:
            self.collisionActor = self.createLinesActor(allCollisionLines, (1.0, 0.0, 1.0), 4.0)
            self.collisionActor.SetVisibility(0)
            renderer.AddActor(self.collisionActor)
        renderWindow = vtk.vtkRenderWindow()
        renderWindow.AddRenderer(renderer)
        renderWindow.SetSize(self.windowSize[0], self.windowSize[1])
        renderWindow.SetWindowName('CNC Toolpath Visualization')
        interactor = vtk.vtkRenderWindowInteractor()
        interactor.SetRenderWindow(renderWindow)
        style = VtkInteractorStyle(self)
        style.SetDefaultRenderer(renderer)
        interactor.SetInteractorStyle(style)
        axesWidget = vtk.vtkOrientationMarkerWidget()
        axesWidget.SetOrientationMarker(vtk.vtkAxesActor())
        axesWidget.SetInteractor(interactor)
        axesWidget.SetEnabled(1)
        axesWidget.InteractiveOff()
        renderer.ResetCamera()
        renderer.GetActiveCamera().Azimuth(30)
        renderer.GetActiveCamera().Elevation(30)
        interactor.Initialize()
        renderWindow.Render()
        interactor.Start()
