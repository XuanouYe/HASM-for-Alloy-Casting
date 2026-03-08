from typing import Dict, List, Optional, Set, Tuple
import numpy as np
import trimesh
import vtk

class VtkInteractorStyle(vtk.vtkInteractorStyleTrackballCamera):
    def __init__(self, visualizer):
        self.AddObserver('KeyPressEvent', self.keyPressEvent)
        self.visualizer = visualizer

    def keyPressEvent(self, obj, event):
        keyValue = self.GetInteractor().GetKeySym()
        if keyValue.isdigit():
            self.visualizer.setVisibleStep(int(keyValue))
            self.GetInteractor().GetRenderWindow().Render()
        elif keyValue in ['c', 'C']:
            self.visualizer.toggleCollisionVisibility()
            self.GetInteractor().GetRenderWindow().Render()

class PathVisualizer:
    def __init__(self):
        self.windowSize = (1200, 800)
        self.stepActors = []
        self.stepCollisionActors = []
        self.currentVisibleStep = 0
        self.collisionVisible = False
        self.legendActor = None

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

    def isSegmentColliding(self, pointA: List[float], pointB: List[float], distanceFunc: vtk.vtkImplicitPolyDataDistance, sampleStep: float = 0.4) -> bool:
        pointAArray = np.asarray(pointA, dtype=float)
        pointBArray = np.asarray(pointB, dtype=float)
        segmentLength = float(np.linalg.norm(pointBArray - pointAArray))
        sampleCount = max(int(np.ceil(segmentLength / max(sampleStep, 1e-6))), 1)
        for sampleIndex in range(sampleCount + 1):
            ratioValue = sampleIndex / float(sampleCount)
            samplePoint = pointAArray * (1.0 - ratioValue) + pointBArray * ratioValue
            distanceValue = float(distanceFunc.FunctionValue(samplePoint.tolist()))
            if distanceValue <= 0.0:
                return True
        return False

    def evaluatePointCollisions(self, points: List[List[float]], distanceFunc: vtk.vtkImplicitPolyDataDistance, sampleStep: float = 0.4):
        normalLines = []
        collisionLines = []
        if not points:
            return normalLines, collisionLines
        for pointIndex in range(1, len(points)):
            pointA = points[pointIndex - 1]
            pointB = points[pointIndex]
            if self.isSegmentColliding(pointA, pointB, distanceFunc, sampleStep):
                collisionLines.append((pointA, pointB))
            else:
                normalLines.append((pointA, pointB))
        return normalLines, collisionLines

    def createLinesActor(self, lines: List[Tuple[List[float], List[float]]], colorValue: Tuple[float, float, float], lineWidth: float, opacityValue: float = 1.0) -> vtk.vtkActor:
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
        actor.GetProperty().SetOpacity(opacityValue)
        return actor

    def createLegendActor(self, clData: Dict) -> vtk.vtkTextActor:
        legendLines = ['0: show all', '1-9: show step', 'C: toggle collision']
        for stepIndex, stepItem in enumerate(clData.get('steps', []), start=1):
            legendLines.append(f'{stepIndex}: {stepItem.get("stepType", "unknown")}')
        textActor = vtk.vtkTextActor()
        textActor.SetInput('\n'.join(legendLines))
        textActor.SetDisplayPosition(20, 20)
        textProperty = textActor.GetTextProperty()
        textProperty.SetFontSize(18)
        textProperty.SetColor(0.1, 0.1, 0.1)
        textProperty.SetBackgroundColor(1.0, 1.0, 1.0)
        textProperty.SetBackgroundOpacity(0.7)
        return textActor

    def updateActorsVisibility(self) -> None:
        showAll = self.currentVisibleStep == 0
        for actorIndex, actorItem in enumerate(self.stepActors):
            actorItem.SetVisibility(1 if showAll or actorIndex == self.currentVisibleStep - 1 else 0)
        for actorIndex, actorItem in enumerate(self.stepCollisionActors):
            if actorItem is None:
                continue
            shouldShow = self.collisionVisible and (showAll or actorIndex == self.currentVisibleStep - 1)
            actorItem.SetVisibility(1 if shouldShow else 0)

    def setVisibleStep(self, stepValue: int) -> None:
        self.currentVisibleStep = max(int(stepValue), 0)
        self.updateActorsVisibility()

    def toggleCollisionVisibility(self) -> None:
        self.collisionVisible = not self.collisionVisible
        self.updateActorsVisibility()

    def visualize(self, targetMesh: trimesh.Trimesh, clData: Dict, stepCollisionMeshes: Optional[List[trimesh.Trimesh]] = None, collisionCheckStepTypes: Optional[Set[str]] = None) -> None:
        renderer = vtk.vtkRenderer()
        renderer.SetBackground(1.0, 1.0, 1.0)
        renderer.AddActor(self.createVtkMeshActor(targetMesh, (0.7, 0.7, 0.7), 0.45))
        colors = [
            (1.0, 0.0, 0.0),
            (1.0, 0.6, 0.0),
            (0.0, 0.75, 0.0),
            (0.0, 0.2, 1.0),
            (0.6, 0.0, 0.8),
            (0.0, 0.7, 0.7)
        ]
        self.stepActors = []
        self.stepCollisionActors = []
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
            stepColor = colors[stepIndex % len(colors)]
            stepActor = self.createLinesActor(normalLines, stepColor, 2.0, 1.0)
            self.stepActors.append(stepActor)
            renderer.AddActor(stepActor)
            if collisionLines:
                collisionActor = self.createLinesActor(collisionLines, stepColor, 5.0, 1.0)
                collisionActor.SetVisibility(0)
                self.stepCollisionActors.append(collisionActor)
                renderer.AddActor(collisionActor)
            else:
                self.stepCollisionActors.append(None)
        self.legendActor = self.createLegendActor(clData)
        renderer.AddActor2D(self.legendActor)
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
        self.currentVisibleStep = 0
        self.collisionVisible = False
        self.updateActorsVisibility()
        renderer.ResetCamera()
        renderer.GetActiveCamera().Azimuth(30)
        renderer.GetActiveCamera().Elevation(30)
        interactor.Initialize()
        renderWindow.Render()
        interactor.Start()
