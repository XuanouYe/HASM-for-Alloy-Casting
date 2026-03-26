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

    def createVtkMeshActor(self, mesh: trimesh.Trimesh, colorValue: Tuple[float, float, float],
                           opacityValue: float) -> vtk.vtkActor:
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

    def isSegmentColliding(self, pointA: List[float], pointB: List[float],
                           distanceFunc: vtk.vtkImplicitPolyDataDistance, sampleStep: float = 0.4) -> bool:
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

    def evaluatePointCollisions(self, points: List[List[float]],
                                distanceFunc: Optional[vtk.vtkImplicitPolyDataDistance], sampleStep: float = 0.4):
        normalLines = []
        collisionLines = []
        if not points:
            return normalLines, collisionLines
        for pointIndex in range(1, len(points)):
            pointA = points[pointIndex - 1]
            pointB = points[pointIndex]
            if distanceFunc is not None and self.isSegmentColliding(pointA, pointB, distanceFunc, sampleStep):
                collisionLines.append((pointA, pointB))
            else:
                normalLines.append((pointA, pointB))
        return normalLines, collisionLines

    def buildSegmentPointMap(self, stepItem: Dict) -> Dict[int, List[List[float]]]:
        segmentPointMap = {}
        sortedPoints = sorted(stepItem.get('clPoints', []), key=lambda pointItem: int(pointItem.get('pointId', 0)))
        for pointItem in sortedPoints:
            segmentId = int(pointItem.get('segmentId', -1))
            segmentPointMap.setdefault(segmentId, []).append(pointItem.get('position', [0.0, 0.0, 0.0]))
        return segmentPointMap

    def createLinesActor(self, lines: List[Tuple[List[float], List[float]]], colorValue: Tuple[float, float, float],
                         lineWidth: float, opacityValue: float = 1.0) -> vtk.vtkActor:
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

    def visualize(self, targetMesh: trimesh.Trimesh, clData: Dict,
                  stepCollisionMeshes: Optional[List[trimesh.Trimesh]] = None,
                  collisionCheckStepTypes: Optional[Set[str]] = None) -> None:
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

        for stepIndex, stepItem in enumerate(clData.get('steps', [])):
            stepColor = colors[stepIndex % len(colors)]
            stepType = str(stepItem.get('stepType', ''))

            normalLinesAll = []
            collisionLinesAll = []

            checkCollision = False
            distanceFunc = None
            if stepCollisionMeshes is not None and stepIndex < len(stepCollisionMeshes):
                stepMesh = stepCollisionMeshes[stepIndex]
                if stepMesh is not None and not stepMesh.is_empty:
                    if collisionCheckStepTypes is None or stepType in collisionCheckStepTypes:
                        checkCollision = True
                        distanceFunc = self.buildImplicitDistance(stepMesh)

            segmentPointMap = self.buildSegmentPointMap(stepItem)

            for segmentId in sorted(segmentPointMap.keys()):
                segmentPoints = segmentPointMap[segmentId]
                if checkCollision and distanceFunc is not None:
                    nLines, cLines = self.evaluatePointCollisions(segmentPoints, distanceFunc, 0.4)
                    normalLinesAll.extend(nLines)
                    collisionLinesAll.extend(cLines)
                else:
                    nLines, _ = self.evaluatePointCollisions(segmentPoints, None, 0.4)
                    normalLinesAll.extend(nLines)

            if normalLinesAll:
                normalActor = self.createLinesActor(normalLinesAll, stepColor, 2.0)
                renderer.AddActor(normalActor)
                self.stepActors.append(normalActor)
            else:
                self.stepActors.append(vtk.vtkActor())

            if collisionLinesAll:
                collisionActor = self.createLinesActor(collisionLinesAll, (1.0, 0.0, 0.0), 4.0)
                collisionActor.SetVisibility(0)
                renderer.AddActor(collisionActor)
                self.stepCollisionActors.append(collisionActor)
            else:
                self.stepCollisionActors.append(None)

        self.legendActor = self.createLegendActor(clData)
        renderer.AddActor(self.legendActor)
        self.updateActorsVisibility()

        renderWindow = vtk.vtkRenderWindow()
        renderWindow.SetSize(self.windowSize[0], self.windowSize[1])
        renderWindow.AddRenderer(renderer)
        renderWindowInteractor = vtk.vtkRenderWindowInteractor()
        renderWindowInteractor.SetRenderWindow(renderWindow)
        interactorStyle = VtkInteractorStyle(self)
        renderWindowInteractor.SetInteractorStyle(interactorStyle)
        renderWindow.Render()
        renderWindowInteractor.Start()