from typing import Dict, List, Optional
import vtk
import trimesh


class VtkInteractorStyle(vtk.vtkInteractorStyleTrackballCamera):
    def __init__(self, visualizer):
        super().__init__()
        self.visualizer = visualizer
        self.AddObserver('KeyPressEvent', self.keyPressEvent)

    def keyPressEvent(self, obj, event):
        keyVal = self.GetInteractor().GetKeySym()
        if keyVal in ['1', '2', '3']:
            stepIndex = int(keyVal) - 1
            for actorIndex, actor in enumerate(self.visualizer.stepActors):
                actor.SetVisibility(1 if actorIndex == stepIndex else 0)
        elif keyVal == '0':
            for actor in self.visualizer.stepActors:
                actor.SetVisibility(1)
        elif keyVal in ['c', 'C'] and self.visualizer.collisionActor is not None:
            currentVisibility = self.visualizer.collisionActor.GetVisibility()
            self.visualizer.collisionActor.SetVisibility(0 if currentVisibility else 1)
        self.GetInteractor().GetRenderWindow().Render()


class PathVisualizer:
    def __init__(self):
        self.windowSize = (1024, 768)
        self.stepActors = []
        self.collisionActor = None

    def createVtkMeshActor(self, mesh: trimesh.Trimesh, colorTuple: tuple, opacityVal: float) -> vtk.vtkActor:
        vtkPoints = vtk.vtkPoints()
        for vertex in mesh.vertices:
            vtkPoints.InsertNextPoint(float(vertex[0]), float(vertex[1]), float(vertex[2]))
        vtkCells = vtk.vtkCellArray()
        for face in mesh.faces:
            triCell = vtk.vtkTriangle()
            triCell.GetPointIds().SetId(0, int(face[0]))
            triCell.GetPointIds().SetId(1, int(face[1]))
            triCell.GetPointIds().SetId(2, int(face[2]))
            vtkCells.InsertNextCell(triCell)
        polyData = vtk.vtkPolyData()
        polyData.SetPoints(vtkPoints)
        polyData.SetPolys(vtkCells)
        normalsFilter = vtk.vtkPolyDataNormals()
        normalsFilter.SetInputData(polyData)
        normalsFilter.ComputePointNormalsOn()
        normalsFilter.Update()
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(normalsFilter.GetOutputPort())
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(colorTuple[0], colorTuple[1], colorTuple[2])
        actor.GetProperty().SetOpacity(float(opacityVal))
        return actor

    def buildCollisionTree(self, mesh: trimesh.Trimesh) -> vtk.vtkOBBTree:
        vtkPoints = vtk.vtkPoints()
        for vertex in mesh.vertices:
            vtkPoints.InsertNextPoint(float(vertex[0]), float(vertex[1]), float(vertex[2]))
        vtkCells = vtk.vtkCellArray()
        for face in mesh.faces:
            triCell = vtk.vtkTriangle()
            triCell.GetPointIds().SetId(0, int(face[0]))
            triCell.GetPointIds().SetId(1, int(face[1]))
            triCell.GetPointIds().SetId(2, int(face[2]))
            vtkCells.InsertNextCell(triCell)
        polyData = vtk.vtkPolyData()
        polyData.SetPoints(vtkPoints)
        polyData.SetPolys(vtkCells)
        obbTree = vtk.vtkOBBTree()
        obbTree.SetDataSet(polyData)
        obbTree.BuildLocator()
        return obbTree

    def evaluateCollisions(self, points: List[List[float]], obbTree: vtk.vtkOBBTree):
        normalLines = []
        collisionLines = []
        if len(points) < 2:
            return normalLines, collisionLines
        intersectPoints = vtk.vtkPoints()
        for pointIndex in range(1, len(points)):
            pointA = points[pointIndex - 1]
            pointB = points[pointIndex]
            hitCount = obbTree.IntersectWithLine(pointA, pointB, intersectPoints, None)
            if hitCount != 0:
                collisionLines.append((pointA, pointB))
            else:
                normalLines.append((pointA, pointB))
        return normalLines, collisionLines

    def createLinesActor(self, lines: List[tuple], colorTuple: tuple, lineWidth: float) -> vtk.vtkActor:
        vtkPoints = vtk.vtkPoints()
        vtkLines = vtk.vtkCellArray()
        pointIndex = 0
        for pointA, pointB in lines:
            vtkPoints.InsertNextPoint(float(pointA[0]), float(pointA[1]), float(pointA[2]))
            vtkPoints.InsertNextPoint(float(pointB[0]), float(pointB[1]), float(pointB[2]))
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
        actor.GetProperty().SetColor(colorTuple[0], colorTuple[1], colorTuple[2])
        actor.GetProperty().SetLineWidth(float(lineWidth))
        return actor

    def visualize(self, targetMesh: trimesh.Trimesh, clData: Dict[str, any], stepCollisionMeshes: Optional[List[trimesh.Trimesh]] = None) -> None:
        renderer = vtk.vtkRenderer()
        renderer.SetBackground(1.0, 1.0, 1.0)
        renderer.AddActor(self.createVtkMeshActor(targetMesh, (0.7, 0.7, 0.7), 0.5))
        defaultTree = self.buildCollisionTree(targetMesh)
        stepTrees = []
        if stepCollisionMeshes is not None:
            for collisionMesh in stepCollisionMeshes:
                if collisionMesh is None or collisionMesh.is_empty:
                    stepTrees.append(defaultTree)
                else:
                    stepTrees.append(self.buildCollisionTree(collisionMesh))
        colors = [(1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)]
        allCollisionLines = []
        for stepIndex, stepData in enumerate(clData.get('steps', [])):
            stepPoints = [pointObj['position'] for pointObj in stepData.get('clPoints', [])]
            collisionTree = defaultTree
            if stepTrees and stepIndex < len(stepTrees):
                collisionTree = stepTrees[stepIndex]
            normalLines, collisionLines = self.evaluateCollisions(stepPoints, collisionTree)
            actor = self.createLinesActor(normalLines, colors[stepIndex % len(colors)], 2.0) if normalLines else None
            if actor is not None:
                self.stepActors.append(actor)
                renderer.AddActor(actor)
            else:
                emptyActor = vtk.vtkActor()
                emptyActor.SetVisibility(0)
                self.stepActors.append(emptyActor)
            allCollisionLines.extend(collisionLines)
        if allCollisionLines:
            self.collisionActor = self.createLinesActor(allCollisionLines, (1.0, 0.0, 1.0), 4.0)
            self.collisionActor.SetVisibility(0)
            renderer.AddActor(self.collisionActor)
        renderWindow = vtk.vtkRenderWindow()
        renderWindow.AddRenderer(renderer)
        renderWindow.SetSize(self.windowSize[0], self.windowSize[1])
        renderWindow.SetWindowName('CNC Toolpath Visualization (1-3: Steps, 0: All, C: Collision)')
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
