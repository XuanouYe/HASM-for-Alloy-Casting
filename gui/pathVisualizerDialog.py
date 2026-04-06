import trimesh
import pyvista as pv
import vtk
import numpy as np
from pyvistaqt import QtInteractor
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QDialog, QVBoxLayout, QHBoxLayout, QPushButton, QCheckBox


stepTypeLabels = {
    "shellRemoval": "Step1 模壳去除",
    "riserRemoval": "Step2 冒口去除",
    "partFinishing": "Step3 零件精加工",
    "gateRemoval": "Step4 浇口去除"
}


class PathVisualizationDialog(QDialog):
    def __init__(self, targetMesh: trimesh.Trimesh, clData: dict, parent=None):
        super().__init__(parent)
        self.setAttribute(Qt.WA_DeleteOnClose)
        self.setWindowTitle("CNC 刀轨可视化")
        self.resize(1024, 768)
        self.targetMesh = targetMesh
        self.clData = clData
        self.stepActors = {}
        self.collisionActor = None

        self.initUI()
        self.renderPaths()

    def closeEvent(self, event):
        self.plotter.close()
        super().closeEvent(event)

    def initUI(self):
        mainLayout = QVBoxLayout(self)

        self.plotter = QtInteractor(self)
        self.plotter.set_background("white")
        self.plotter.add_axes(line_width=3, color="black")
        mainLayout.addWidget(self.plotter.interactor, 1)

        controlLayout = QHBoxLayout()

        stepCount = len(self.clData.get("steps", []))
        self.showAllButton = QPushButton(f"显示全部({stepCount})")
        self.showAllButton.clicked.connect(self.showAllSteps)
        controlLayout.addWidget(self.showAllButton)

        self.stepButtons = []
        steps = self.clData.get("steps", [])
        for stepIndex, stepData in enumerate(steps):
            stepType = stepData.get("stepType", "")
            stepId = stepData.get("stepId", stepIndex + 1)
            buttonText = stepTypeLabels.get(stepType, f"步骤 {stepId}")
            button = QPushButton(buttonText)
            button.clicked.connect(lambda checked, idx=stepIndex: self.showStepOnly(idx))
            controlLayout.addWidget(button)
            self.stepButtons.append(button)

        self.collisionCheck = QCheckBox("显示碰撞路径(C)")
        self.collisionCheck.setChecked(False)
        self.collisionCheck.toggled.connect(self.toggleCollision)
        controlLayout.addWidget(self.collisionCheck)

        controlLayout.addStretch()
        mainLayout.addLayout(controlLayout)

    def renderPaths(self):
        try:
            from geometryAdapters import trimeshToPyVista
            pvMesh = trimeshToPyVista(self.targetMesh)
        except ImportError:
            vertices = self.targetMesh.vertices
            faces = self.targetMesh.faces
            pvFaces = np.column_stack((np.full(len(faces), 3), faces)).flatten()
            pvMesh = pv.PolyData(vertices, pvFaces)

        self.plotter.add_mesh(
            pvMesh,
            color="lightgrey",
            opacity=0.5,
            show_edges=True,
            edge_color="darkgrey",
            name="target_mesh"
        )

        tree = vtk.vtkOBBTree()
        tree.SetDataSet(pvMesh)
        tree.BuildLocator()
        intersectPoints = vtk.vtkPoints()

        colors = ["red", "green", "blue", "orange", "purple", "cyan"]
        allCollisionLines = []

        for stepIndex, stepData in enumerate(self.clData.get("steps", [])):
            clPoints = stepData.get("clPoints", [])
            if not clPoints:
                continue

            segments = {}
            for pointData in clPoints:
                segmentId = pointData.get("segmentId", 0)
                segments.setdefault(segmentId, []).append(pointData["position"])

            normalLines = []

            for _, points in segments.items():
                if len(points) < 2:
                    continue

                for pointIndex in range(len(points) - 1):
                    point1 = points[pointIndex]
                    point2 = points[pointIndex + 1]

                    distance = np.linalg.norm(np.array(point1) - np.array(point2))
                    if distance < 1e-6:
                        continue

                    code = tree.IntersectWithLine(point1, point2, intersectPoints, None)
                    if code != 0:
                        allCollisionLines.extend([point1, point2])
                    else:
                        normalLines.extend([point1, point2])

            if normalLines:
                pointCount = len(normalLines)
                pointArray = np.array(normalLines)
                lineArray = np.empty((pointCount // 2, 3), dtype=int)
                lineArray[:, 0] = 2
                lineArray[:, 1] = np.arange(0, pointCount, 2)
                lineArray[:, 2] = np.arange(1, pointCount, 2)

                pvLines = pv.PolyData()
                pvLines.points = pointArray
                pvLines.lines = lineArray.flatten()

                color = colors[stepIndex % len(colors)]
                actor = self.plotter.add_mesh(pvLines, color=color, line_width=2.0, name=f"step_{stepIndex}")
                self.stepActors[stepIndex] = actor

        if allCollisionLines:
            pointCount = len(allCollisionLines)
            pointArray = np.array(allCollisionLines)
            lineArray = np.empty((pointCount // 2, 3), dtype=int)
            lineArray[:, 0] = 2
            lineArray[:, 1] = np.arange(0, pointCount, 2)
            lineArray[:, 2] = np.arange(1, pointCount, 2)

            pvCollisionLines = pv.PolyData()
            pvCollisionLines.points = pointArray
            pvCollisionLines.lines = lineArray.flatten()

            self.collisionActor = self.plotter.add_mesh(
                pvCollisionLines,
                color="red",
                line_width=1.0,
                opacity=0.3,
                name="collision_lines"
            )
            self.collisionActor.SetVisibility(False)

        self.plotter.reset_camera()

    def showAllSteps(self):
        for actor in self.stepActors.values():
            actor.SetVisibility(True)
        self.plotter.render()

    def showStepOnly(self, stepIndex):
        for currentIndex, actor in self.stepActors.items():
            actor.SetVisibility(currentIndex == stepIndex)
        self.plotter.render()

    def toggleCollision(self, checked):
        if self.collisionActor:
            self.collisionActor.SetVisibility(checked)
            self.plotter.render()