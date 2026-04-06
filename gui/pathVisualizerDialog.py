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

CUT_LINE_WIDTH = 2.5
LINK_LINE_WIDTH = 0.8


class PathVisualizationDialog(QDialog):
    def __init__(self, targetMesh: trimesh.Trimesh, clData: dict,
                 kinematicsConfig: dict = None, parent=None):
        super().__init__(parent)
        self.setAttribute(Qt.WA_DeleteOnClose)
        self.setWindowTitle("CNC 刀轨可视化")
        self.resize(1200, 800)
        self.targetMesh = targetMesh
        self.clData = clData
        self.stepActors = {}
        self.linkActors = {}
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
        for stepIndex, stepData in enumerate(self.clData.get("steps", [])):
            stepType = stepData.get("stepType", "")
            stepId = stepData.get("stepId", stepIndex + 1)
            buttonText = stepTypeLabels.get(stepType, f"步骤 {stepId}")
            button = QPushButton(buttonText)
            button.clicked.connect(lambda checked, idx=stepIndex: self.showStepOnly(idx))
            controlLayout.addWidget(button)
            self.stepButtons.append(button)

        self.showLinkCheck = QCheckBox("显示抬刀/连接")
        self.showLinkCheck.setChecked(True)
        self.showLinkCheck.toggled.connect(self.toggleLinkVisibility)
        controlLayout.addWidget(self.showLinkCheck)

        self.collisionCheck = QCheckBox("显示碰撞路径")
        self.collisionCheck.setChecked(False)
        self.collisionCheck.toggled.connect(self.toggleCollision)
        controlLayout.addWidget(self.collisionCheck)

        controlLayout.addStretch()
        mainLayout.addLayout(controlLayout)

    def buildMeshPolyData(self) -> pv.PolyData:
        try:
            from geometryAdapters import trimeshToPyVista
            return trimeshToPyVista(self.targetMesh)
        except ImportError:
            vertices = self.targetMesh.vertices
            faces = self.targetMesh.faces
            pvFaces = np.column_stack((np.full(len(faces), 3), faces)).flatten()
            return pv.PolyData(vertices, pvFaces)

    def buildPolyLines(self, pairsList):
        if not pairsList:
            return None
        pts = np.array(pairsList, dtype=float)
        lineCount = len(pairsList) // 2
        lines = np.empty((lineCount, 3), dtype=int)
        lines[:, 0] = 2
        lines[:, 1] = np.arange(0, len(pairsList), 2)
        lines[:, 2] = np.arange(1, len(pairsList), 2)
        mesh = pv.PolyData()
        mesh.points = pts
        mesh.lines = lines.flatten()
        return mesh

    def renderPaths(self):
        pvMesh = self.buildMeshPolyData()
        self.plotter.add_mesh(
            pvMesh, color="lightgrey", opacity=0.5,
            show_edges=True, edge_color="darkgrey", name="target_mesh")

        tree = vtk.vtkOBBTree()
        tree.SetDataSet(pvMesh)
        tree.BuildLocator()
        intersectPoints = vtk.vtkPoints()

        stepColors = ["red", "green", "blue", "orange", "purple", "cyan"]
        allCollisionLines = []

        for stepIndex, stepData in enumerate(self.clData.get("steps", [])):
            clPoints = stepData.get("clPoints", [])
            if not clPoints:
                continue

            stepColor = stepColors[stepIndex % len(stepColors)]
            sortedPts = sorted(clPoints, key=lambda p: int(p.get("pointId", 0)))

            cutPairs = []
            linkPairs = []

            for ptIdx in range(len(sortedPts) - 1):
                pt0 = sortedPts[ptIdx]
                pt1 = sortedPts[ptIdx + 1]
                motionType = str(pt0.get("motionType", "cut")).lower()
                pos0 = list(pt0["position"])
                pos1 = list(pt1["position"])
                if motionType == "cut":
                    if float(np.linalg.norm(np.array(pos0) - np.array(pos1))) < 1e-6:
                        continue
                    code = tree.IntersectWithLine(pos0, pos1, intersectPoints, None)
                    if code != 0:
                        allCollisionLines.extend([pos0, pos1])
                    else:
                        cutPairs.extend([pos0, pos1])
                else:
                    linkPairs.extend([pos0, pos1])

            cutMesh = self.buildPolyLines(cutPairs)
            if cutMesh is not None:
                actor = self.plotter.add_mesh(
                    cutMesh, color=stepColor, line_width=CUT_LINE_WIDTH,
                    name=f"cut_{stepIndex}")
                self.stepActors[stepIndex] = actor

            linkMesh = self.buildPolyLines(linkPairs)
            if linkMesh is not None:
                linkActor = self.plotter.add_mesh(
                    linkMesh, color=stepColor, line_width=LINK_LINE_WIDTH,
                    opacity=0.6, name=f"link_{stepIndex}")
                self.linkActors[stepIndex] = linkActor

        if allCollisionLines:
            colMesh = self.buildPolyLines(allCollisionLines)
            if colMesh is not None:
                self.collisionActor = self.plotter.add_mesh(
                    colMesh, color="red", line_width=1.0,
                    opacity=0.3, name="collision_lines")
                self.collisionActor.SetVisibility(False)

        self.plotter.reset_camera()

    def showAllSteps(self):
        for actor in self.stepActors.values():
            actor.SetVisibility(True)
        for idx, actor in self.linkActors.items():
            actor.SetVisibility(self.showLinkCheck.isChecked())
        self.plotter.render()

    def showStepOnly(self, stepIndex):
        for idx, actor in self.stepActors.items():
            actor.SetVisibility(idx == stepIndex)
        for idx, actor in self.linkActors.items():
            actor.SetVisibility(idx == stepIndex and self.showLinkCheck.isChecked())
        self.plotter.render()

    def toggleLinkVisibility(self, checked):
        for actor in self.linkActors.values():
            actor.SetVisibility(checked)
        self.plotter.render()

    def toggleCollision(self, checked):
        if self.collisionActor:
            self.collisionActor.SetVisibility(checked)
            self.plotter.render()
