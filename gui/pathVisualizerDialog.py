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

CUT_LINE_WIDTH = 2.0
LINK_LINE_WIDTH = 0.8
LINK_LINE_OPACITY = 0.6

NON_CUT_TYPES = {"retract", "rapid", "approach"}


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
        showAllBtn = QPushButton(f"显示全部({stepCount})")
        showAllBtn.clicked.connect(self.showAllSteps)
        controlLayout.addWidget(showAllBtn)

        self.stepButtons = []
        for stepIndex, stepData in enumerate(self.clData.get("steps", [])):
            stepType = stepData.get("stepType", "")
            stepId = stepData.get("stepId", stepIndex + 1)
            buttonText = stepTypeLabels.get(stepType, f"步骤 {stepId}")
            btn = QPushButton(buttonText)
            btn.clicked.connect(lambda checked, idx=stepIndex: self.showStepOnly(idx))
            controlLayout.addWidget(btn)
            self.stepButtons.append(btn)

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

            cutSegments: dict = {}
            linkLines: list = []
            sortedPts = sorted(clPoints, key=lambda p: int(p.get("pointId", 0)))

            for ptIdx in range(len(sortedPts) - 1):
                pt0 = sortedPts[ptIdx]
                pt1 = sortedPts[ptIdx + 1]
                mt0 = str(pt0.get("motionType", "cut")).lower()
                mt1 = str(pt1.get("motionType", "cut")).lower()
                isLinkSegment = (mt0 in NON_CUT_TYPES) or (mt1 in NON_CUT_TYPES)
                pos0 = list(pt0["position"])
                pos1 = list(pt1["position"])
                if isLinkSegment:
                    linkLines.extend([pos0, pos1])
                else:
                    segId = pt0.get("segmentId", 0)
                    if int(segId) < 0:
                        continue
                    cutSegments.setdefault(segId, [])
                    if not cutSegments[segId]:
                        cutSegments[segId].append(pos0)
                    cutSegments[segId].append(pos1)

            normalLines = []
            for segId, segPositions in cutSegments.items():
                displayPts = np.array(segPositions, dtype=float)
                for k in range(len(displayPts) - 1):
                    p1 = displayPts[k].tolist()
                    p2 = displayPts[k + 1].tolist()
                    if float(np.linalg.norm(np.array(p1) - np.array(p2))) < 1e-6:
                        continue
                    code = tree.IntersectWithLine(p1, p2, intersectPoints, None)
                    if code != 0:
                        allCollisionLines.extend([p1, p2])
                    else:
                        normalLines.extend([p1, p2])

            stepColor = stepColors[stepIndex % len(stepColors)]

            if normalLines:
                ptArr = np.array(normalLines, dtype=float)
                lineCount = len(normalLines) // 2
                lineArr = np.empty((lineCount, 3), dtype=int)
                lineArr[:, 0] = 2
                lineArr[:, 1] = np.arange(0, len(normalLines), 2)
                lineArr[:, 2] = np.arange(1, len(normalLines), 2)
                pvCut = pv.PolyData()
                pvCut.points = ptArr
                pvCut.lines = lineArr.flatten()
                actor = self.plotter.add_mesh(
                    pvCut, color=stepColor, line_width=CUT_LINE_WIDTH,
                    name=f"step_{stepIndex}")
                self.stepActors[stepIndex] = actor

            if linkLines:
                ptArr = np.array(linkLines, dtype=float)
                lineCount = len(linkLines) // 2
                lineArr = np.empty((lineCount, 3), dtype=int)
                lineArr[:, 0] = 2
                lineArr[:, 1] = np.arange(0, len(linkLines), 2)
                lineArr[:, 2] = np.arange(1, len(linkLines), 2)
                pvLink = pv.PolyData()
                pvLink.points = ptArr
                pvLink.lines = lineArr.flatten()
                linkActor = self.plotter.add_mesh(
                    pvLink, color=stepColor,
                    line_width=LINK_LINE_WIDTH,
                    opacity=LINK_LINE_OPACITY,
                    name=f"link_{stepIndex}")
                self.linkActors[stepIndex] = linkActor

        if allCollisionLines:
            ptArr = np.array(allCollisionLines, dtype=float)
            lineCount = len(allCollisionLines) // 2
            lineArr = np.empty((lineCount, 3), dtype=int)
            lineArr[:, 0] = 2
            lineArr[:, 1] = np.arange(0, len(allCollisionLines), 2)
            lineArr[:, 2] = np.arange(1, len(allCollisionLines), 2)
            pvColl = pv.PolyData()
            pvColl.points = ptArr
            pvColl.lines = lineArr.flatten()
            self.collisionActor = self.plotter.add_mesh(
                pvColl, color="red", line_width=1.0,
                opacity=0.3, name="collision_lines")
            self.collisionActor.SetVisibility(False)

        self.plotter.reset_camera()

    def showAllSteps(self):
        for actor in self.stepActors.values():
            actor.SetVisibility(True)
        showLinks = self.showLinkCheck.isChecked()
        for actor in self.linkActors.values():
            actor.SetVisibility(showLinks)
        self.plotter.render()

    def showStepOnly(self, stepIndex):
        for idx, actor in self.stepActors.items():
            actor.SetVisibility(idx == stepIndex)
        showLinks = self.showLinkCheck.isChecked()
        for idx, actor in self.linkActors.items():
            actor.SetVisibility(idx == stepIndex and showLinks)
        self.plotter.render()

    def toggleLinkVisibility(self, checked):
        for actor in self.linkActors.values():
            actor.SetVisibility(checked)
        self.plotter.render()

    def toggleCollision(self, checked):
        if self.collisionActor:
            self.collisionActor.SetVisibility(checked)
            self.plotter.render()