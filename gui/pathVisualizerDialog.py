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

MOTION_COLORS = {
    "retract": "yellow",
    "rapid": "cyan",
    "approach": "lime",
}


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
            linkPairs: list = []
            sortedPts = sorted(clPoints, key=lambda p: int(p.get("pointId", 0)))

            for ptIdx in range(len(sortedPts) - 1):
                pt0 = sortedPts[ptIdx]
                pt1 = sortedPts[ptIdx + 1]
                motionType = str(pt0.get("motionType", "cut")).lower()
                segId = pt0.get("segmentId", 0)
                pos0 = list(pt0["position"])
                pos1 = list(pt1["position"])
                if motionType == "cut":
                    cutSegments.setdefault(segId, [])
                    if not cutSegments[segId]:
                        cutSegments[segId].append(pos0)
                    cutSegments[segId].append(pos1)
                else:
                    linkColor = MOTION_COLORS.get(motionType, "white")
                    linkPairs.append((pos0, pos1, linkColor))

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

            if normalLines:
                pointArray = np.array(normalLines, dtype=float)
                lineCount = len(normalLines) // 2
                lineArray = np.empty((lineCount, 3), dtype=int)
                lineArray[:, 0] = 2
                lineArray[:, 1] = np.arange(0, len(normalLines), 2)
                lineArray[:, 2] = np.arange(1, len(normalLines), 2)
                pvLines = pv.PolyData()
                pvLines.points = pointArray
                pvLines.lines = lineArray.flatten()
                color = stepColors[stepIndex % len(stepColors)]
                actor = self.plotter.add_mesh(pvLines, color=color,
                                              line_width=2.0, name=f"step_{stepIndex}")
                self.stepActors[stepIndex] = actor

            linkLinesByColor: dict = {}
            for pos0, pos1, linkColor in linkPairs:
                linkLinesByColor.setdefault(linkColor, []).extend([pos0, pos1])

            stepLinkActors = {}
            for linkColor, pts in linkLinesByColor.items():
                pointArray = np.array(pts, dtype=float)
                lineCount = len(pts) // 2
                lineArray = np.empty((lineCount, 3), dtype=int)
                lineArray[:, 0] = 2
                lineArray[:, 1] = np.arange(0, len(pts), 2)
                lineArray[:, 2] = np.arange(1, len(pts), 2)
                pvLink = pv.PolyData()
                pvLink.points = pointArray
                pvLink.lines = lineArray.flatten()
                linkActor = self.plotter.add_mesh(
                    pvLink, color=linkColor, line_width=1.5,
                    opacity=0.85, name=f"link_{stepIndex}_{linkColor}")
                stepLinkActors[linkColor] = linkActor
            self.linkActors[stepIndex] = stepLinkActors

        if allCollisionLines:
            pointArray = np.array(allCollisionLines, dtype=float)
            lineCount = len(allCollisionLines) // 2
            lineArray = np.empty((lineCount, 3), dtype=int)
            lineArray[:, 0] = 2
            lineArray[:, 1] = np.arange(0, len(allCollisionLines), 2)
            lineArray[:, 2] = np.arange(1, len(allCollisionLines), 2)
            pvCollisionLines = pv.PolyData()
            pvCollisionLines.points = pointArray
            pvCollisionLines.lines = lineArray.flatten()
            self.collisionActor = self.plotter.add_mesh(
                pvCollisionLines, color="red", line_width=1.0,
                opacity=0.3, name="collision_lines")
            self.collisionActor.SetVisibility(False)

        self.plotter.reset_camera()

    def showAllSteps(self):
        for actor in self.stepActors.values():
            actor.SetVisibility(True)
        for stepLinkActorMap in self.linkActors.values():
            for actor in stepLinkActorMap.values():
                actor.SetVisibility(self.showLinkCheck.isChecked())
        self.plotter.render()

    def showStepOnly(self, stepIndex):
        for idx, actor in self.stepActors.items():
            actor.SetVisibility(idx == stepIndex)
        for idx, stepLinkActorMap in self.linkActors.items():
            for actor in stepLinkActorMap.values():
                actor.SetVisibility(idx == stepIndex and self.showLinkCheck.isChecked())
        self.plotter.render()

    def toggleLinkVisibility(self, checked):
        for stepLinkActorMap in self.linkActors.values():
            for actor in stepLinkActorMap.values():
                actor.SetVisibility(checked)
        self.plotter.render()

    def toggleCollision(self, checked):
        if self.collisionActor:
            self.collisionActor.SetVisibility(checked)
            self.plotter.render()
