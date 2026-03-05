import trimesh
import pyvista as pv
import vtk
import numpy as np
from pyvistaqt import QtInteractor
from PyQt5.QtWidgets import QDialog, QVBoxLayout, QHBoxLayout, QPushButton, QCheckBox


class PathVisualizationDialog(QDialog):
    def __init__(self, targetMesh: trimesh.Trimesh, clData: dict, parent=None):
        super().__init__(parent)
        self.setWindowTitle("CNC 刀轨可视化")
        self.resize(1024, 768)
        self.targetMesh = targetMesh
        self.clData = clData
        self.stepActors = {}
        self.collisionActor = None

        self.initUI()
        self.renderPaths()

    def initUI(self):
        mainLayout = QVBoxLayout(self)

        self.plotter = QtInteractor(self)
        self.plotter.set_background("white")
        self.plotter.add_axes(line_width=3, color="black")
        mainLayout.addWidget(self.plotter.interactor, 1)

        controlLayout = QHBoxLayout()

        self.showAllButton = QPushButton("显示全部(0)")
        self.showAllButton.clicked.connect(self.showAllSteps)
        controlLayout.addWidget(self.showAllButton)

        self.stepButtons = []
        steps = self.clData.get("steps", [])
        for i, step in enumerate(steps):
            btn = QPushButton(f"步骤 {i + 1}")
            btn.clicked.connect(lambda checked, idx=i: self.showStepOnly(idx))
            controlLayout.addWidget(btn)
            self.stepButtons.append(btn)

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
            pv_faces = np.column_stack((np.full(len(faces), 3), faces)).flatten()
            pvMesh = pv.PolyData(vertices, pv_faces)

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

        for stepIndex, step in enumerate(self.clData.get("steps", [])):
            clPoints = step.get("clPoints", [])
            if not clPoints:
                continue

            segments = {}
            for p in clPoints:
                sid = p.get("segmentId", 0)
                segments.setdefault(sid, []).append(p["position"])

            normal_lines = []

            for sid, pts in segments.items():
                if len(pts) < 2:
                    continue

                for i in range(len(pts) - 1):
                    p1 = pts[i]
                    p2 = pts[i + 1]

                    dist = np.linalg.norm(np.array(p1) - np.array(p2))
                    if dist < 1e-6:
                        continue

                    code = tree.IntersectWithLine(p1, p2, intersectPoints, None)
                    if code != 0:
                        allCollisionLines.extend([p1, p2])
                    else:
                        normal_lines.extend([p1, p2])

            if normal_lines:
                n_pts = len(normal_lines)
                pts_arr = np.array(normal_lines)
                lines_arr = np.empty((n_pts // 2, 3), dtype=int)
                lines_arr[:, 0] = 2
                lines_arr[:, 1] = np.arange(0, n_pts, 2)
                lines_arr[:, 2] = np.arange(1, n_pts, 2)

                pvLines = pv.PolyData()
                pvLines.points = pts_arr
                pvLines.lines = lines_arr.flatten()

                color = colors[stepIndex % len(colors)]
                actor = self.plotter.add_mesh(pvLines, color=color, line_width=2.0, name=f"step_{stepIndex}")
                self.stepActors[stepIndex] = actor

        if allCollisionLines:
            n_pts = len(allCollisionLines)
            pts_arr = np.array(allCollisionLines)
            lines_arr = np.empty((n_pts // 2, 3), dtype=int)
            lines_arr[:, 0] = 2
            lines_arr[:, 1] = np.arange(0, n_pts, 2)
            lines_arr[:, 2] = np.arange(1, n_pts, 2)

            pvColLines = pv.PolyData()
            pvColLines.points = pts_arr
            pvColLines.lines = lines_arr.flatten()

            self.collisionActor = self.plotter.add_mesh(pvColLines, color="red", line_width=1.0, opacity=0.3,
                                                        name="collision_lines")
            self.collisionActor.SetVisibility(False)

        self.plotter.reset_camera()

    def showAllSteps(self):
        for actor in self.stepActors.values():
            actor.SetVisibility(True)
        self.plotter.render()

    def showStepOnly(self, stepIndex):
        for idx, actor in self.stepActors.items():
            actor.SetVisibility(idx == stepIndex)
        self.plotter.render()

    def toggleCollision(self, checked):
        if self.collisionActor:
            self.collisionActor.SetVisibility(checked)
            self.plotter.render()
