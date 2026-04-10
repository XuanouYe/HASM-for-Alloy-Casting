from typing import Optional, Dict
import trimesh
from pyvistaqt import QtInteractor
from PyQt5.QtCore import Qt, pyqtSignal, pyqtSlot
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import QWidget, QLabel, QHBoxLayout, QVBoxLayout
from geometryAdapters import trimeshToPyVista


class ModelViewerWidget(QWidget):
    modelLoaded = pyqtSignal(dict)
    moldLoaded = pyqtSignal(dict)
    viewError = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.castingTriMesh: Optional[trimesh.Trimesh] = None
        self.moldTriMesh: Optional[trimesh.Trimesh] = None
        self.castingPlotter = None
        self.moldPlotter = None
        self.syncEnabled = True
        self.initUI()

    def initUI(self):
        mainLayout = QHBoxLayout()
        leftContainer = self.createViewerContainer("铸件模型", "casting")
        mainLayout.addWidget(leftContainer, 1)
        rightContainer = self.createViewerContainer("模具模型", "mold")
        mainLayout.addWidget(rightContainer, 1)
        self.setLayout(mainLayout)
        self.setupCameraSync()

    def createViewerContainer(self, title: str, viewType: str) -> QWidget:
        container = QWidget()
        layout = QVBoxLayout()
        layout.setContentsMargins(5, 5, 5, 5)
        titleLabel = QLabel(title)
        titleFont = QFont()
        titleFont.setPointSize(10)
        titleFont.setBold(True)
        titleLabel.setFont(titleFont)
        titleLabel.setAlignment(Qt.AlignCenter)
        titleLabel.setStyleSheet("background-color: #f0f0f0; padding: 5px;")
        layout.addWidget(titleLabel)

        try:
            plotter = QtInteractor(container)
            plotter.set_background("white")
            plotter.add_axes(line_width=3, color="black")
            layout.addWidget(plotter.interactor)
            if viewType == "casting":
                self.castingPlotter = plotter
            else:
                self.moldPlotter = plotter
        except Exception as e:
            errorLabel = QLabel(f"PyVista初始化失败: {str(e)}")
            layout.addWidget(errorLabel)
            if viewType == "casting":
                self.castingPlotter = None
            else:
                self.moldPlotter = None

        container.setLayout(layout)
        return container

    def setupCameraSync(self):
        if not (self.castingPlotter and self.moldPlotter):
            return
        try:
            self.moldPlotter.renderer.camera = self.castingPlotter.camera

            def syncRenderMold(obj, event):
                if self.syncEnabled:
                    self.moldPlotter.render()

            def syncRenderCasting(obj, event):
                if self.syncEnabled:
                    self.castingPlotter.render()

            self.castingPlotter.iren.add_observer("InteractionEvent", syncRenderMold)
            self.moldPlotter.iren.add_observer("InteractionEvent", syncRenderCasting)
            self.castingPlotter.camera.AddObserver("ModifiedEvent", lambda o, e: self.moldPlotter.render())
        except Exception as e:
            self.viewError.emit(f"相机同步设置警告: {str(e)}")

    @pyqtSlot(object)
    def loadCastingMesh(self, mesh: trimesh.Trimesh):
        if self.castingPlotter is None or mesh is None:
            return

        self.clearMoldView()
        self.castingTriMesh = mesh.copy()
        try:
            pvMesh = trimeshToPyVista(self.castingTriMesh)

            self.castingPlotter.clear_actors()
            self.castingPlotter.add_mesh(
                pvMesh,
                name="casting_mesh_actor",
                color="lightblue",
                opacity=1.0,
                show_edges=True,
                edge_color="navy",
                line_width=2.0,
                lighting=False,
                reset_camera=True
            )

            self.castingPlotter.enable_anti_aliasing()
            self.castingPlotter.reset_camera()
            self.castingPlotter.camera.zoom(1.3)
            self.castingPlotter.update()
            modelInfo = self.getCastingModelInfo()
            if modelInfo:
                self.modelLoaded.emit(modelInfo)

        except Exception as e:
            self.viewError.emit(f"渲染铸件模型失败: {str(e)}")

    @pyqtSlot(object)
    def loadMoldMesh(self, moldMesh: trimesh.Trimesh):
        if self.moldPlotter is None or moldMesh is None:
            return

        self.moldTriMesh = moldMesh.copy()
        try:
            pvMesh = trimeshToPyVista(self.moldTriMesh)
            self.moldPlotter.clear_actors()
            self.moldPlotter.add_mesh(
                pvMesh,
                name="mold_mesh_actor",
                color="lightcoral",
                opacity=0.9,
                show_edges=True,
                edge_color="darkred",
                line_width=1.5,
                lighting=False,
                reset_camera=True
            )

            self.moldPlotter.reset_camera()
            self.moldPlotter.camera.zoom(1.3)
            self.moldPlotter.update()
            moldInfo = {
                "vertices": len(self.moldTriMesh.vertices),
                "faces": len(self.moldTriMesh.faces),
                "bounds": self.moldTriMesh.bounds.tolist()
            }
            self.moldLoaded.emit(moldInfo)

        except Exception as e:
            self.viewError.emit(f"渲染模具模型失败: {str(e)}")

    @pyqtSlot()
    def clearMoldView(self):
        if self.moldPlotter:
            self.moldPlotter.clear()
            self.moldTriMesh = None
            self.moldPlotter.render()

    @pyqtSlot()
    def resetView(self):
        if self.castingPlotter:
            self.castingPlotter.reset_camera()
        if self.moldPlotter:
            self.moldPlotter.render()

    def getCastingModelInfo(self) -> Optional[Dict]:
        if self.castingTriMesh is None:
            return None
        return {
            "vertices": len(self.castingTriMesh.vertices),
            "faces": len(self.castingTriMesh.faces),
            "bounds": self.castingTriMesh.bounds.tolist(),
            "volume": float(self.castingTriMesh.volume),
            "area": float(self.castingTriMesh.area)
        }
