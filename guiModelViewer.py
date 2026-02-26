from typing import Optional, Dict
import numpy as np
import trimesh
from pyvistaqt import QtInteractor
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import QWidget, QLabel, QHBoxLayout, QVBoxLayout
from geometryAdapters import trimeshToPyVista


class ModelViewerWidget(QWidget):
    modelLoaded = pyqtSignal(dict)
    moldLoaded = pyqtSignal(dict)

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
            print(f"相机同步设置警告: {str(e)}")

    def loadCastingModel(self, filePath: str) -> bool:
        from geometryAdapters import loadMeshFromFile
        self.castingTriMesh = loadMeshFromFile(filePath)
        if self.castingPlotter is None:
            return False
        pvMesh = trimeshToPyVista(self.castingTriMesh)
        self.castingPlotter.clear()
        self.castingPlotter.add_mesh(
            pvMesh,
            color="lightblue",
            opacity=1.0,
            show_edges=True,
            edge_color="navy",
            line_width=2.0,
            lighting=True,
            smooth_shading=True,
            specular=0.5,
            specular_power=15,
            ambient=0.3,
            diffuse=0.8,
            metallic=0.2,
            roughness=0.5
        )
        self.castingPlotter.enable_shadows()
        self.castingPlotter.enable_anti_aliasing()
        self.castingPlotter.reset_camera()
        self.castingPlotter.camera.zoom(1.3)
        modelInfo = {
            "filePath": filePath,
            "vertices": len(self.castingTriMesh.vertices),
            "faces": len(self.castingTriMesh.faces),
            "bounds": self.castingTriMesh.bounds.tolist(),
            "volume": float(self.castingTriMesh.volume),
            "area": float(self.castingTriMesh.area)
        }
        self.modelLoaded.emit(modelInfo)
        return True

    def loadMoldModel(self, moldMesh: trimesh.Trimesh) -> bool:
        self.moldTriMesh = moldMesh
        if self.moldPlotter is None:
            return False
        pvMesh = trimeshToPyVista(moldMesh)
        self.moldPlotter.clear()
        self.moldPlotter.add_mesh(
            pvMesh,
            color="lightcoral",
            opacity=0.9,
            show_edges=True,
            edge_color="darkred",
            line_width=1.5,
            lighting=True,
            smooth_shading=True
        )
        self.moldPlotter.render()
        moldInfo = {
            "vertices": len(moldMesh.vertices),
            "faces": len(moldMesh.faces),
            "bounds": moldMesh.bounds.tolist()
        }
        self.moldLoaded.emit(moldInfo)
        return True

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
