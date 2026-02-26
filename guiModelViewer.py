import pyvista as pv
from pyvistaqt import QtInteractor
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import QWidget, QLabel, QHBoxLayout, QVBoxLayout


class ModelViewerWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.castingPlotter = None
        self.moldPlotter = None
        self.syncEnabled = True
        self.initUI()

    def initUI(self):
        mainLayout = QHBoxLayout()
        leftContainer = self.createViewerContainer("零件预览", "casting")
        mainLayout.addWidget(leftContainer, 1)
        rightContainer = self.createViewerContainer("模具预览", "mold")
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
        layout.addWidget(titleLabel)
        try:
            plotter = QtInteractor(container)
            plotter.set_background("white")
            plotter.add_axes()
            layout.addWidget(plotter.interactor)
            if viewType == "casting":
                self.castingPlotter = plotter
            else:
                self.moldPlotter = plotter
        except Exception as e:
            layout.addWidget(QLabel(f"PyVista 初始化失败: {e}"))
        container.setLayout(layout)
        return container

    def setupCameraSync(self):
        if not self.castingPlotter or not self.moldPlotter: return
        self.moldPlotter.renderer.camera = self.castingPlotter.camera
        def syncRenderMold(*args):
            if self.syncEnabled: self.moldPlotter.render()
        def syncRenderCasting(*args):
            if self.syncEnabled: self.castingPlotter.render()
        self.castingPlotter.iren.add_observer("InteractionEvent", syncRenderMold)
        self.moldPlotter.iren.add_observer("InteractionEvent", syncRenderCasting)

    def loadFromJobContext(self, jobContext):
        partPath = jobContext.getArtifactPath("partMesh")
        moldPath = jobContext.getArtifactPath("moldShell")

        if partPath and self.castingPlotter:
            mesh = pv.read(partPath)
            self.castingPlotter.clear()
            self.castingPlotter.add_mesh(mesh, color="lightblue", show_edges=True)
            self.castingPlotter.reset_camera()

        if moldPath and self.moldPlotter:
            mesh = pv.read(moldPath)
            self.moldPlotter.clear()
            self.moldPlotter.add_mesh(mesh, color="lightcoral", opacity=0.8, show_edges=True)
            self.moldPlotter.render()

    def resetView(self):
        if self.castingPlotter: self.castingPlotter.reset_camera()
        if self.moldPlotter: self.moldPlotter.render()
