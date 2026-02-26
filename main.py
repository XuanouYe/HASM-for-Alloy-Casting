import sys
from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import Qt
from guiModelViewer import ModelViewerWidget
from guiMoldProcessPanel import MoldProcessPanel
from guiParameterPanel import ProcessParameterPanel
from guiMainWindow import MainWindow
from guiMainController import MainController
from guiMoldProcessController import MoldProcessController


def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    viewer = ModelViewerWidget()
    moldProcessPanel = MoldProcessPanel()
    parameterPanel = ProcessParameterPanel()
    mainWindow = MainWindow(moldProcessPanel, parameterPanel, viewer)
    moldController = MoldProcessController()
    moldProcessPanel.intentLoadModel.connect(moldController.handleLoadModel)
    moldProcessPanel.intentGenerateMold.connect(moldController.handleGenerateMold)
    moldProcessPanel.intentAddGating.connect(moldController.handleAddGating)
    moldProcessPanel.intentOptimizeOrientation.connect(moldController.handleOptimizeOrientation)
    moldProcessPanel.intentAdjustStructure.connect(moldController.handleAdjustStructure)
    moldController.modelLoaded.connect(moldProcessPanel.onModelLoadedSuccess)
    moldController.moldGenerated.connect(moldProcessPanel.onMoldGeneratedSuccess)
    moldController.gatingAdded.connect(moldProcessPanel.onGatingAddedSuccess)
    moldController.orientationOptimized.connect(moldProcessPanel.onOrientationOptimizedSuccess)
    moldController.structureAdjusted.connect(moldProcessPanel.onStructureAdjustedSuccess)
    moldController.processError.connect(moldProcessPanel.onProcessError)
    moldController.updateCastingView.connect(viewer.loadCastingMesh, type=Qt.QueuedConnection)
    moldController.updateMoldView.connect(viewer.loadMoldMesh, type=Qt.QueuedConnection)
    viewer.viewError.connect(lambda err: moldProcessPanel.onProcessError("渲染错误", err))
    mainController = MainController(mainWindow, moldController, parameterPanel)
    mainWindow.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
