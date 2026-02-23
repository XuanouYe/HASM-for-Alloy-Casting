import sys
from datetime import timedelta
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QApplication, QMessageBox,
    QVBoxLayout, QSplitter, QTabWidget, QLabel, QProgressBar,
    QAction
)
from guiModelViewer import ModelViewerWidget
from guiMoldProcessPanel import MoldProcessPanel
from guiParameterPanel import ProcessParameterPanel
from controlConfig import ConfigManager


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("增减材复合制造系统控制平台")
        self.setGeometry(100, 100, 1600, 900)

        self.currentTask = None
        self.elapsedTime = timedelta(0)
        self.isProcessing = False
        self.currentCastingTrimesh = None
        self.configManager = ConfigManager()

        self.initUI()
        self.initMenuBar()
        self.initStatusBar()
        self.initConnections()

        self.timerDisplay = QTimer()
        self.timerDisplay.timeout.connect(self.updateTimer)

    def initUI(self):
        centralWidget = QWidget()
        mainLayout = QVBoxLayout()

        horizontalSplitter = QSplitter(Qt.Horizontal)

        self.dualModelViewer = ModelViewerWidget()
        horizontalSplitter.addWidget(self.dualModelViewer)

        rightPanel = QWidget()
        rightLayout = QVBoxLayout()
        rightLayout.setContentsMargins(0, 0, 0, 0)

        tabWidget = QTabWidget()
        self.moldProcessPanel = MoldProcessPanel()
        tabWidget.addTab(self.moldProcessPanel, "模具生成")
        self.parameterPanel = ProcessParameterPanel()
        tabWidget.addTab(self.parameterPanel, "工艺参数")

        rightLayout.addWidget(tabWidget)
        rightPanel.setLayout(rightLayout)
        horizontalSplitter.addWidget(rightPanel)

        horizontalSplitter.setSizes([1000, 600])
        horizontalSplitter.setCollapsible(0, False)
        horizontalSplitter.setCollapsible(1, False)

        mainLayout.addWidget(horizontalSplitter, 1)
        centralWidget.setLayout(mainLayout)
        self.setCentralWidget(centralWidget)

    def initMenuBar(self):
        menuBar = self.menuBar()

        fileMenu = menuBar.addMenu("文件(&F)")
        newAction = QAction("新建项目", self)
        newAction.setShortcut("Ctrl+N")
        newAction.triggered.connect(self.onNewProject)
        fileMenu.addAction(newAction)

        saveAction = QAction("保存项目", self)
        saveAction.setShortcut("Ctrl+S")
        saveAction.triggered.connect(self.onSaveProject)
        fileMenu.addAction(saveAction)

        fileMenu.addSeparator()

        exitAction = QAction("退出", self)
        exitAction.setShortcut("Alt+F4")
        exitAction.triggered.connect(self.close)
        fileMenu.addAction(exitAction)

        editMenu = menuBar.addMenu("编辑(&E)")
        undoAction = QAction("撤销", self)
        undoAction.setShortcut("Ctrl+Z")
        undoAction.setEnabled(False)
        editMenu.addAction(undoAction)

        redoAction = QAction("重做", self)
        redoAction.setShortcut("Ctrl+Y")
        redoAction.setEnabled(False)
        editMenu.addAction(redoAction)

        editMenu.addSeparator()

        preferencesAction = QAction("偏好设置", self)
        preferencesAction.triggered.connect(self.onPreferences)
        editMenu.addAction(preferencesAction)

        viewMenu = menuBar.addMenu("视图(&V)")
        resetViewAction = QAction("重置视角", self)
        resetViewAction.triggered.connect(lambda: self.dualModelViewer.resetView())
        viewMenu.addAction(resetViewAction)

        viewMenu.addSeparator()

        fullscreenAction = QAction("全屏", self)
        fullscreenAction.setShortcut("F11")
        fullscreenAction.triggered.connect(self.toggleFullscreen)
        viewMenu.addAction(fullscreenAction)

    def initStatusBar(self):
        self.statusLabel = QLabel("就绪")
        self.statusBar().addWidget(self.statusLabel, 1)

        self.progressBar = QProgressBar()
        self.progressBar.setMaximumWidth(200)
        self.progressBar.setValue(0)
        self.statusBar().addPermanentWidget(self.progressBar)

        self.timerLabel = QLabel("00:00:00")
        self.timerLabel.setMinimumWidth(80)
        self.statusBar().addPermanentWidget(self.timerLabel)

    def initConnections(self):
        self.dualModelViewer.modelLoaded.connect(self.onModelLoaded)
        self.dualModelViewer.moldLoaded.connect(self.onMoldModelLoaded)
        self.parameterPanel.parametersChanged.connect(self.onParametersChanged)
        self.moldProcessPanel.moldGenerated.connect(self.onMoldGenerated)

    def onNewProject(self):
        reply = QMessageBox.question(
            self, "新建项目",
            "是否创建新项目?当前项目的未保存数据将丢失。",
            QMessageBox.Yes | QMessageBox.No
        )
        if reply == QMessageBox.Yes:
            self.statusLabel.setText("新项目已创建")
            self.progressBar.setValue(0)
            self.elapsedTime = timedelta(0)

    def onSaveProject(self):
        self.statusLabel.setText("项目正在保存...")
        QMessageBox.information(self, "保存成功", "项目已保存")

    def onPreferences(self):
        QMessageBox.information(self, "偏好设置", "该功能将在后续版本中实现")

    def onModelLoaded(self, modelInfo):
        self.statusLabel.setText(
            f"铸件模型已加载 | 顶点: {modelInfo['vertices']} | "
            f"面数: {modelInfo['faces']}"
        )

    def onMoldModelLoaded(self, moldInfo):
        self.statusLabel.setText(
            f"模具模型已加载 | 顶点: {moldInfo['vertices']} | "
            f"面数: {moldInfo['faces']}"
        )

    def onMoldGenerated(self, moldShell):
        success = self.dualModelViewer.loadMoldModel(moldShell)
        if success:
            self.statusLabel.setText("模具已生成并显示")
        else:
            self.statusLabel.setText("模具已生成(显示失败)")

    def onParametersChanged(self, parameters):
        pass

    def updateTimer(self):
        self.elapsedTime += timedelta(seconds=1)
        totalSeconds = int(self.elapsedTime.total_seconds())
        hours, remainder = divmod(totalSeconds, 3600)
        minutes, seconds = divmod(remainder, 60)
        self.timerLabel.setText(f"{hours:02d}:{minutes:02d}:{seconds:02d}")

    def toggleFullscreen(self):
        if self.isFullScreen():
            self.showNormal()
        else:
            self.showFullScreen()

    def closeEvent(self, event):
        reply = QMessageBox.question(
            self, "退出确认",
            "确定要退出应用程序吗?",
            QMessageBox.Yes | QMessageBox.No
        )
        if reply == QMessageBox.Yes:
            if self.timerDisplay.isActive():
                self.timerDisplay.stop()
            event.accept()
        else:
            event.ignore()


def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()