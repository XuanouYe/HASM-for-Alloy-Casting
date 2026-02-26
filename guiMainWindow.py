import json
import sys
from datetime import timedelta
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QApplication, QMessageBox,
    QVBoxLayout, QSplitter, QTabWidget, QLabel, QProgressBar,
    QAction, QPushButton, QFileDialog, QGroupBox, QScrollArea,
    QToolButton, QHBoxLayout
)
from guiModelViewer import ModelViewerWidget
from guiMoldProcessPanel import MoldProcessPanel
from guiParameterPanel import ProcessParameterPanel
from guiWorkerThread import WorkerThread
from controlConfig import ConfigManager
from fdmExecutor import generateGcodeInterface
from cncPathDesigner import generateCncJobInterface


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("增减材复合制造系统控制平台")
        self.setGeometry(100, 100, 1600, 900)

        self.currentTask = None
        self.elapsedTime = timedelta(0)
        self.isProcessing = False
        self.currentCastingTrimesh = None
        self.currentStlPath = None

        # Track normalized asset paths to feed into CNC step
        self.currentManifestData = None
        self.partStlPath = None
        self.moldStlPath = None
        self.gatingStlPath = None

        self.configManager = ConfigManager()
        self.currentConfigDict = self.configManager.getDefaultConfig()

        self.initUI()
        self.initMenuBar()
        self.initStatusBar()
        self.initConnections()

        self.timerDisplay = QTimer()
        self.timerDisplay.timeout.connect(self.updateTimer)

    def initUI(self):
        centralWidget = QWidget()
        mainLayout = QVBoxLayout()

        self.horizontalSplitter = QSplitter(Qt.Horizontal)

        self.dualModelViewer = ModelViewerWidget()
        self.horizontalSplitter.addWidget(self.dualModelViewer)

        self.rightPanel = QWidget()
        rightLayout = QVBoxLayout()
        rightLayout.setContentsMargins(0, 0, 0, 0)

        self.toggleRightPanelButton = QToolButton()
        self.toggleRightPanelButton.setText("◀")
        self.toggleRightPanelButton.setToolTip("折叠/展开右侧面板")
        self.toggleRightPanelButton.clicked.connect(self.onToggleRightPanel)
        toolbarLayout = QHBoxLayout()
        toolbarLayout.addWidget(self.toggleRightPanelButton)
        toolbarLayout.addStretch()
        toolbarWidget = QWidget()
        toolbarWidget.setLayout(toolbarLayout)
        rightLayout.addWidget(toolbarWidget)

        self.rightScrollArea = QScrollArea()
        self.rightScrollArea.setWidgetResizable(True)
        self.rightScrollContent = QWidget()
        scrollContentLayout = QVBoxLayout()
        scrollContentLayout.setContentsMargins(4, 4, 4, 4)

        tabWidget = QTabWidget()
        self.moldProcessPanel = MoldProcessPanel()
        tabWidget.addTab(self.moldProcessPanel, "模具生成")
        self.parameterPanel = ProcessParameterPanel()
        tabWidget.addTab(self.parameterPanel, "工艺参数")
        scrollContentLayout.addWidget(tabWidget)

        self.unifiedConfigGroup = QGroupBox("配置文件")
        unifiedConfigLayout = QHBoxLayout()
        self.loadUnifiedConfigButton = QPushButton("加载配置")
        self.loadUnifiedConfigButton.clicked.connect(self.onLoadUnifiedConfigClicked)
        self.saveUnifiedConfigButton = QPushButton("保存配置")
        self.saveUnifiedConfigButton.clicked.connect(self.onSaveUnifiedConfigClicked)
        self.resetUnifiedConfigButton = QPushButton("重置为默认")
        self.resetUnifiedConfigButton.clicked.connect(self.onResetUnifiedConfigClicked)
        unifiedConfigLayout.addWidget(self.loadUnifiedConfigButton)
        unifiedConfigLayout.addWidget(self.saveUnifiedConfigButton)
        unifiedConfigLayout.addWidget(self.resetUnifiedConfigButton)
        unifiedConfigLayout.addStretch()
        self.unifiedConfigGroup.setLayout(unifiedConfigLayout)
        scrollContentLayout.addWidget(self.unifiedConfigGroup)

        gcodeGroup = QGroupBox("制造文件")
        gcodeLayout = QVBoxLayout()
        self.generateGcodeButton = QPushButton("生成FDM G代码")
        self.generateGcodeButton.setEnabled(False)
        self.generateGcodeButton.clicked.connect(self.onGenerateGcodeClicked)
        gcodeLayout.addWidget(self.generateGcodeButton)

        self.generateCncButton = QPushButton("生成CNC刀位(CL-JSON)")
        self.generateCncButton.setEnabled(False)
        self.generateCncButton.clicked.connect(self.onGenerateCncClicked)
        gcodeLayout.addWidget(self.generateCncButton)

        gcodeGroup.setLayout(gcodeLayout)
        scrollContentLayout.addWidget(gcodeGroup)

        scrollContentLayout.addStretch()
        self.rightScrollContent.setLayout(scrollContentLayout)
        self.rightScrollArea.setWidget(self.rightScrollContent)
        rightLayout.addWidget(self.rightScrollArea, 1)

        self.rightPanel.setLayout(rightLayout)
        self.rightPanelCollapsed = False
        self.horizontalSplitter.addWidget(self.rightPanel)

        self.horizontalSplitter.setSizes([1000, 600])
        self.horizontalSplitter.setCollapsible(0, False)
        self.horizontalSplitter.setCollapsible(1, False)

        mainLayout.addWidget(self.horizontalSplitter, 1)
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

        loadManifestAction = QAction("加载制造清单(Manifest)", self)
        loadManifestAction.triggered.connect(self.onLoadManifest)
        fileMenu.addAction(loadManifestAction)

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
            self.currentManifestData = None
            self.partStlPath = None
            self.moldStlPath = None
            self.gatingStlPath = None
            self.generateCncButton.setEnabled(False)

    def onSaveProject(self):
        self.statusLabel.setText("项目正在保存...")
        QMessageBox.information(self, "保存成功", "项目已保存")

    def onPreferences(self):
        QMessageBox.information(self, "偏好设置", "该功能将在后续版本中实现")

    def onLoadManifest(self):
        filePath, _ = QFileDialog.getOpenFileName(
            self, "加载制造清单", "", "Manifest Files (*.json);;All Files (*)"
        )
        if not filePath:
            return

        try:
            with open(filePath, "r", encoding="utf-8") as f:
                manifestData = json.load(f)

            self.currentManifestData = manifestData
            files = manifestData.get("files", {})
            self.partStlPath = files.get("partStl")
            self.moldStlPath = files.get("moldStl")
            self.gatingStlPath = files.get("gatingStl")

            if self.partStlPath and self.moldStlPath:
                self.generateCncButton.setEnabled(True)
                QMessageBox.information(self, "成功", "制造清单加载成功，可以生成CNC路径")
            else:
                QMessageBox.warning(self, "警告", "制造清单缺少必要的部件模型或模具模型路径")

        except Exception as e:
            QMessageBox.critical(self, "错误", f"加载清单失败: {str(e)}")

    def onModelLoaded(self, modelInfo):
        self.currentStlPath = modelInfo.get("filePath")
        self.generateGcodeButton.setEnabled(bool(self.currentStlPath))
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
            QMessageBox.information(self, "提示", "请先通过脚本生成Manifest及标准化几何以启用CNC功能。")
        else:
            self.statusLabel.setText("模具已生成(显示失败)")

    def onParametersChanged(self, parameters):
        pass

    def onLoadUnifiedConfigClicked(self):
        filePath, _ = QFileDialog.getOpenFileName(
            self, "加载配置", "", "JSON Files (*.json);;All Files (*)"
        )
        if not filePath:
            return
        try:
            with open(filePath, "r", encoding="utf-8") as f:
                configDict = json.load(f)
            errors = self.configManager.validate(configDict)
            if errors:
                QMessageBox.warning(
                    self, "配置验证失败",
                    "配置文件存在以下问题:\n" + "\n".join(errors[:10])
                )
                return
            self.currentConfigDict = configDict
            self.parameterPanel.loadConfiguration(configDict)
            self.moldProcessPanel.loadConfiguration(configDict)
            QMessageBox.information(self, "成功", f"配置已加载: {filePath}")
        except Exception as e:
            QMessageBox.critical(self, "错误", f"加载失败: {str(e)}")

    def onSaveUnifiedConfigClicked(self):
        panelConfig = self.parameterPanel.getConfiguration()
        moldSection = self.moldProcessPanel.getMoldConfigurationSection()
        self.currentConfigDict["additive"] = panelConfig.get("additive", {})
        self.currentConfigDict["casting"] = panelConfig.get("casting", {})
        self.currentConfigDict["subtractive"] = panelConfig.get("subtractive", {})
        self.currentConfigDict["mold"] = moldSection
        filePath, _ = QFileDialog.getSaveFileName(
            self, "保存配置", "", "JSON Files (*.json);;All Files (*)"
        )
        if not filePath:
            return
        try:
            with open(filePath, "w", encoding="utf-8") as f:
                json.dump(self.currentConfigDict, f, indent=2, ensure_ascii=False)
            QMessageBox.information(self, "成功", f"配置已保存: {filePath}")
        except Exception as e:
            QMessageBox.critical(self, "错误", f"保存失败: {str(e)}")

    def onResetUnifiedConfigClicked(self):
        reply = QMessageBox.question(
            self, "重置为默认",
            "确定要重置为默认配置吗?",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No
        )
        if reply != QMessageBox.Yes:
            return
        self.currentConfigDict = self.configManager.getDefaultConfig()
        self.parameterPanel.loadConfiguration(self.currentConfigDict)
        self.moldProcessPanel.loadConfiguration(self.currentConfigDict)
        QMessageBox.information(self, "成功", "已重置为默认配置")

    def onToggleRightPanel(self):
        if self.rightPanelCollapsed:
            self.rightPanel.setMaximumWidth(16777215)
            self.rightScrollArea.show()
            self.toggleRightPanelButton.setText("◀")
            self.horizontalSplitter.setSizes([1000, 600])
            self.rightPanelCollapsed = False
        else:
            self.rightPanel.setMaximumWidth(40)
            self.rightScrollArea.hide()
            self.toggleRightPanelButton.setText("▶")
            self.rightPanelCollapsed = True

    def onGenerateGcodeClicked(self):
        if not self.currentStlPath:
            return
        config = self.parameterPanel.getConfiguration()
        outputPath, _ = QFileDialog.getSaveFileName(
            self,
            "保存G代码",
            "",
            "G-code Files (*.gcode);;All Files (*)"
        )
        if not outputPath:
            return
        self.generateGcodeButton.setEnabled(False)

        def taskCallable():
            return generateGcodeInterface(
                stlPath=self.currentStlPath,
                outputPath=outputPath,
                processConfig=config,
            )

        worker = WorkerThread(taskCallable)
        worker.taskCompleted.connect(self.onGenerateGcodeCompleted)
        worker.taskError.connect(self.onGenerateGcodeError)
        worker.start()
        self.currentGcodeWorker = worker

    def onGenerateGcodeCompleted(self, result):
        self.generateGcodeButton.setEnabled(True)
        gcodePath = result.get("result") if "result" in result else result.get("gcodePath", "")
        if gcodePath:
            QMessageBox.information(self, "成功", f"G代码已生成:\n{gcodePath}")
        else:
            QMessageBox.information(self, "完成", "生成完成")

    def onGenerateGcodeError(self, errorMsg):
        self.generateGcodeButton.setEnabled(True)
        QMessageBox.critical(self, "错误", f"生成G代码失败:\n{errorMsg}")

    def onGenerateCncClicked(self):
        if not self.partStlPath or not self.moldStlPath:
            QMessageBox.warning(self, "错误", "缺少必要的标准化STL模型路径。请先加载Manufacturing Manifest。")
            return

        config = self.parameterPanel.getConfiguration()
        outputPath, _ = QFileDialog.getSaveFileName(
            self,
            "保存CNC刀位文件",
            "",
            "JSON Files (*.json);;All Files (*)"
        )
        if not outputPath:
            return

        # Ask for visualization
        reply = QMessageBox.question(
            self, "可视化",
            "是否在生成后启动VTK可视化窗口检查刀路？",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.Yes
        )
        visualize = (reply == QMessageBox.Yes)

        self.generateCncButton.setEnabled(False)
        self.statusLabel.setText("正在计算CNC刀位...")

        def taskCallable():
            gatingPathToUse = self.gatingStlPath if self.gatingStlPath else self.partStlPath
            return generateCncJobInterface(
                partStl=self.partStlPath,
                moldStl=self.moldStlPath,
                gatingStl=gatingPathToUse,
                outputJsonPath=outputPath,
                processConfig=config,
                visualize=visualize
            )

        worker = WorkerThread(taskCallable)
        worker.taskCompleted.connect(self.onGenerateCncCompleted)
        worker.taskError.connect(self.onGenerateCncError)
        worker.start()
        self.currentCncWorker = worker

    def onGenerateCncCompleted(self, result):
        self.generateCncButton.setEnabled(True)
        self.statusLabel.setText("CNC刀位生成完成")
        QMessageBox.information(self, "完成", "CNC刀位JSON已成功生成。")

    def onGenerateCncError(self, errorMsg):
        self.generateCncButton.setEnabled(True)
        self.statusLabel.setText("CNC刀位生成失败")
        QMessageBox.critical(self, "错误", f"生成CNC刀位失败:\n{errorMsg}")

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
