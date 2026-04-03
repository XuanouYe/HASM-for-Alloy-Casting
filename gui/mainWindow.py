from PyQt5.QtCore import Qt, pyqtSignal, QTimer
from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QMessageBox,
    QVBoxLayout, QSplitter, QTabWidget, QLabel, QProgressBar,
    QAction, QPushButton, QFileDialog, QGroupBox, QScrollArea,
    QToolButton, QHBoxLayout
)
from gui.modelViewer import ModelViewerWidget
from gui.moldProcessPanel import MoldProcessPanel
from gui.parameterPanel import ProcessParameterPanel


class MainWindow(QMainWindow):
    intentNewProject = pyqtSignal()
    intentSaveProject = pyqtSignal()
    intentLoadManifest = pyqtSignal(str)

    intentLoadConfig = pyqtSignal(str)
    intentSaveConfig = pyqtSignal(str)
    intentResetConfig = pyqtSignal()

    intentGenerateGcode = pyqtSignal(str)
    intentGenerateCnc = pyqtSignal(str, bool)

    def __init__(self, moldProcessPanel: MoldProcessPanel, parameterPanel: ProcessParameterPanel,
                 dualModelViewer: ModelViewerWidget):
        super().__init__()
        self.setWindowTitle("增减材复合制造系统控制平台")
        self.setGeometry(100, 100, 1600, 900)

        self.moldProcessPanel = moldProcessPanel
        self.parameterPanel = parameterPanel
        self.dualModelViewer = dualModelViewer

        self.initUI()
        self.initMenuBar()
        self.initStatusBar()

    def initUI(self):
        centralWidget = QWidget()
        mainLayout = QVBoxLayout()

        self.horizontalSplitter = QSplitter(Qt.Horizontal)
        self.horizontalSplitter.addWidget(self.dualModelViewer)

        self.rightPanel = QWidget()
        rightLayout = QVBoxLayout()
        rightLayout.setContentsMargins(0, 0, 0, 0)
        rightLayout.setSpacing(0)

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
        tabWidget.addTab(self.moldProcessPanel, "模具生成")
        tabWidget.addTab(self.parameterPanel, "工艺参数")
        scrollContentLayout.addWidget(tabWidget)

        self.rightScrollContent.setLayout(scrollContentLayout)
        self.rightScrollArea.setWidget(self.rightScrollContent)
        rightLayout.addWidget(self.rightScrollArea, 1)

        bottomPanel = QWidget()
        bottomLayout = QVBoxLayout(bottomPanel)
        bottomLayout.setContentsMargins(4, 4, 4, 4)
        bottomLayout.setSpacing(6)

        self.unifiedConfigGroup = QGroupBox("配置文件")
        unifiedConfigLayout = QHBoxLayout()
        self.loadUnifiedConfigButton = QPushButton("加载配置")
        self.loadUnifiedConfigButton.clicked.connect(self.onLoadConfigClicked)
        self.saveUnifiedConfigButton = QPushButton("保存配置")
        self.saveUnifiedConfigButton.clicked.connect(self.onSaveConfigClicked)
        self.resetUnifiedConfigButton = QPushButton("重置为默认")
        self.resetUnifiedConfigButton.clicked.connect(self.onResetConfigClicked)
        unifiedConfigLayout.addWidget(self.loadUnifiedConfigButton)
        unifiedConfigLayout.addWidget(self.saveUnifiedConfigButton)
        unifiedConfigLayout.addWidget(self.resetUnifiedConfigButton)
        unifiedConfigLayout.addStretch()
        self.unifiedConfigGroup.setLayout(unifiedConfigLayout)
        bottomLayout.addWidget(self.unifiedConfigGroup)

        gcodeGroup = QGroupBox("制造文件")
        gcodeLayout = QVBoxLayout()
        self.generateGcodeButton = QPushButton("生成FDM G代码")
        self.generateGcodeButton.setEnabled(False)
        self.generateGcodeButton.clicked.connect(self.onGenerateGcodeClicked)
        gcodeLayout.addWidget(self.generateGcodeButton)
        self.generateCncButton = QPushButton("生成CNC G代码")
        self.generateCncButton.setEnabled(False)
        self.generateCncButton.clicked.connect(self.onGenerateCncClicked)
        gcodeLayout.addWidget(self.generateCncButton)
        gcodeGroup.setLayout(gcodeLayout)
        bottomLayout.addWidget(gcodeGroup)

        rightLayout.addWidget(bottomPanel)

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
        newAction.triggered.connect(self.intentNewProject.emit)
        fileMenu.addAction(newAction)

        saveAction = QAction("保存项目", self)
        saveAction.setShortcut("Ctrl+S")
        saveAction.triggered.connect(self.intentSaveProject.emit)
        fileMenu.addAction(saveAction)

        loadManifestAction = QAction("加载制造清单(Manifest)", self)
        loadManifestAction.triggered.connect(self.onLoadManifestClicked)
        fileMenu.addAction(loadManifestAction)

        fileMenu.addSeparator()
        exitAction = QAction("退出", self)
        exitAction.setShortcut("Alt+F4")
        exitAction.triggered.connect(self.close)
        fileMenu.addAction(exitAction)

        viewMenu = menuBar.addMenu("视图(&V)")
        resetViewAction = QAction("重置视角", self)
        resetViewAction.triggered.connect(self.dualModelViewer.resetView)
        viewMenu.addAction(resetViewAction)

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

    def onLoadManifestClicked(self):
        filePath, _ = QFileDialog.getOpenFileName(self, "加载制造清单", "", "Manifest Files (*.json);;All Files (*)")
        if filePath:
            self.intentLoadManifest.emit(filePath)

    def onLoadConfigClicked(self):
        filePath, _ = QFileDialog.getOpenFileName(self, "加载配置", "", "JSON Files (*.json);;All Files (*)")
        if filePath:
            self.intentLoadConfig.emit(filePath)

    def onSaveConfigClicked(self):
        filePath, _ = QFileDialog.getSaveFileName(self, "保存配置", "", "JSON Files (*.json);;All Files (*)")
        if filePath:
            self.intentSaveConfig.emit(filePath)

    def onResetConfigClicked(self):
        reply = QMessageBox.question(self, "重置为默认", "确定要重置为默认配置吗?", QMessageBox.Yes | QMessageBox.No,
                                     QMessageBox.No)
        if reply == QMessageBox.Yes:
            self.intentResetConfig.emit()

    def onGenerateGcodeClicked(self):
        outputPath, _ = QFileDialog.getSaveFileName(self, "保存G代码", "", "G-code Files (*.gcode);;All Files (*)")
        if outputPath:
            self.generateGcodeButton.setEnabled(False)  # 锁按钮防连点
            self.intentGenerateGcode.emit(outputPath)

    def onGenerateCncClicked(self):
        outputPath, _ = QFileDialog.getSaveFileName(self, "保存CNC G代码", "", "G-code Files (*.gcode);;All Files (*)")
        if outputPath:
            reply = QMessageBox.question(self, "可视化", "是否在生成后启动VTK可视化窗口检查刀路？", QMessageBox.Yes | QMessageBox.No, QMessageBox.Yes)
            self.generateCncButton.setEnabled(False)
            self.intentGenerateCnc.emit(outputPath, reply == QMessageBox.Yes)

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

    def setGcodeButtonEnabled(self, enabled: bool):
        self.generateGcodeButton.setEnabled(enabled)

    def setCncButtonEnabled(self, enabled: bool):
        self.generateCncButton.setEnabled(enabled)

    def setStatusText(self, text: str):
        self.statusLabel.setText(text)

    def setTimerText(self, text: str):
        self.timerLabel.setText(text)

    def showMessage(self, title: str, msg: str, isError: bool = False):
        if isError:
            QMessageBox.critical(self, title, msg)
        else:
            QMessageBox.information(self, title, msg)
