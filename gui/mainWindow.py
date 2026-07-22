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
from gui.languageManager import languageManager, tr


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
        self.setWindowTitle(tr("appTitle"))
        self.setGeometry(100, 100, 1600, 900)

        self.moldProcessPanel = moldProcessPanel
        self.parameterPanel = parameterPanel
        self.dualModelViewer = dualModelViewer

        self.initUI()
        self.initMenuBar()
        self.initStatusBar()
        languageManager.languageChanged.connect(self.retranslateUi)
        self.retranslateUi()

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
        self.toggleRightPanelButton.setToolTip(tr("tooltipTogglePanel"))
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

        self.tabWidget = QTabWidget()
        self.tabWidget.addTab(self.moldProcessPanel, tr("tabMoldProcess"))
        self.tabWidget.addTab(self.parameterPanel, tr("tabProcessParameters"))
        scrollContentLayout.addWidget(self.tabWidget)

        self.rightScrollContent.setLayout(scrollContentLayout)
        self.rightScrollArea.setWidget(self.rightScrollContent)
        rightLayout.addWidget(self.rightScrollArea, 1)

        bottomPanel = QWidget()
        bottomLayout = QVBoxLayout(bottomPanel)
        bottomLayout.setContentsMargins(4, 4, 4, 4)
        bottomLayout.setSpacing(6)

        self.unifiedConfigGroup = QGroupBox(tr("groupConfigFile"))
        unifiedConfigLayout = QHBoxLayout()
        self.loadUnifiedConfigButton = QPushButton(tr("buttonLoadConfig"))
        self.loadUnifiedConfigButton.clicked.connect(self.onLoadConfigClicked)
        self.saveUnifiedConfigButton = QPushButton(tr("buttonSaveConfig"))
        self.saveUnifiedConfigButton.clicked.connect(self.onSaveConfigClicked)
        self.resetUnifiedConfigButton = QPushButton(tr("buttonResetDefault"))
        self.resetUnifiedConfigButton.clicked.connect(self.onResetConfigClicked)
        unifiedConfigLayout.addWidget(self.loadUnifiedConfigButton)
        unifiedConfigLayout.addWidget(self.saveUnifiedConfigButton)
        unifiedConfigLayout.addWidget(self.resetUnifiedConfigButton)
        unifiedConfigLayout.addStretch()
        self.unifiedConfigGroup.setLayout(unifiedConfigLayout)
        bottomLayout.addWidget(self.unifiedConfigGroup)

        self.gcodeGroup = QGroupBox(tr("groupManufacturingFile"))
        gcodeLayout = QVBoxLayout()
        self.generateGcodeButton = QPushButton(tr("buttonGenerateFdmGcode"))
        self.generateGcodeButton.setEnabled(False)
        self.generateGcodeButton.clicked.connect(self.onGenerateGcodeClicked)
        gcodeLayout.addWidget(self.generateGcodeButton)
        self.generateCncButton = QPushButton(tr("buttonGenerateCncGcode"))
        self.generateCncButton.setEnabled(False)
        self.generateCncButton.clicked.connect(self.onGenerateCncClicked)
        gcodeLayout.addWidget(self.generateCncButton)
        self.gcodeGroup.setLayout(gcodeLayout)
        bottomLayout.addWidget(self.gcodeGroup)

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
        self.fileMenu = menuBar.addMenu(tr("menuFile"))

        self.newAction = QAction(tr("actionNewProject"), self)
        self.newAction.setShortcut("Ctrl+N")
        self.newAction.triggered.connect(self.intentNewProject.emit)
        self.fileMenu.addAction(self.newAction)

        self.saveAction = QAction(tr("actionSaveProject"), self)
        self.saveAction.setShortcut("Ctrl+S")
        self.saveAction.triggered.connect(self.intentSaveProject.emit)
        self.fileMenu.addAction(self.saveAction)

        self.loadManifestAction = QAction(tr("actionLoadManifest"), self)
        self.loadManifestAction.triggered.connect(self.onLoadManifestClicked)
        self.fileMenu.addAction(self.loadManifestAction)

        self.fileMenu.addSeparator()
        self.exitAction = QAction(tr("actionExit"), self)
        self.exitAction.setShortcut("Alt+F4")
        self.exitAction.triggered.connect(self.close)
        self.fileMenu.addAction(self.exitAction)

        self.viewMenu = menuBar.addMenu(tr("menuView"))
        self.resetViewAction = QAction(tr("actionResetView"), self)
        self.resetViewAction.triggered.connect(self.dualModelViewer.resetView)
        self.viewMenu.addAction(self.resetViewAction)

        self.languageMenu = menuBar.addMenu(tr("menuLanguage"))
        self.chineseAction = QAction(tr("menuChinese"), self, checkable=True)
        self.englishAction = QAction(tr("menuEnglish"), self, checkable=True)
        self.chineseAction.triggered.connect(lambda: languageManager.setLanguage("zh"))
        self.englishAction.triggered.connect(lambda: languageManager.setLanguage("en"))
        self.languageMenu.addAction(self.chineseAction)
        self.languageMenu.addAction(self.englishAction)

    def initStatusBar(self):
        self.statusLabel = QLabel(tr("statusReady"))
        self.statusBar().addWidget(self.statusLabel, 1)

        self.progressBar = QProgressBar()
        self.progressBar.setMaximumWidth(200)
        self.progressBar.setValue(0)
        self.statusBar().addPermanentWidget(self.progressBar)

        self.timerLabel = QLabel("00:00:00")
        self.timerLabel.setMinimumWidth(80)
        self.statusBar().addPermanentWidget(self.timerLabel)

    def retranslateUi(self):
        self.setWindowTitle(tr("appTitle"))
        self.toggleRightPanelButton.setToolTip(tr("tooltipTogglePanel"))
        self.tabWidget.setTabText(0, tr("tabMoldProcess"))
        self.tabWidget.setTabText(1, tr("tabProcessParameters"))
        self.unifiedConfigGroup.setTitle(tr("groupConfigFile"))
        self.loadUnifiedConfigButton.setText(tr("buttonLoadConfig"))
        self.saveUnifiedConfigButton.setText(tr("buttonSaveConfig"))
        self.resetUnifiedConfigButton.setText(tr("buttonResetDefault"))
        self.gcodeGroup.setTitle(tr("groupManufacturingFile"))
        self.generateGcodeButton.setText(tr("buttonGenerateFdmGcode"))
        self.generateCncButton.setText(tr("buttonGenerateCncGcode"))
        self.fileMenu.setTitle(tr("menuFile"))
        self.viewMenu.setTitle(tr("menuView"))
        self.languageMenu.setTitle(tr("menuLanguage"))
        self.newAction.setText(tr("actionNewProject"))
        self.saveAction.setText(tr("actionSaveProject"))
        self.loadManifestAction.setText(tr("actionLoadManifest"))
        self.exitAction.setText(tr("actionExit"))
        self.resetViewAction.setText(tr("actionResetView"))
        self.chineseAction.setText(tr("menuChinese"))
        self.englishAction.setText(tr("menuEnglish"))
        self.chineseAction.setChecked(languageManager.currentLanguage == "zh")
        self.englishAction.setChecked(languageManager.currentLanguage == "en")

    def onLoadManifestClicked(self):
        filePath, _ = QFileDialog.getOpenFileName(self, tr("dialogLoadManifest"), "", "Manifest Files (*.json);;All Files (*)")
        if filePath:
            self.intentLoadManifest.emit(filePath)

    def onLoadConfigClicked(self):
        filePath, _ = QFileDialog.getOpenFileName(self, tr("dialogLoadConfig"), "", "JSON Files (*.json);;All Files (*)")
        if filePath:
            self.intentLoadConfig.emit(filePath)

    def onSaveConfigClicked(self):
        filePath, _ = QFileDialog.getSaveFileName(self, tr("dialogSaveConfig"), "", "JSON Files (*.json);;All Files (*)")
        if filePath:
            self.intentSaveConfig.emit(filePath)

    def onResetConfigClicked(self):
        reply = QMessageBox.question(self, tr("buttonResetDefault"), tr("confirmResetConfig"), QMessageBox.Yes | QMessageBox.No,
                                     QMessageBox.No)
        if reply == QMessageBox.Yes:
            self.intentResetConfig.emit()

    def onGenerateGcodeClicked(self):
        outputPath, _ = QFileDialog.getSaveFileName(self, tr("dialogSaveGcode"), "", "G-code Files (*.gcode);;All Files (*)")
        if outputPath:
            self.generateGcodeButton.setEnabled(False)  # 锁按钮防连点
            self.intentGenerateGcode.emit(outputPath)

    def onGenerateCncClicked(self):
        outputPath, _ = QFileDialog.getSaveFileName(self, tr("dialogSaveCncGcode"), "", "G-code Files (*.gcode);;All Files (*)")
        if outputPath:
            reply = QMessageBox.question(self, tr("dialogVisualization"), tr("confirmCncVisualization"), QMessageBox.Yes | QMessageBox.No, QMessageBox.Yes)
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
