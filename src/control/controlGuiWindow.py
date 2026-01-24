import sys
import json
from datetime import datetime, timedelta
from pathlib import Path
from typing import Optional, Callable, Dict, Any

from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QSplitter,
    QPushButton, QLabel, QLineEdit, QSpinBox, QDoubleSpinBox,
    QComboBox, QCheckBox, QProgressBar, QStatusBar, QMenuBar,
    QMenu, QToolBar, QFileDialog, QMessageBox, QGridLayout,
    QGroupBox, QSlider, QTabWidget, QTableWidget, QTableWidgetItem,
    QApplication, QFrame, QMenuBar, QAction
)
from PyQt5.QtCore import (
    Qt, QSize, QTimer, pyqtSignal, QThread, QObject, QRect,
    QPoint, QUrl
)
from PyQt5.QtGui import QIcon
from PyQt5.QtCore import QPointF

import pyvista as pv
from pyvista import QtInteractor

PYVISTA_AVAILABLE = True


class WorkerThread(QThread):
    """后台工作线程，防止长时任务阻塞UI主线程"""

    taskStarted = pyqtSignal()
    taskProgress = pyqtSignal(int)  # 进度0-100
    taskCompleted = pyqtSignal(dict)  # 返回结果字典
    taskError = pyqtSignal(str)  # 错误信息

    def __init__(self, task: Callable, *args, **kwargs):
        """
        参数：
            task: 要执行的函数
            *args, **kwargs: 传递给task的参数
        """
        super().__init__()
        self.task = task
        self.args = args
        self.kwargs = kwargs
        self.shouldStop = False

    def run(self):
        """在线程中执行任务"""
        try:
            self.taskStarted.emit()
            result = self.task(*self.args, **self.kwargs)
            self.taskCompleted.emit(result if isinstance(result, dict) else {"result": result})
        except Exception as e:
            self.taskError.emit(str(e))

    def stop(self):
        """停止线程"""
        self.shouldStop = True


# ============================================================================
# 2. 3D模型显示组件
# ============================================================================

class ModelViewerWidget(QWidget):
    """集成PyVista的3D模型显示组件"""

    modelLoaded = pyqtSignal(dict)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.initUI()
        self.currentModel = None
        self.cameraPosition = None

    def initUI(self):
        """初始化3D显示区域"""
        layout = QVBoxLayout()

        if PYVISTA_AVAILABLE:
            try:
                self.plotter = QtInteractor(self)
                self.plotter.set_background("white")
                layout.addWidget(self.plotter.interactor)
            except Exception as e:
                fallbackLabel = QLabel(f"PyVista初始化失败: {str(e)}\n请检查依赖库")
                layout.addWidget(fallbackLabel)
                self.plotter = None
        else:
            fallbackLabel = QLabel(
                "3D模型显示器\n\n"
                "请安装: pip install pyvista PyVista\n\n"
                "此区域将显示加载的STL模型"
            )
            fallbackLabel.setAlignment(Qt.AlignCenter)
            fallbackLabel.setStyleSheet("color: #666; font-size: 12px;")
            layout.addWidget(fallbackLabel)
            self.plotter = None

        self.setLayout(layout)

    def loadModel(self, filePath: str) -> bool:
        """
        加载STL模型文件

        参数：
            filePath: STL文件路径

        返回：
            成功返回True，失败返回False
        """
        if not PYVISTA_AVAILABLE or self.plotter is None:
            return False

        try:
            self.currentModel = pv.read(filePath)
            self.plotter.clear()
            self.plotter.add_mesh(
                self.currentModel,
                color="cyan",
                opacity=0.8,
                edge_color="black",
                line_width=0.5
            )
            self.plotter.reset_camera()

            # 获取模型信息
            bounds = self.currentModel.bounds
            modelInfo = {
                "filePath": filePath,
                "vertices": self.currentModel.n_points,
                "faces": self.currentModel.n_cells,
                "bounds": bounds,
                "volume": self.currentModel.volume,
                "area": self.currentModel.area
            }
            self.modelLoaded.emit(modelInfo)
            return True
        except Exception as e:
            print(f"模型加载失败: {e}")
            return False

    def resetView(self):
        """重置相机视角"""
        if self.plotter:
            self.plotter.reset_camera()

    def getModelInfo(self) -> Optional[Dict]:
        """获取当前模型信息"""
        if self.currentModel is None:
            return None

        return {
            "vertices": self.currentModel.n_points,
            "faces": self.currentModel.n_cells,
            "bounds": self.currentModel.bounds,
            "volume": self.currentModel.volume,
            "area": self.currentModel.area
        }


# ============================================================================
# 3. 工艺参数配置面板
# ============================================================================

class ProcessParameterPanel(QWidget):
    """工艺参数输入与配置面板"""

    parametersChanged = pyqtSignal(dict)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.parameters = {}
        self.initUI()

    def initUI(self):
        """初始化参数面板布局"""
        mainLayout = QVBoxLayout()

        # 使用TabWidget组织参数分类
        self.tabWidget = QTabWidget()

        # 选项卡1：铸造工艺参数
        castingTab = QWidget()
        castingLayout = QGridLayout()

        # 浇注温度
        castingLayout.addWidget(QLabel("浇注温度 (°C):"), 0, 0)
        self.pouring_temperature = QDoubleSpinBox()
        self.pouring_temperature.setRange(20, 300)
        self.pouring_temperature.setValue(120)
        self.pouring_temperature.setSuffix(" °C")
        self.pouring_temperature.valueChanged.connect(self.onParameterChanged)
        castingLayout.addWidget(self.pouring_temperature, 0, 1)

        # 铸造时间
        castingLayout.addWidget(QLabel("铸造时间 (s):"), 1, 0)
        self.casting_duration = QSpinBox()
        self.casting_duration.setRange(5, 120)
        self.casting_duration.setValue(30)
        self.casting_duration.setSuffix(" s")
        self.casting_duration.valueChanged.connect(self.onParameterChanged)
        castingLayout.addWidget(self.casting_duration, 1, 1)

        # 固化温度
        castingLayout.addWidget(QLabel("固化温度 (°C):"), 2, 0)
        self.curing_temperature = QDoubleSpinBox()
        self.curing_temperature.setRange(20, 200)
        self.curing_temperature.setValue(60)
        self.curing_temperature.setSuffix(" °C")
        self.curing_temperature.valueChanged.connect(self.onParameterChanged)
        castingLayout.addWidget(self.curing_temperature, 2, 1)

        # 固化时间
        castingLayout.addWidget(QLabel("固化时间 (min):"), 3, 0)
        self.curing_duration = QSpinBox()
        self.curing_duration.setRange(1, 480)
        self.curing_duration.setValue(30)
        self.curing_duration.setSuffix(" min")
        self.curing_duration.valueChanged.connect(self.onParameterChanged)
        castingLayout.addWidget(self.curing_duration, 3, 1)

        castingTab.setLayout(castingLayout)
        self.tabWidget.addTab(castingTab, "铸造工艺")

        # 选项卡2：减材加工参数
        subtractiveTab = QWidget()
        subtractiveLayout = QGridLayout()

        # 主轴转速
        subtractiveLayout.addWidget(QLabel("主轴转速 (rpm):"), 0, 0)
        self.spindle_speed = QSpinBox()
        self.spindle_speed.setRange(100, 10000)
        self.spindle_speed.setValue(3000)
        self.spindle_speed.setSuffix(" rpm")
        self.spindle_speed.setSingleStep(100)
        self.spindle_speed.valueChanged.connect(self.onParameterChanged)
        subtractiveLayout.addWidget(self.spindle_speed, 0, 1)

        # 进给速度
        subtractiveLayout.addWidget(QLabel("进给速度 (mm/min):"), 1, 0)
        self.feedRate = QDoubleSpinBox()
        self.feedRate.setRange(10, 1000)
        self.feedRate.setValue(200)
        self.feedRate.setSuffix(" mm/min")
        self.feedRate.setSingleStep(10)
        self.feedRate.valueChanged.connect(self.onParameterChanged)
        subtractiveLayout.addWidget(self.feedRate, 1, 1)

        # 刀具直径
        subtractiveLayout.addWidget(QLabel("刀具直径 (mm):"), 2, 0)
        self.toolDiameter = QDoubleSpinBox()
        self.toolDiameter.setRange(1, 50)
        self.toolDiameter.setValue(10)
        self.toolDiameter.setSuffix(" mm")
        self.toolDiameter.setSingleStep(0.5)
        self.toolDiameter.valueChanged.connect(self.onParameterChanged)
        subtractiveLayout.addWidget(self.toolDiameter, 2, 1)

        # 切削深度
        subtractiveLayout.addWidget(QLabel("切削深度 (mm):"), 3, 0)
        self.cuttingDepth = QDoubleSpinBox()
        self.cuttingDepth.setRange(0.1, 50)
        self.cuttingDepth.setValue(5)
        self.cuttingDepth.setSuffix(" mm")
        self.cuttingDepth.setSingleStep(0.1)
        self.cuttingDepth.valueChanged.connect(self.onParameterChanged)
        subtractiveLayout.addWidget(self.cuttingDepth, 3, 1)

        subtractiveTab.setLayout(subtractiveLayout)
        self.tabWidget.addTab(subtractiveTab, "减材加工")

        # 选项卡3：模具参数
        moldTab = QWidget()
        moldLayout = QGridLayout()

        # 模具材料
        moldLayout.addWidget(QLabel("模具材料:"), 0, 0)
        self.moldMaterial = QComboBox()
        self.moldMaterial.addItems(["PLA", "ABS", "PETG", "TPU"])
        self.moldMaterial.currentTextChanged.connect(self.onParameterChanged)
        moldLayout.addWidget(self.moldMaterial, 0, 1)

        # 模具壁厚
        moldLayout.addWidget(QLabel("模具壁厚 (mm):"), 1, 0)
        self.moldWallThickness = QDoubleSpinBox()
        self.moldWallThickness.setRange(1, 20)
        self.moldWallThickness.setValue(3)
        self.moldWallThickness.setSuffix(" mm")
        self.moldWallThickness.valueChanged.connect(self.onParameterChanged)
        moldLayout.addWidget(self.moldWallThickness, 1, 1)

        # 脱模角度
        moldLayout.addWidget(QLabel("脱模角度 (°):"), 2, 0)
        self.draftAngle = QDoubleSpinBox()
        self.draftAngle.setRange(0, 45)
        self.draftAngle.setValue(2)
        self.draftAngle.setSuffix(" °")
        self.draftAngle.setSingleStep(0.5)
        self.draftAngle.valueChanged.connect(self.onParameterChanged)
        moldLayout.addWidget(self.draftAngle, 2, 1)

        moldTab.setLayout(moldLayout)
        self.tabWidget.addTab(moldTab, "模具参数")

        mainLayout.addWidget(self.tabWidget)

        # 按钮组
        buttonLayout = QHBoxLayout()

        self.resetButton = QPushButton("重置默认值")
        self.resetButton.clicked.connect(self.resetParameters)
        buttonLayout.addWidget(self.resetButton)

        self.saveButton = QPushButton("保存配置")
        self.saveButton.clicked.connect(self.saveParameters)
        buttonLayout.addWidget(self.saveButton)

        self.loadButton = QPushButton("加载配置")
        self.loadButton.clicked.connect(self.loadParameters)
        buttonLayout.addWidget(self.loadButton)

        mainLayout.addLayout(buttonLayout)

        self.setLayout(mainLayout)

    def onParameterChanged(self):
        """参数变化时的处理"""
        self.updateParameters()
        self.parametersChanged.emit(self.parameters)

    def updateParameters(self) -> Dict[str, Any]:
        """更新参数字典"""
        self.parameters = {
            "casting": {
                "pouring_temperature": self.pouring_temperature.value(),
                "casting_duration": self.casting_duration.value(),
                "curing_temperature": self.curing_temperature.value(),
                "curing_duration": self.curing_duration.value(),
            },
            "subtractive": {
                "spindle_speed": self.spindle_speed.value(),
                "feed_rate": self.feedRate.value(),
                "tool_diameter": self.toolDiameter.value(),
                "cutting_depth": self.cuttingDepth.value(),
            },
            "mold": {
                "material": self.moldMaterial.currentText(),
                "wall_thickness": self.moldWallThickness.value(),
                "draft_angle": self.draftAngle.value(),
            }
        }
        return self.parameters

    def getParameters(self) -> Dict[str, Any]:
        """获取当前参数"""
        return self.updateParameters()

    def resetParameters(self):
        """重置为默认值"""
        self.pouring_temperature.setValue(120)
        self.casting_duration.setValue(30)
        self.curing_temperature.setValue(60)
        self.curing_duration.setValue(30)
        self.spindle_speed.setValue(3000)
        self.feedRate.setValue(200)
        self.toolDiameter.setValue(10)
        self.cuttingDepth.setValue(5)
        self.moldMaterial.setCurrentText("PLA")
        self.moldWallThickness.setValue(3)
        self.draftAngle.setValue(2)

    def saveParameters(self):
        """保存参数到JSON文件"""
        filePath, _ = QFileDialog.getSaveFileName(
            self, "保存工艺参数", "", "JSON Files (*.json)"
        )
        if filePath:
            try:
                params = self.getParameters()
                with open(filePath, 'w', encoding='utf-8') as f:
                    json.dump(params, f, indent=2, ensure_ascii=False)
                QMessageBox.information(self, "成功", "工艺参数已保存")
            except Exception as e:
                QMessageBox.critical(self, "错误", f"保存失败: {str(e)}")

    def loadParameters(self):
        """从JSON文件加载参数"""
        filePath, _ = QFileDialog.getOpenFileName(
            self, "加载工艺参数", "", "JSON Files (*.json)"
        )
        if filePath:
            try:
                with open(filePath, 'r', encoding='utf-8') as f:
                    params = json.load(f)

                # 恢复参数值
                if "casting" in params:
                    self.pouring_temperature.setValue(params["casting"].get("pouring_temperature", 120))
                    self.casting_duration.setValue(params["casting"].get("casting_duration", 30))
                    self.curing_temperature.setValue(params["casting"].get("curing_temperature", 60))
                    self.curing_duration.setValue(params["casting"].get("curing_duration", 30))

                if "subtractive" in params:
                    self.spindle_speed.setValue(params["subtractive"].get("spindle_speed", 3000))
                    self.feedRate.setValue(params["subtractive"].get("feed_rate", 200))
                    self.toolDiameter.setValue(params["subtractive"].get("tool_diameter", 10))
                    self.cuttingDepth.setValue(params["subtractive"].get("cutting_depth", 5))

                if "mold" in params:
                    self.moldMaterial.setCurrentText(params["mold"].get("material", "PLA"))
                    self.moldWallThickness.setValue(params["mold"].get("wall_thickness", 3))
                    self.draftAngle.setValue(params["mold"].get("draft_angle", 2))

                QMessageBox.information(self, "成功", "工艺参数已加载")
            except Exception as e:
                QMessageBox.critical(self, "错误", f"加载失败: {str(e)}")


# ============================================================================
# 4. 主窗口类
# ============================================================================

class MainWindow(QMainWindow):
    """增减材复合制造系统主窗口"""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("增减材复合制造系统控制平台")
        self.setGeometry(100, 100, 1400, 900)

        # 初始化变量
        self.currentTask = None
        self.elapsedTime = timedelta(0)
        self.isProcessing = False

        # UI初始化
        self.initUI()
        self.initMenuBar()
        self.initToolBar()
        self.initStatusBar()
        self.initConnections()

        # 定时器
        self.timerDisplay = QTimer()
        self.timerDisplay.timeout.connect(self.updateTimer)

    def initUI(self):
        """初始化主界面布局"""
        # 中央组件
        centralWidget = QWidget()
        mainLayout = QHBoxLayout()

        # 左侧：3D模型显示
        self.modelViewer = ModelViewerWidget()

        # 右侧：工艺参数面板
        self.parameterPanel = ProcessParameterPanel()

        # 分割器
        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(self.modelViewer)
        splitter.addWidget(self.parameterPanel)
        splitter.setSizes([700, 400])
        splitter.setCollapsible(0, False)
        splitter.setCollapsible(1, False)

        mainLayout.addWidget(splitter)
        centralWidget.setLayout(mainLayout)
        self.setCentralWidget(centralWidget)

        # 下方控制按钮区
        self.createControlButtons()

    def createControlButtons(self):
        """创建下方控制按钮"""
        # 获取中央组件布局
        centralWidget = self.centralWidget()
        mainLayout = centralWidget.layout()

        # 创建控制按钮区
        controlFrame = QFrame()
        controlFrame.setStyleSheet("""
            QFrame {
                background-color: #f0f0f0;
                border-top: 1px solid #d0d0d0;
                padding: 10px;
            }
        """)
        controlLayout = QHBoxLayout()

        # 开始按钮
        self.startButton = QPushButton("开始 (S)")
        self.startButton.setStyleSheet("""
            QPushButton {
                background-color: #4CAF50;
                color: white;
                font-weight: bold;
                padding: 8px 20px;
                border-radius: 4px;
            }
            QPushButton:hover {
                background-color: #45a049;
            }
            QPushButton:pressed {
                background-color: #3d8b40;
            }
        """)
        self.startButton.clicked.connect(self.onStartProcess)
        controlLayout.addWidget(self.startButton)

        # 暂停按钮
        self.pauseButton = QPushButton("暂停 (P)")
        self.pauseButton.setStyleSheet("""
            QPushButton {
                background-color: #ff9800;
                color: white;
                font-weight: bold;
                padding: 8px 20px;
                border-radius: 4px;
            }
            QPushButton:hover {
                background-color: #e68900;
            }
        """)
        self.pauseButton.setEnabled(False)
        self.pauseButton.clicked.connect(self.onPauseProcess)
        controlLayout.addWidget(self.pauseButton)

        # 取消按钮
        self.cancelButton = QPushButton("取消 (C)")
        self.cancelButton.setStyleSheet("""
            QPushButton {
                background-color: #f44336;
                color: white;
                font-weight: bold;
                padding: 8px 20px;
                border-radius: 4px;
            }
            QPushButton:hover {
                background-color: #da190b;
            }
        """)
        self.cancelButton.setEnabled(False)
        self.cancelButton.clicked.connect(self.onCancelProcess)
        controlLayout.addWidget(self.cancelButton)

        # 重置按钮
        self.resetButton = QPushButton("重置 (R)")
        self.resetButton.setStyleSheet("""
            QPushButton {
                background-color: #2196F3;
                color: white;
                font-weight: bold;
                padding: 8px 20px;
                border-radius: 4px;
            }
            QPushButton:hover {
                background-color: #0b7dda;
            }
        """)
        self.resetButton.clicked.connect(self.onResetProcess)
        controlLayout.addWidget(self.resetButton)

        controlLayout.addStretch()

        controlFrame.setLayout(controlLayout)

        # 将控制按钮添加到主布局
        if isinstance(mainLayout, QHBoxLayout):
            # 获取中央组件
            baseWidget = QWidget()
            baseLayout = QVBoxLayout()

            # 获取当前的分割器
            splitter = mainLayout.itemAt(0).widget()

            # 移除分割器
            mainLayout.removeWidget(splitter)

            # 重新组织布局
            baseLayout.addWidget(splitter)
            baseLayout.addWidget(controlFrame)
            baseLayout.setContentsMargins(0, 0, 0, 0)
            baseLayout.setSpacing(0)

            baseWidget.setLayout(baseLayout)
            mainLayout.addWidget(baseWidget)

    def initMenuBar(self):
        """初始化菜单栏"""
        menuBar = self.menuBar()

        # 文件菜单
        fileMenu = menuBar.addMenu("文件(&F)")

        newAction = QAction("新建项目", self)
        newAction.setShortcut("Ctrl+N")
        newAction.triggered.connect(self.onNewProject)
        fileMenu.addAction(newAction)

        openAction = QAction("打开STL文件", self)
        openAction.setShortcut("Ctrl+O")
        openAction.triggered.connect(self.onOpenSTL)
        fileMenu.addAction(openAction)

        saveAction = QAction("保存项目", self)
        saveAction.setShortcut("Ctrl+S")
        saveAction.triggered.connect(self.onSaveProject)
        fileMenu.addAction(saveAction)

        fileMenu.addSeparator()

        exitAction = QAction("退出", self)
        exitAction.setShortcut("Alt+F4")
        exitAction.triggered.connect(self.close)
        fileMenu.addAction(exitAction)

        # 编辑菜单
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

        # 工艺菜单
        processMenu = menuBar.addMenu("工艺(&P)")

        generateMoldAction = QAction("生成模具", self)
        generateMoldAction.setShortcut("Ctrl+G")
        generateMoldAction.triggered.connect(self.onGenerateMold)
        processMenu.addAction(generateMoldAction)

        sliceAction = QAction("切片处理", self)
        sliceAction.setShortcut("Ctrl+L")
        sliceAction.triggered.connect(self.onSliceModel)
        processMenu.addAction(sliceAction)

        previewAction = QAction("工艺预览", self)
        previewAction.triggered.connect(self.onPreview)
        processMenu.addAction(previewAction)

        # 视图菜单
        viewMenu = menuBar.addMenu("视图(&V)")

        resetViewAction = QAction("重置视角", self)
        resetViewAction.triggered.connect(lambda: self.modelViewer.resetView())
        viewMenu.addAction(resetViewAction)

        viewMenu.addSeparator()

        fullscreenAction = QAction("全屏", self)
        fullscreenAction.setShortcut("F11")
        fullscreenAction.triggered.connect(self.toggleFullscreen)
        viewMenu.addAction(fullscreenAction)

        # 帮助菜单
        helpMenu = menuBar.addMenu("帮助(&H)")

        documentationAction = QAction("用户手册", self)
        documentationAction.triggered.connect(self.onDocumentation)
        helpMenu.addAction(documentationAction)

        helpMenu.addSeparator()

        aboutAction = QAction("关于本软件", self)
        aboutAction.triggered.connect(self.onAbout)
        helpMenu.addAction(aboutAction)

    def initToolBar(self):
        """初始化工具栏"""
        toolBar = self.addToolBar("主工具栏")
        toolBar.setMovable(False)
        toolBar.setIconSize(QSize(24, 24))

        # 加载STL
        loadSTLAction = QAction("加载STL", self)
        loadSTLAction.triggered.connect(self.onOpenSTL)
        toolBar.addAction(loadSTLAction)

        toolBar.addSeparator()

        # 生成模具
        generateMoldAction = QAction("生成模具", self)
        generateMoldAction.triggered.connect(self.onGenerateMold)
        toolBar.addAction(generateMoldAction)

        # 切片处理
        sliceAction = QAction("切片处理", self)
        sliceAction.triggered.connect(self.onSliceModel)
        toolBar.addAction(sliceAction)

        toolBar.addSeparator()

        # 帮助按钮
        helpAction = QAction("帮助", self)
        helpAction.triggered.connect(self.onDocumentation)
        toolBar.addAction(helpAction)

    def initStatusBar(self):
        """初始化状态栏"""
        # 状态消息标签
        self.statusLabel = QLabel("就绪")
        self.statusBar().addWidget(self.statusLabel, 1)

        # 进度条
        self.progressBar = QProgressBar()
        self.progressBar.setMaximumWidth(200)
        self.progressBar.setValue(0)
        self.statusBar().addPermanentWidget(self.progressBar)

        # 时间计时器
        self.timerLabel = QLabel("00:00:00")
        self.timerLabel.setMinimumWidth(80)
        self.statusBar().addPermanentWidget(self.timerLabel)

    def initConnections(self):
        """初始化信号连接"""
        self.modelViewer.modelLoaded.connect(self.onModelLoaded)
        self.parameterPanel.parametersChanged.connect(self.onParametersChanged)

    # ========================================================================
    # 菜单事件处理
    # ========================================================================

    def onNewProject(self):
        """新建项目"""
        reply = QMessageBox.question(
            self, "新建项目",
            "是否创建新项目？当前项目的未保存数据将丢失。",
            QMessageBox.Yes | QMessageBox.No
        )
        if reply == QMessageBox.Yes:
            self.statusLabel.setText("新项目已创建")
            self.progressBar.setValue(0)
            self.elapsedTime = timedelta(0)

    def onOpenSTL(self):
        """打开STL文件"""
        filePath, _ = QFileDialog.getOpenFileName(
            self, "打开STL文件", "", "STL Files (*.stl);;All Files (*)"
        )
        if filePath:
            self.statusLabel.setText(f"加载中: {Path(filePath).name}...")

            # 在工作线程中加载模型
            def loadModel():
                return self.modelViewer.loadModel(filePath)

            worker = WorkerThread(loadModel)
            worker.taskCompleted.connect(
                lambda result: self.statusLabel.setText(
                    f"已加载: {Path(filePath).name}"
                )
            )
            worker.taskError.connect(
                lambda err: QMessageBox.critical(self, "加载失败", f"错误: {err}")
            )
            worker.start()

    def onSaveProject(self):
        """保存项目"""
        self.statusLabel.setText("项目正在保存...")
        QMessageBox.information(self, "保存成功", "项目已保存")

    def onPreferences(self):
        """打开偏好设置"""
        QMessageBox.information(self, "偏好设置", "该功能将在后续版本中实现")

    def onGenerateMold(self):
        """生成模具"""
        self.statusLabel.setText("正在生成模具...")
        self.progressBar.setValue(0)

        # 模拟耗时操作
        def generateMold():
            import time
            for i in range(11):
                time.sleep(0.5)
                self.progressBar.setValue(i * 10)
            return {"status": "success", "moldPath": "/path/to/mold.stl"}

        worker = WorkerThread(generateMold)
        worker.taskCompleted.connect(self.onMoldGenerated)
        worker.start()

    def onMoldGenerated(self, result):
        """模具生成完成"""
        self.statusLabel.setText("模具生成完成")
        self.progressBar.setValue(100)
        QMessageBox.information(self, "成功", "模具已生成")

    def onSliceModel(self):
        """切片处理"""
        self.statusLabel.setText("正在进行切片处理...")
        QMessageBox.information(self, "切片处理", "该功能将在后续版本中实现")

    def onPreview(self):
        """工艺预览"""
        self.statusLabel.setText("正在生成工艺预览...")
        QMessageBox.information(self, "预览", "工艺预览已生成")

    def onDocumentation(self):
        """打开用户手册"""
        QMessageBox.information(
            self, "用户手册",
            "用户手册：\n\n"
            "1. 加载STL文件\n"
            "2. 配置工艺参数\n"
            "3. 生成模具\n"
            "4. 启动制造流程\n"
            "5. 监视制造过程\n\n"
            "详细文档请访问: docs/manual.pdf"
        )

    def onAbout(self):
        """关于本软件"""
        QMessageBox.about(
            self, "关于本软件",
            "增减材复合制造系统控制平台\n\n"
            "版本: 1.0.0\n"
            "开发机构: 毕业设计团队\n"
            "时间: 2026年1月\n\n"
            "本软件用于液态金属增减材复合制造\n"
            "的工艺控制与管理。"
        )

    # ========================================================================
    # 控制按钮事件处理
    # ========================================================================

    def onStartProcess(self):
        """启动制造流程"""
        if not self.modelViewer.currentModel:
            QMessageBox.warning(self, "警告", "请先加载STL模型")
            return

        self.isProcessing = True
        self.startButton.setEnabled(False)
        self.pauseButton.setEnabled(True)
        self.cancelButton.setEnabled(True)

        self.statusLabel.setText("制造流程已启动...")
        self.progressBar.setValue(0)
        self.elapsedTime = timedelta(0)
        self.timerDisplay.start(1000)  # 每1秒更新一次

        # 模拟制造过程
        def manufacturingProcess():
            import time
            for i in range(101):
                if not self.isProcessing:
                    break
                time.sleep(0.5)
                self.progressBar.setValue(i)
            return {"status": "completed", "time": str(self.elapsedTime)}

        self.currentTask = WorkerThread(manufacturingProcess)
        self.currentTask.taskCompleted.connect(self.onProcessCompleted)
        self.currentTask.taskError.connect(self.onProcessError)
        self.currentTask.start()

    def onPauseProcess(self):
        """暂停制造流程"""
        self.statusLabel.setText("制造流程已暂停")
        self.timerDisplay.stop()
        self.startButton.setEnabled(True)
        self.pauseButton.setEnabled(False)

    def onCancelProcess(self):
        """取消制造流程"""
        reply = QMessageBox.question(
            self, "确认取消",
            "确定要取消当前流程吗？",
            QMessageBox.Yes | QMessageBox.No
        )
        if reply == QMessageBox.Yes:
            self.isProcessing = False
            self.timerDisplay.stop()
            self.statusLabel.setText("制造流程已取消")
            self.startButton.setEnabled(True)
            self.pauseButton.setEnabled(False)
            self.cancelButton.setEnabled(False)
            if self.currentTask:
                self.currentTask.stop()

    def onResetProcess(self):
        """重置状态"""
        self.isProcessing = False
        self.timerDisplay.stop()
        self.statusLabel.setText("就绪")
        self.progressBar.setValue(0)
        self.elapsedTime = timedelta(0)
        self.timerLabel.setText("00:00:00")
        self.startButton.setEnabled(True)
        self.pauseButton.setEnabled(False)
        self.cancelButton.setEnabled(False)

    def onProcessCompleted(self, result):
        """制造流程完成"""
        self.isProcessing = False
        self.timerDisplay.stop()
        self.statusLabel.setText("制造流程已完成")
        self.progressBar.setValue(100)
        self.startButton.setEnabled(True)
        self.pauseButton.setEnabled(False)
        self.cancelButton.setEnabled(False)

        QMessageBox.information(
            self, "完成",
            f"制造流程已完成\n耗时: {self.timerLabel.text()}"
        )

    def onProcessError(self, errorMsg):
        """处理制造过程中的错误"""
        self.isProcessing = False
        self.timerDisplay.stop()
        self.statusLabel.setText("发生错误")
        self.startButton.setEnabled(True)
        self.pauseButton.setEnabled(False)
        self.cancelButton.setEnabled(False)

        QMessageBox.critical(
            self, "错误",
            f"制造过程中发生错误:\n{errorMsg}"
        )

    # ========================================================================
    # 其他事件处理
    # ========================================================================

    def onModelLoaded(self, modelInfo):
        """模型加载完成"""
        self.statusLabel.setText(
            f"模型已加载 | 顶点: {modelInfo['vertices']} | "
            f"面数: {modelInfo['faces']}"
        )

    def onParametersChanged(self, parameters):
        """工艺参数变化"""
        # 可在此处添加参数验证或实时更新UI的逻辑
        pass

    def updateTimer(self):
        """更新计时器显示"""
        self.elapsedTime += timedelta(seconds=1)
        totalSeconds = int(self.elapsedTime.total_seconds())
        hours, remainder = divmod(totalSeconds, 3600)
        minutes, seconds = divmod(remainder, 60)
        self.timerLabel.setText(f"{hours:02d}:{minutes:02d}:{seconds:02d}")

    def toggleFullscreen(self):
        """切换全屏模式"""
        if self.isFullScreen():
            self.showNormal()
        else:
            self.showFullScreen()

    def closeEvent(self, event):
        """窗口关闭事件"""
        reply = QMessageBox.question(
            self, "退出确认",
            "确定要退出应用程序吗？",
            QMessageBox.Yes | QMessageBox.No
        )
        if reply == QMessageBox.Yes:
            if self.timerDisplay.isActive():
                self.timerDisplay.stop()
            event.accept()
        else:
            event.ignore()


# ============================================================================
# 5. 应用程序入口
# ============================================================================

def main():
    """应用程序主函数"""
    app = QApplication(sys.argv)

    # 设置应用样式
    app.setStyle("Fusion")

    # 创建主窗口
    window = MainWindow()
    window.show()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()