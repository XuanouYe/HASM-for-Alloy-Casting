"""
增减材复合制造系统控制平台主窗口 - 完整版本

功能特性：
- 集成3D模型显示（PyVista）
- 工艺参数配置与管理（参考fdmSliceParameterUI的设计模式）
- 多工艺阶段参数（铸造、减材、模具）
- 参数配置保存/加载
- 后台任务处理（不阻塞UI）
- 工艺流程控制与监视
- 完整的菜单栏、工具栏、状态栏

命名规范：所有变量和函数使用驼峰命名法
"""

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
    QApplication, QFrame, QAction, QFormLayout, QScrollArea,
    QSizePolicy
)

from PyQt5.QtCore import (
    Qt, QSize, QTimer, pyqtSignal, QThread, QObject, QRect,
    QPoint, QUrl, QRect
)

from PyQt5.QtGui import QIcon, QFont, QColor

import pyvista as pv
from pyvista import QtInteractor

PYVISTA_AVAILABLE = True


# ============================================================================
# 1. 后台工作线程
# ============================================================================

class WorkerThread(QThread):
    """后台工作线程，防止长时任务阻塞UI主线程"""

    taskStarted = pyqtSignal()
    taskProgress = pyqtSignal(int)  # 进度0-100
    taskCompleted = pyqtSignal(dict)  # 返回结果字典
    taskError = pyqtSignal(str)  # 错误信息

    def __init__(self, task: Callable, *args, **kwargs):
        """
        初始化工作线程

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
        self.wait()


# ============================================================================
# 2. 3D模型显示组件
# ============================================================================

class ModelViewerWidget(QWidget):
    """集成PyVista的3D模型显示组件"""

    modelLoaded = pyqtSignal(dict)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.currentModel = None
        self.cameraPosition = None
        self.initUI()

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

        except Exception:
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
        self.parameterControls = {}
        self.changeCallbacks = []
        self.configFilePath = None
        self.initUI()

    def initUI(self):
        """初始化参数面板布局"""
        mainLayout = QVBoxLayout()
        mainLayout.setSpacing(10)
        mainLayout.setContentsMargins(10, 10, 10, 10)

        # 标题
        titleLabel = QLabel("工艺参数配置")
        titleFont = QFont()
        titleFont.setPointSize(12)
        titleFont.setBold(True)
        titleLabel.setFont(titleFont)
        mainLayout.addWidget(titleLabel)

        # 预设选择栏
        mainLayout.addWidget(self._createPresetBar())

        # 参数标签页
        mainLayout.addWidget(self._createParameterTabs())

        # 操作按钮栏
        mainLayout.addWidget(self._createActionBar())

        # 参数状态显示
        mainLayout.addWidget(self._createStatusBar())

        mainLayout.addStretch()

        self.setLayout(mainLayout)
        self.setStyleSheet(self._getStylesheet())

    def _createPresetBar(self) -> QGroupBox:
        """创建预设选择栏"""
        group = QGroupBox("预设配置")
        layout = QHBoxLayout()

        label = QLabel("选择预设:")
        self.presetComboBox = QComboBox()

        # 预设列表
        presets = [
            "standard",
            "fast",
            "quality",
            "high precision"
        ]

        self.presetComboBox.addItems(presets)
        self.presetComboBox.currentTextChanged.connect(self._onPresetChanged)

        layout.addWidget(label)
        layout.addWidget(self.presetComboBox, 1)

        group.setLayout(layout)
        return group

    def _createParameterTabs(self) -> QTabWidget:
        """创建参数标签页"""
        tabWidget = QTabWidget()
        tabWidget.addTab(self._createAdditiveParametersTab(), "增材工艺")
        tabWidget.addTab(self._createCastingParametersTab(), "铸造工艺")
        tabWidget.addTab(self._createSubtractiveParametersTab(), "减材加工")
        return tabWidget

    def _createAdditiveParametersTab(self) -> QTabWidget:
        """创建参数标签页"""
        tabWidget = QTabWidget()
        tabWidget.addTab(self._createAdditiveParametersTabBasic(), "基础配置")
        tabWidget.addTab(self._createAdditiveParametersTabAdvanced(), "高级配置")
        return tabWidget

    def _createAdditiveParametersTabBasic(self) -> QWidget:
        widget = QWidget()
        layout = QFormLayout()

        # 层高 (0.1 - 0.4 mm)
        self.layerHeightSpinBox = QDoubleSpinBox()
        self.layerHeightSpinBox.setRange(0.08, 0.4)
        self.layerHeightSpinBox.setSingleStep(0.05)
        self.layerHeightSpinBox.setValue(0.2)
        self.layerHeightSpinBox.setSuffix(" mm")
        self.layerHeightSpinBox.valueChanged.connect(self._onParameterChanged)
        self.parameterControls["layer_height"] = self.layerHeightSpinBox
        layout.addRow("层高 (Layer Height):", self.layerHeightSpinBox)

        # 壁厚 (0.4 - 2.0 mm)
        self.wallThicknessSpinBox = QDoubleSpinBox()
        self.wallThicknessSpinBox.setRange(0.4, 2.0)
        self.wallThicknessSpinBox.setSingleStep(0.1)
        self.wallThicknessSpinBox.setValue(0.8)
        self.wallThicknessSpinBox.setSuffix(" mm")
        self.wallThicknessSpinBox.valueChanged.connect(self._onParameterChanged)
        self.parameterControls["wall_thickness"] = self.wallThicknessSpinBox
        layout.addRow("壁厚 (Wall Thickness):", self.wallThicknessSpinBox)

        # 壁线数
        self.wallLineCountSpinBox = QSpinBox()
        self.wallLineCountSpinBox.setRange(1, 10)
        self.wallLineCountSpinBox.setValue(2)
        self.wallLineCountSpinBox.valueChanged.connect(self._onParameterChanged)
        self.parameterControls["wall_line_count"] = self.wallLineCountSpinBox
        layout.addRow("壁线数 (Wall Line Count):", self.wallLineCountSpinBox)

        # 填充密度 (0 - 100%)
        self.infillDensitySpinBox = QSpinBox()
        self.infillDensitySpinBox.setRange(0, 100)
        self.infillDensitySpinBox.setSingleStep(5)
        self.infillDensitySpinBox.setValue(50)
        self.infillDensitySpinBox.setSuffix(" %")
        self.infillDensitySpinBox.valueChanged.connect(self._onParameterChanged)
        self.parameterControls["infill_density"] = self.infillDensitySpinBox
        layout.addRow("填充密度 (Infill Density):", self.infillDensitySpinBox)

        # 填充模式
        self.infillPatternComboBox = QComboBox()
        self.infillPatternComboBox.addItems(["grid", "honeycomb", "gyroid", "cubic"])
        self.infillPatternComboBox.currentTextChanged.connect(self._onParameterChanged)
        self.parameterControls["infill_pattern"] = self.infillPatternComboBox
        layout.addRow("填充模式 (Infill Pattern):", self.infillPatternComboBox)

        # 顶层数
        self.topLayersSpinBox = QSpinBox()
        self.topLayersSpinBox.setRange(0, 20)
        self.topLayersSpinBox.setValue(4)
        self.topLayersSpinBox.valueChanged.connect(self._onParameterChanged)
        self.parameterControls["top_layers"] = self.topLayersSpinBox
        layout.addRow("顶层数 (Top Layers):", self.topLayersSpinBox)

        # 底层数
        self.bottomLayersSpinBox = QSpinBox()
        self.bottomLayersSpinBox.setRange(0, 20)
        self.bottomLayersSpinBox.setValue(4)
        self.bottomLayersSpinBox.valueChanged.connect(self._onParameterChanged)
        self.parameterControls["bottom_layers"] = self.bottomLayersSpinBox
        layout.addRow("底层数 (Bottom Layers):", self.bottomLayersSpinBox)

        # 模具材料
        self.moldMaterial = QComboBox()
        self.moldMaterial.addItems(["PLA", "ABS", "PETG", "TPU", "Nylon"])
        self.moldMaterial.currentTextChanged.connect(self._onParameterChanged)
        self.parameterControls["mold_material"] = self.moldMaterial
        layout.addRow("模具材料 (Mold Material):", self.moldMaterial)

        # 打印温度 (20 - 450°C)
        self.printTemperatureSpinBox = QSpinBox()
        self.printTemperatureSpinBox.setRange(20, 450)
        self.printTemperatureSpinBox.setSingleStep(5)
        self.printTemperatureSpinBox.setValue(210)
        self.printTemperatureSpinBox.setSuffix(" °C")
        self.printTemperatureSpinBox.valueChanged.connect(self._onParameterChanged)
        self.parameterControls["print_temperature"] = self.printTemperatureSpinBox
        layout.addRow("打印温度 (Print Temperature):", self.printTemperatureSpinBox)

        # 热床温度 (20 - 100°C)
        self.bedTemperatureSpinBox = QSpinBox()
        self.bedTemperatureSpinBox.setRange(20, 100)
        self.bedTemperatureSpinBox.setSingleStep(5)
        self.bedTemperatureSpinBox.setValue(60)
        self.bedTemperatureSpinBox.setSuffix(" °C")
        self.bedTemperatureSpinBox.valueChanged.connect(self._onParameterChanged)
        self.parameterControls["bed_temperature"] = self.bedTemperatureSpinBox
        layout.addRow("热床温度 (Bed Temperature):", self.bedTemperatureSpinBox)


        widget.setLayout(layout)
        return widget

    def _createAdditiveParametersTabAdvanced(self) -> QWidget:
        widget = QWidget()
        layout = QFormLayout()

        # 打印速度 (10 - 150 mm/s)
        self.printSpeedSpinBox = QSpinBox()
        self.printSpeedSpinBox.setRange(10, 150)
        self.printSpeedSpinBox.setSingleStep(5)
        self.printSpeedSpinBox.setValue(50)
        self.printSpeedSpinBox.setSuffix(" mm/s")
        self.printSpeedSpinBox.valueChanged.connect(self._onParameterChanged)
        self.parameterControls["print_speed"] = self.printSpeedSpinBox
        layout.addRow("打印速度 (Print Speed):", self.printSpeedSpinBox)

        # 首层打印速度
        self.firstLayerSpeedSpinBox = QSpinBox()
        self.firstLayerSpeedSpinBox.setRange(5, 100)
        self.firstLayerSpeedSpinBox.setSingleStep(2)
        self.firstLayerSpeedSpinBox.setValue(20)
        self.firstLayerSpeedSpinBox.setSuffix(" mm/s")
        self.firstLayerSpeedSpinBox.valueChanged.connect(self._onParameterChanged)
        self.parameterControls["print_speed_layer_0"] = self.firstLayerSpeedSpinBox
        layout.addRow("首层速度:", self.firstLayerSpeedSpinBox)

        # 移动速度
        self.travelSpeedSpinBox = QSpinBox()
        self.travelSpeedSpinBox.setRange(50, 300)
        self.travelSpeedSpinBox.setSingleStep(10)
        self.travelSpeedSpinBox.setValue(150)
        self.travelSpeedSpinBox.setSuffix(" mm/s")
        self.travelSpeedSpinBox.valueChanged.connect(self._onParameterChanged)
        self.parameterControls["travel_speed"] = self.travelSpeedSpinBox
        layout.addRow("移动速度 (Travel Speed):", self.travelSpeedSpinBox)

        # 冷却风扇开启
        self.fanEnabledCheckBox = QCheckBox("启用冷却风扇")
        self.fanEnabledCheckBox.setChecked(False)
        self.fanEnabledCheckBox.stateChanged.connect(self._onParameterChanged)
        self.parameterControls["fan_enabled"] = self.fanEnabledCheckBox
        layout.addRow("冷却风扇:", self.fanEnabledCheckBox)

        # 风扇速度
        self.fanSpeedSpinBox = QSpinBox()
        self.fanSpeedSpinBox.setRange(0, 100)
        self.fanSpeedSpinBox.setSingleStep(10)
        self.fanSpeedSpinBox.setValue(100)
        self.fanSpeedSpinBox.setSuffix(" %")
        self.fanSpeedSpinBox.valueChanged.connect(self._onParameterChanged)
        self.parameterControls["cool_fan_speed"] = self.fanSpeedSpinBox
        layout.addRow("风扇速度:", self.fanSpeedSpinBox)

        # 退丝启用
        self.retractionEnabledCheckBox = QCheckBox("启用退丝")
        self.retractionEnabledCheckBox.setChecked(False)
        self.retractionEnabledCheckBox.stateChanged.connect(self._onParameterChanged)
        self.parameterControls["retraction_enabled"] = self.retractionEnabledCheckBox
        layout.addRow("启用退丝:", self.retractionEnabledCheckBox)

        # 退丝距离
        self.retractionDistanceSpinBox = QDoubleSpinBox()
        self.retractionDistanceSpinBox.setRange(0, 20)
        self.retractionDistanceSpinBox.setSingleStep(0.5)
        self.retractionDistanceSpinBox.setValue(5)
        self.retractionDistanceSpinBox.setSuffix(" mm")
        self.retractionDistanceSpinBox.valueChanged.connect(self._onParameterChanged)
        self.parameterControls["retraction_distance"] = self.retractionDistanceSpinBox
        layout.addRow("退丝距离:", self.retractionDistanceSpinBox)

        # 退丝速度
        self.retractionSpeedSpinBox = QSpinBox()
        self.retractionSpeedSpinBox.setRange(10, 100)
        self.retractionSpeedSpinBox.setSingleStep(5)
        self.retractionSpeedSpinBox.setValue(45)
        self.retractionSpeedSpinBox.setSuffix(" mm/s")
        self.retractionSpeedSpinBox.valueChanged.connect(self._onParameterChanged)
        self.parameterControls["retraction_speed"] = self.retractionSpeedSpinBox
        layout.addRow("退丝速度:", self.retractionSpeedSpinBox)

        # 最小层打印时间
        self.minLayerTimeSpinBox = QSpinBox()
        self.minLayerTimeSpinBox.setRange(1, 60)
        self.minLayerTimeSpinBox.setSingleStep(1)
        self.minLayerTimeSpinBox.setValue(10)
        self.minLayerTimeSpinBox.setSuffix(" s")
        self.minLayerTimeSpinBox.valueChanged.connect(self._onParameterChanged)
        self.parameterControls["cool_min_layer_time"] = self.minLayerTimeSpinBox
        layout.addRow("最小层打印时间:", self.minLayerTimeSpinBox)

        # 支撑启用
        self.supportEnabledCheckBox = QCheckBox("启用支撑")
        self.supportEnabledCheckBox.setChecked(False)
        self.supportEnabledCheckBox.stateChanged.connect(self._onParameterChanged)
        self.parameterControls["support_enabled"] = self.supportEnabledCheckBox
        layout.addRow("启用支撑:", self.supportEnabledCheckBox)

        # 粘附类型
        self.adhesionTypeComboBox = QComboBox()
        self.adhesionTypeComboBox.addItems(["none", "skirt", "brim", "raft"])
        self.adhesionTypeComboBox.currentTextChanged.connect(self._onParameterChanged)
        self.parameterControls["adhesion_type"] = self.adhesionTypeComboBox
        layout.addRow("粘附类型:", self.adhesionTypeComboBox)


        widget.setLayout(layout)
        return widget

    def _createCastingParametersTab(self) -> QWidget:
        """创建铸造工艺参数标签页"""
        widget = QWidget()
        layout = QFormLayout()

        # 浇注温度 (20 - 300°C)
        self.pouringTemperature = QDoubleSpinBox()
        self.pouringTemperature.setRange(20, 300)
        self.pouringTemperature.setSingleStep(5)
        self.pouringTemperature.setValue(120)
        self.pouringTemperature.setSuffix(" °C")
        self.pouringTemperature.valueChanged.connect(self._onParameterChanged)
        self.parameterControls["pouring_temperature"] = self.pouringTemperature
        layout.addRow("浇注温度 (Pouring Temperature):", self.pouringTemperature)

        # 铸造时间 (5 - 120 s)
        self.castingDuration = QSpinBox()
        self.castingDuration.setRange(5, 120)
        self.castingDuration.setSingleStep(5)
        self.castingDuration.setValue(30)
        self.castingDuration.setSuffix(" s")
        self.castingDuration.valueChanged.connect(self._onParameterChanged)
        self.parameterControls["casting_duration"] = self.castingDuration
        layout.addRow("铸造时间 (Casting Duration):", self.castingDuration)

        # 固化温度 (20 - 200°C)
        self.curingTemperature = QDoubleSpinBox()
        self.curingTemperature.setRange(20, 200)
        self.curingTemperature.setSingleStep(5)
        self.curingTemperature.setValue(60)
        self.curingTemperature.setSuffix(" °C")
        self.curingTemperature.valueChanged.connect(self._onParameterChanged)
        self.parameterControls["curing_temperature"] = self.curingTemperature
        layout.addRow("固化温度 (Curing Temperature):", self.curingTemperature)

        # 固化时间 (1 - 480 min)
        self.curingDuration = QSpinBox()
        self.curingDuration.setRange(1, 480)
        self.curingDuration.setSingleStep(10)
        self.curingDuration.setValue(30)
        self.curingDuration.setSuffix(" min")
        self.curingDuration.valueChanged.connect(self._onParameterChanged)
        self.parameterControls["curing_duration"] = self.curingDuration
        layout.addRow("固化时间 (Curing Duration):", self.curingDuration)

        widget.setLayout(layout)
        return widget

    def _createSubtractiveParametersTab(self) -> QWidget:
        """创建减材加工参数标签页"""
        widget = QWidget()
        layout = QFormLayout()

        # 主轴转速 (100 - 10000 rpm)
        self.spindleSpeed = QSpinBox()
        self.spindleSpeed.setRange(100, 10000)
        self.spindleSpeed.setSingleStep(100)
        self.spindleSpeed.setValue(3000)
        self.spindleSpeed.setSuffix(" rpm")
        self.spindleSpeed.valueChanged.connect(self._onParameterChanged)
        self.parameterControls["spindle_speed"] = self.spindleSpeed
        layout.addRow("主轴转速 (Spindle Speed):", self.spindleSpeed)

        # 进给速度 (10 - 1000 mm/min)
        self.feedRate = QDoubleSpinBox()
        self.feedRate.setRange(10, 1000)
        self.feedRate.setSingleStep(10)
        self.feedRate.setValue(200)
        self.feedRate.setSuffix(" mm/min")
        self.feedRate.valueChanged.connect(self._onParameterChanged)
        self.parameterControls["feed_rate"] = self.feedRate
        layout.addRow("进给速度 (Feed Rate):", self.feedRate)

        # 刀具直径 (1 - 50 mm)
        self.toolDiameter = QDoubleSpinBox()
        self.toolDiameter.setRange(1, 50)
        self.toolDiameter.setSingleStep(0.5)
        self.toolDiameter.setValue(10)
        self.toolDiameter.setSuffix(" mm")
        self.toolDiameter.valueChanged.connect(self._onParameterChanged)
        self.parameterControls["tool_diameter"] = self.toolDiameter
        layout.addRow("刀具直径 (Tool Diameter):", self.toolDiameter)

        # 切削深度 (0.1 - 50 mm)
        self.cuttingDepth = QDoubleSpinBox()
        self.cuttingDepth.setRange(0.1, 50)
        self.cuttingDepth.setSingleStep(0.1)
        self.cuttingDepth.setValue(5)
        self.cuttingDepth.setSuffix(" mm")
        self.cuttingDepth.valueChanged.connect(self._onParameterChanged)
        self.parameterControls["cutting_depth"] = self.cuttingDepth
        layout.addRow("切削深度 (Cutting Depth):", self.cuttingDepth)

        # 冷却液启用
        self.coolantEnabled = QCheckBox("启用冷却液")
        self.coolantEnabled.setChecked(True)
        self.coolantEnabled.stateChanged.connect(self._onParameterChanged)
        self.parameterControls["coolant_enabled"] = self.coolantEnabled
        layout.addRow("冷却液:", self.coolantEnabled)

        widget.setLayout(layout)
        return widget

    def _createActionBar(self) -> QGroupBox:
        """创建操作按钮栏"""
        group = QGroupBox("操作")
        layout = QHBoxLayout()

        # 保存配置按钮
        self.saveCfgButton = QPushButton("保存配置")
        self.saveCfgButton.clicked.connect(self._onSaveConfiguration)
        layout.addWidget(self.saveCfgButton)

        # 加载配置按钮
        self.loadCfgButton = QPushButton("加载配置")
        self.loadCfgButton.clicked.connect(self._onLoadConfiguration)
        layout.addWidget(self.loadCfgButton)

        # 重置按钮
        self.resetButton = QPushButton("重置为默认")
        self.resetButton.clicked.connect(self._onReset)
        layout.addWidget(self.resetButton)

        # 导出为JSON
        self.exportButton = QPushButton("导出JSON")
        self.exportButton.clicked.connect(self._onExport)
        layout.addWidget(self.exportButton)

        layout.addStretch()

        group.setLayout(layout)
        return group

    def _createStatusBar(self) -> QGroupBox:
        """创建状态显示栏"""
        group = QGroupBox("当前配置状态")
        layout = QVBoxLayout()

        self.statusLabel = QLabel("已加载: standard 预设")
        self.statusLabel.setStyleSheet("color: green; font-weight: bold;")

        layout.addWidget(self.statusLabel)
        group.setLayout(layout)
        return group

    def _onParameterChanged(self) -> None:
        """参数变化回调"""
        self._updateCurrentConfig()

        # 发出信号
        self.parametersChanged.emit(self.parameters)

        # 执行回调函数
        for callback in self.changeCallbacks:
            callback(self.parameters)

    def _onPresetChanged(self, presetName: str) -> None:
        """预设切换回调"""
        # 加载预设配置
        presetConfig = self._loadPresetConfig(presetName)
        if presetConfig:
            self.loadConfiguration(presetConfig)

        self.statusLabel.setText(f"已加载: {presetName} 预设")

    def _loadPresetConfig(self, presetName: str) -> Optional[Dict]:
        """加载预设配置"""
        presetConfigs = {
            "standard": {
                "pouring_temperature": 120,
                "casting_duration": 30,
                "curing_temperature": 60,
                "curing_duration": 30,
                "spindle_speed": 3000,
                "feed_rate": 200,
                "tool_diameter": 10,
                "cutting_depth": 5,
                "mold_material": "PLA",
                "mold_wall_thickness": 3,
                "draft_angle": 2,
            },
            "fast": {
                "pouring_temperature": 150,
                "casting_duration": 20,
                "curing_temperature": 70,
                "curing_duration": 20,
                "spindle_speed": 4000,
                "feed_rate": 300,
                "tool_diameter": 12,
                "cutting_depth": 8,
                "mold_material": "ABS",
                "mold_wall_thickness": 2,
                "draft_angle": 1.5,
            },
            "quality": {
                "pouring_temperature": 100,
                "casting_duration": 60,
                "curing_temperature": 50,
                "curing_duration": 60,
                "spindle_speed": 2000,
                "feed_rate": 100,
                "tool_diameter": 8,
                "cutting_depth": 2,
                "mold_material": "PETG",
                "mold_wall_thickness": 4,
                "draft_angle": 3,
            },
            "high_precision": {
                "pouring_temperature": 110,
                "casting_duration": 90,
                "curing_temperature": 55,
                "curing_duration": 90,
                "spindle_speed": 1500,
                "feed_rate": 80,
                "tool_diameter": 6,
                "cutting_depth": 1,
                "mold_material": "PETG",
                "mold_wall_thickness": 5,
                "draft_angle": 2.5,
            }
        }

        return presetConfigs.get(presetName)

    def _onSaveConfiguration(self) -> None:
        """保存配置按钮回调"""
        filePath, _ = QFileDialog.getSaveFileName(
            self,
            "保存工艺配置",
            "",
            "JSON Files (*.json);;All Files (*)"
        )

        if filePath:
            try:
                with open(filePath, 'w', encoding='utf-8') as f:
                    json.dump(self.parameters, f, indent=2, ensure_ascii=False)
                QMessageBox.information(self, "成功", f"配置已保存到: {filePath}")
            except Exception as e:
                QMessageBox.critical(self, "错误", f"保存失败: {str(e)}")

    def _onLoadConfiguration(self) -> None:
        """加载配置按钮回调"""
        filePath, _ = QFileDialog.getOpenFileName(
            self,
            "加载工艺配置",
            "",
            "JSON Files (*.json);;All Files (*)"
        )

        if filePath:
            try:
                with open(filePath, 'r', encoding='utf-8') as f:
                    config = json.load(f)

                self.loadConfiguration(config)
                QMessageBox.information(self, "成功", f"配置已加载: {filePath}")
            except Exception as e:
                QMessageBox.critical(self, "错误", f"加载失败: {str(e)}")

    def _onReset(self) -> None:
        """重置按钮回调"""
        reply = QMessageBox.question(
            self,
            "确认重置",
            "确定要重置为默认参数吗？",
            QMessageBox.Yes | QMessageBox.No
        )

        if reply == QMessageBox.Yes:
            self._resetToDefault()
            self.statusLabel.setText("已重置为默认配置")

    def _onExport(self) -> None:
        """导出JSON按钮回调"""
        jsonStr = json.dumps(self.parameters, indent=2, ensure_ascii=False)

        print("\n" + "=" * 60)
        print("当前配置 (JSON格式):")
        print("=" * 60)
        print(jsonStr)
        print("=" * 60 + "\n")

    def _updateCurrentConfig(self) -> None:
        """更新当前配置字典"""
        self.parameters = {
            "casting": {
                "pouring_temperature": self.pouringTemperature.value(),
                "casting_duration": self.castingDuration.value(),
                "curing_temperature": self.curingTemperature.value(),
                "curing_duration": self.curingDuration.value(),
            },
            "subtractive": {
                "spindle_speed": self.spindleSpeed.value(),
                "feed_rate": self.feedRate.value(),
                "tool_diameter": self.toolDiameter.value(),
                "cutting_depth": self.cuttingDepth.value(),
                "coolant_enabled": self.coolantEnabled.isChecked(),
            },
            "mold": {
                "material": self.moldMaterial.currentText(),
                "wall_thickness": self.moldWallThickness.value(),
                "draft_angle": self.draftAngle.value(),
                "support_enabled": self.supportEnabled.isChecked(),
                "support_density": self.supportDensity.value(),
            }
        }

    def _resetToDefault(self) -> None:
        """重置为默认参数"""
        self.pouringTemperature.setValue(120)
        self.castingDuration.setValue(30)
        self.curingTemperature.setValue(60)
        self.curingDuration.setValue(30)
        self.spindleSpeed.setValue(3000)
        self.feedRate.setValue(200)
        self.toolDiameter.setValue(10)
        self.cuttingDepth.setValue(5)
        self.coolantEnabled.setChecked(True)
        self.moldMaterial.setCurrentText("PLA")
        self.moldWallThickness.setValue(3)
        self.draftAngle.setValue(2)
        self.supportEnabled.setChecked(False)
        self.supportDensity.setValue(30)

    def loadConfiguration(self, configDict: Dict) -> None:
        """
        加载配置字典

        参数：
            configDict: 配置字典
        """
        try:
            # 铸造工艺参数
            if "casting" in configDict:
                casting = configDict["casting"]
                if "pouring_temperature" in casting:
                    self.pouringTemperature.setValue(casting["pouring_temperature"])
                if "casting_duration" in casting:
                    self.castingDuration.setValue(casting["casting_duration"])
                if "curing_temperature" in casting:
                    self.curingTemperature.setValue(casting["curing_temperature"])
                if "curing_duration" in casting:
                    self.curingDuration.setValue(casting["curing_duration"])

            # 减材加工参数
            if "subtractive" in configDict:
                subtractive = configDict["subtractive"]
                if "spindle_speed" in subtractive:
                    self.spindleSpeed.setValue(subtractive["spindle_speed"])
                if "feed_rate" in subtractive:
                    self.feedRate.setValue(subtractive["feed_rate"])
                if "tool_diameter" in subtractive:
                    self.toolDiameter.setValue(subtractive["tool_diameter"])
                if "cutting_depth" in subtractive:
                    self.cuttingDepth.setValue(subtractive["cutting_depth"])
                if "coolant_enabled" in subtractive:
                    self.coolantEnabled.setChecked(subtractive["coolant_enabled"])

            # 模具参数
            if "mold" in configDict:
                mold = configDict["mold"]
                if "material" in mold:
                    self.moldMaterial.setCurrentText(mold["material"])
                if "wall_thickness" in mold:
                    self.moldWallThickness.setValue(mold["wall_thickness"])
                if "draft_angle" in mold:
                    self.draftAngle.setValue(mold["draft_angle"])
                if "support_enabled" in mold:
                    self.supportEnabled.setChecked(mold["support_enabled"])
                if "support_density" in mold:
                    self.supportDensity.setValue(mold["support_density"])

            self._updateCurrentConfig()

        except Exception:
            pass

    def getConfiguration(self) -> Dict:
        """
        获取当前配置

        返回：
            配置字典
        """
        self._updateCurrentConfig()
        return self.parameters.copy()

    def registerChangeCallback(self, callback: Callable) -> None:
        """
        注册参数变化回调函数

        参数：
            callback: 回调函数 (接收config字典)
        """
        self.changeCallbacks.append(callback)

    def _getStylesheet(self) -> str:
        """获取样式表"""
        return """
            QGroupBox {
                border: 1px solid #cccccc;
                border-radius: 5px;
                margin-top: 6px;
                padding-top: 6px;
                font-weight: bold;
            }

            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 3px 0 3px;
            }

            QPushButton {
                background-color: #0078d4;
                color: white;
                border: none;
                border-radius: 4px;
                padding: 6px 12px;
                font-weight: bold;
            }

            QPushButton:hover {
                background-color: #1084d7;
            }

            QPushButton:pressed {
                background-color: #005a9e;
            }

            QTabWidget::pane {
                border: 1px solid #cccccc;
            }

            QTabBar::tab {
                background-color: #e8e8e8;
                border: 1px solid #cccccc;
                padding: 5px 15px;
            }

            QTabBar::tab:selected {
                background-color: #0078d4;
                color: white;
            }
        """


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
        mainLayout = QVBoxLayout()

        # 上方：3D模型显示和工艺参数（左右分割）
        horizontalSplitter = QSplitter(Qt.Horizontal)

        self.modelViewer = ModelViewerWidget()
        self.parameterPanel = ProcessParameterPanel()

        horizontalSplitter.addWidget(self.modelViewer)
        horizontalSplitter.addWidget(self.parameterPanel)
        horizontalSplitter.setSizes([600, 400])
        horizontalSplitter.setCollapsible(0, False)
        horizontalSplitter.setCollapsible(1, False)

        mainLayout.addWidget(horizontalSplitter, 1)

        # 下方：控制按钮
        controlFrame = self._createControlFrame()
        mainLayout.addWidget(controlFrame, 0)

        centralWidget.setLayout(mainLayout)
        self.setCentralWidget(centralWidget)

    def _createControlFrame(self) -> QFrame:
        """创建控制按钮框架"""
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
                min-width: 100px;
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
                min-width: 100px;
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
                min-width: 100px;
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
                min-width: 100px;
            }
            QPushButton:hover {
                background-color: #0b7dda;
            }
        """)
        self.resetButton.clicked.connect(self.onResetProcess)
        controlLayout.addWidget(self.resetButton)

        controlLayout.addStretch()

        controlFrame.setLayout(controlLayout)
        return controlFrame

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
        if not self.modelViewer.currentModel:
            QMessageBox.warning(self, "警告", "请先加载STL模型")
            return

        self.statusLabel.setText("正在生成模具...")
        self.progressBar.setValue(0)

        # 模拟耗时操作
        def generateMold():
            import time
            for i in range(11):
                if not self.isProcessing:
                    break
                time.sleep(0.5)
                self.progressBar.setValue(i * 10)
            return {"status": "success", "moldPath": "/path/to/mold.stl"}

        worker = WorkerThread(generateMold)
        worker.taskCompleted.connect(self.onMoldGenerated)
        worker.taskError.connect(
            lambda err: QMessageBox.critical(self, "错误", f"生成失败: {err}")
        )
        worker.start()

    def onMoldGenerated(self, result):
        """模具生成完成"""
        self.statusLabel.setText("模具生成完成")
        self.progressBar.setValue(100)
        QMessageBox.information(self, "成功", "模具已生成")

    def onSliceModel(self):
        """切片处理"""
        if not self.modelViewer.currentModel:
            QMessageBox.warning(self, "警告", "请先加载STL模型")
            return

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
        # 参数验证或实时更新UI的逻辑
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
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()