"""
CuraEngine切片参数GUI面板 - PyQt5实现

功能：
- 可视化配置切片参数
- 支持预设切换
- 自定义参数保存/加载
- 实时参数验证
"""

import sys
import json
from pathlib import Path
from typing import Dict, Callable, Optional

try:
    from PyQt5.QtWidgets import (
        QWidget, QVBoxLayout, QHBoxLayout, QGroupBox, QLabel,
        QSlider, QSpinBox, QDoubleSpinBox, QComboBox, QPushButton,
        QCheckBox, QFileDialog, QMessageBox, QTabWidget, QScrollArea,
        QFormLayout, QFrame, QSizePolicy
    )
    from PyQt5.QtCore import Qt, pyqtSignal, QTimer
    from PyQt5.QtGui import QFont, QColor, QIcon

    PYQT5_AVAILABLE = True
except ImportError:
    PYQT5_AVAILABLE = False
    print("Warning: PyQt5 not available. GUI components will be disabled.")


class CuraSettingsPanel(QWidget):
    """
    CuraEngine切片参数设置面板

    信号:
        parametersChanged: 参数变化时发出（发送参数字典）
        presetChanged: 预设切换时发出（发送预设名称）
    """

    parametersChanged = pyqtSignal(dict) if PYQT5_AVAILABLE else None
    presetChanged = pyqtSignal(str) if PYQT5_AVAILABLE else None

    def __init__(self, parent=None):
        """
        初始化设置面板

        参数:
            parent: 父部件
        """
        super().__init__(parent)

        self.currentConfig = {}
        self.configFilePath = None

        # 参数控件映射
        self.parameterControls = {}
        self.changeCallbacks = []

        # 初始化UI
        self._initializeUI()

    def _initializeUI(self) -> None:
        """初始化用户界面"""
        mainLayout = QVBoxLayout(self)
        mainLayout.setSpacing(10)
        mainLayout.setContentsMargins(10, 10, 10, 10)

        # 1. 标题
        titleLabel = QLabel("CuraEngine 切片参数配置")
        titleFont = QFont()
        titleFont.setPointSize(12)
        titleFont.setBold(True)
        titleLabel.setFont(titleFont)
        mainLayout.addWidget(titleLabel)

        # 2. 预设选择栏
        mainLayout.addWidget(self._createPresetBar())

        # 3. 参数标签页
        mainLayout.addWidget(self._createParameterTabs())

        # 4. 操作按钮栏
        mainLayout.addWidget(self._createActionBar())

        # 5. 参数状态显示
        mainLayout.addWidget(self._createStatusBar())

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
            "high_temp_450",
            "standard",
            "fast",
            "quality"
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

        # 标签页1: 基础参数
        tabWidget.addTab(self._createBasicParametersTab(), "基础参数")

        # 标签页2: 温度和速度
        tabWidget.addTab(self._createTemperatureSpeedTab(), "温度和速度")

        # 标签页3: 高级参数
        tabWidget.addTab(self._createAdvancedParametersTab(), "高级参数")

        return tabWidget

    def _createBasicParametersTab(self) -> QWidget:
        """创建基础参数标签页"""
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

        # 壁线数
        self.wallLineCountSpinBox = QSpinBox()
        self.wallLineCountSpinBox.setRange(1, 10)
        self.wallLineCountSpinBox.setValue(2)
        self.wallLineCountSpinBox.valueChanged.connect(self._onParameterChanged)
        self.parameterControls["wall_line_count"] = self.wallLineCountSpinBox
        layout.addRow("壁线数 (Wall Line Count):", self.wallLineCountSpinBox)

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

        widget.setLayout(layout)
        return widget

    def _createTemperatureSpeedTab(self) -> QWidget:
        """创建温度和速度参数标签页"""
        widget = QWidget()
        layout = QFormLayout()

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
        layout.addRow("首层打印速度:", self.firstLayerSpeedSpinBox)

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
        self.fanEnabledCheckBox.setChecked(True)
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

        widget.setLayout(layout)
        return widget

    def _createAdvancedParametersTab(self) -> QWidget:
        """创建高级参数标签页"""
        widget = QWidget()
        layout = QFormLayout()

        # 退丝启用
        self.retractionEnabledCheckBox = QCheckBox("启用退丝")
        self.retractionEnabledCheckBox.setChecked(True)
        self.retractionEnabledCheckBox.stateChanged.connect(self._onParameterChanged)
        self.parameterControls["retraction_enabled"] = self.retractionEnabledCheckBox
        layout.addRow("退丝:", self.retractionEnabledCheckBox)

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
        layout.addRow("支撑:", self.supportEnabledCheckBox)

        # 粘附类型
        self.adhesionTypeComboBox = QComboBox()
        self.adhesionTypeComboBox.addItems(["none", "skirt", "brim", "raft"])
        self.adhesionTypeComboBox.currentTextChanged.connect(self._onParameterChanged)
        self.parameterControls["adhesion_type"] = self.adhesionTypeComboBox
        layout.addRow("粘附类型:", self.adhesionTypeComboBox)

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
        if self.parametersChanged:
            self.parametersChanged.emit(self.currentConfig)

        # 执行回调函数
        for callback in self.changeCallbacks:
            callback(self.currentConfig)

    def _onPresetChanged(self, presetName: str) -> None:
        """预设切换回调"""
        self.logger = logging.getLogger("CuraSettingsPanel")
        self.logger.info(f"Switching to preset: {presetName}")

        # 发出信号
        if self.presetChanged:
            self.presetChanged.emit(presetName)

        self.statusLabel.setText(f"已加载: {presetName} 预设")

    def _onSaveConfiguration(self) -> None:
        """保存配置按钮回调"""
        filePath, _ = QFileDialog.getSaveFileName(
            self,
            "保存切片配置",
            "",
            "JSON Files (*.json);;All Files (*)"
        )

        if filePath:
            try:
                with open(filePath, 'w', encoding='utf-8') as f:
                    json.dump(self.currentConfig, f, indent=2, ensure_ascii=False)

                QMessageBox.information(self, "成功", f"配置已保存到: {filePath}")
            except Exception as e:
                QMessageBox.critical(self, "错误", f"保存失败: {str(e)}")

    def _onLoadConfiguration(self) -> None:
        """加载配置按钮回调"""
        filePath, _ = QFileDialog.getOpenFileName(
            self,
            "加载切片配置",
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
        jsonStr = json.dumps(self.currentConfig, indent=2, ensure_ascii=False)
        print("\n" + "=" * 60)
        print("当前配置 (JSON格式):")
        print("=" * 60)
        print(jsonStr)
        print("=" * 60 + "\n")

    def _updateCurrentConfig(self) -> None:
        """更新当前配置字典"""
        self.currentConfig = {
            "layer_height": self.layerHeightSpinBox.value(),
            "wall_thickness": self.wallThicknessSpinBox.value(),
            "infill_density": self.infillDensitySpinBox.value(),
            "infill_pattern": self.infillPatternComboBox.currentText(),
            "wall_line_count": self.wallLineCountSpinBox.value(),
            "top_layers": self.topLayersSpinBox.value(),
            "bottom_layers": self.bottomLayersSpinBox.value(),
            "print_temperature": self.printTemperatureSpinBox.value(),
            "bed_temperature": self.bedTemperatureSpinBox.value(),
            "print_speed": self.printSpeedSpinBox.value(),
            "print_speed_layer_0": self.firstLayerSpeedSpinBox.value(),
            "travel_speed": self.travelSpeedSpinBox.value(),
            "fan_enabled": self.fanEnabledCheckBox.isChecked(),
            "cool_fan_speed": self.fanSpeedSpinBox.value(),
            "retraction_enabled": self.retractionEnabledCheckBox.isChecked(),
            "retraction_distance": self.retractionDistanceSpinBox.value(),
            "retraction_speed": self.retractionSpeedSpinBox.value(),
            "cool_min_layer_time": self.minLayerTimeSpinBox.value(),
            "support_enabled": self.supportEnabledCheckBox.isChecked(),
            "adhesion_type": self.adhesionTypeComboBox.currentText(),
        }

    def _resetToDefault(self) -> None:
        """重置为默认参数"""
        self.layerHeightSpinBox.setValue(0.2)
        self.wallThicknessSpinBox.setValue(0.8)
        self.infillDensitySpinBox.setValue(50)
        self.wallLineCountSpinBox.setValue(2)
        self.topLayersSpinBox.setValue(4)
        self.bottomLayersSpinBox.setValue(4)
        self.printTemperatureSpinBox.setValue(210)
        self.bedTemperatureSpinBox.setValue(60)
        self.printSpeedSpinBox.setValue(50)
        self.firstLayerSpeedSpinBox.setValue(20)
        self.travelSpeedSpinBox.setValue(150)
        self.fanEnabledCheckBox.setChecked(True)
        self.fanSpeedSpinBox.setValue(100)
        self.retractionEnabledCheckBox.setChecked(True)
        self.retractionDistanceSpinBox.setValue(5)
        self.retractionSpeedSpinBox.setValue(45)
        self.minLayerTimeSpinBox.setValue(10)
        self.supportEnabledCheckBox.setChecked(False)
        self.adhesionTypeComboBox.setCurrentText("none")

    def loadConfiguration(self, configDict: Dict) -> None:
        """
        加载配置字典

        参数:
            configDict: 配置字典
        """
        try:
            if "layer_height" in configDict:
                self.layerHeightSpinBox.setValue(configDict["layer_height"])
            if "wall_thickness" in configDict:
                self.wallThicknessSpinBox.setValue(configDict["wall_thickness"])
            if "infill_density" in configDict:
                self.infillDensitySpinBox.setValue(configDict["infill_density"])
            if "infill_pattern" in configDict:
                self.infillPatternComboBox.setCurrentText(configDict["infill_pattern"])
            if "wall_line_count" in configDict:
                self.wallLineCountSpinBox.setValue(configDict["wall_line_count"])
            if "top_layers" in configDict:
                self.topLayersSpinBox.setValue(configDict["top_layers"])
            if "bottom_layers" in configDict:
                self.bottomLayersSpinBox.setValue(configDict["bottom_layers"])
            if "print_temperature" in configDict:
                self.printTemperatureSpinBox.setValue(configDict["print_temperature"])
            if "bed_temperature" in configDict:
                self.bedTemperatureSpinBox.setValue(configDict["bed_temperature"])
            if "print_speed" in configDict:
                self.printSpeedSpinBox.setValue(configDict["print_speed"])
            if "print_speed_layer_0" in configDict:
                self.firstLayerSpeedSpinBox.setValue(configDict["print_speed_layer_0"])
            if "travel_speed" in configDict:
                self.travelSpeedSpinBox.setValue(configDict["travel_speed"])
            if "fan_enabled" in configDict:
                self.fanEnabledCheckBox.setChecked(configDict["fan_enabled"])
            if "cool_fan_speed" in configDict:
                self.fanSpeedSpinBox.setValue(configDict["cool_fan_speed"])
            if "retraction_enabled" in configDict:
                self.retractionEnabledCheckBox.setChecked(configDict["retraction_enabled"])
            if "retraction_distance" in configDict:
                self.retractionDistanceSpinBox.setValue(configDict["retraction_distance"])
            if "retraction_speed" in configDict:
                self.retractionSpeedSpinBox.setValue(configDict["retraction_speed"])
            if "cool_min_layer_time" in configDict:
                self.minLayerTimeSpinBox.setValue(configDict["cool_min_layer_time"])
            if "support_enabled" in configDict:
                self.supportEnabledCheckBox.setChecked(configDict["support_enabled"])
            if "adhesion_type" in configDict:
                self.adhesionTypeComboBox.setCurrentText(configDict["adhesion_type"])

            self._updateCurrentConfig()
        except Exception as e:
            print(f"Error loading configuration: {str(e)}")

    def getConfiguration(self) -> Dict:
        """
        获取当前配置

        返回:
            配置字典
        """
        self._updateCurrentConfig()
        return self.currentConfig.copy()

    def registerChangeCallback(self, callback: Callable) -> None:
        """
        注册参数变化回调函数

        参数:
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
        """


# 示例使用和测试
if __name__ == "__main__" and PYQT5_AVAILABLE:
    import logging
    logging.basicConfig(level=logging.INFO)
    from PyQt5.QtWidgets import QApplication
    app = QApplication(sys.argv)
    panel = CuraSettingsPanel()
    panel.setWindowTitle("CuraEngine 切片参数配置")
    panel.resize(800, 900)
    panel.show()
    sys.exit(app.exec_())
