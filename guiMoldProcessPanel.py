from pathlib import Path
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QFormLayout,
    QPushButton, QLabel, QDoubleSpinBox, QGroupBox,
    QRadioButton, QButtonGroup, QMessageBox, QFileDialog
)
from controlConfig import ConfigManager


class MoldProcessPanel(QWidget):
    # ==== 意图信号 (Intent Signals) ====
    # 将用户的操作意图以及参数打包发送给 Controller
    intentLoadModel = pyqtSignal(str)
    intentGenerateMold = pyqtSignal(dict)
    intentAddGating = pyqtSignal(dict)
    intentOptimizeOrientation = pyqtSignal(dict)
    intentAdjustStructure = pyqtSignal(dict)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.configManager = ConfigManager()
        self.currentConfig = self.configManager.getDefaultConfig()

        # 纯 UI 状态，仅用于构建底部的状态文本显示
        self.statusFlags = {
            "loaded": False,
            "gated": False,
            "molded": False,
            "oriented": False,
            "adjusted": False
        }
        self.initUI()

    def initUI(self):
        mainLayout = QVBoxLayout()
        mainLayout.setSpacing(15)
        mainLayout.setContentsMargins(10, 10, 10, 10)

        titleLabel = QLabel("模具生成与处理")
        titleFont = QFont()
        titleFont.setPointSize(12)
        titleFont.setBold(True)
        titleLabel.setFont(titleFont)

        mainLayout.addWidget(titleLabel)
        mainLayout.addWidget(self.createLoadModelGroup())
        mainLayout.addWidget(self.createAddGatingGroup())
        mainLayout.addWidget(self.createMoldGenerationGroup())
        mainLayout.addWidget(self.createOrientationGroup())
        mainLayout.addWidget(self.createSurfaceOffsetGroup())
        mainLayout.addWidget(self.createStatusDisplay())
        mainLayout.addStretch()

        self.setLayout(mainLayout)
        self.setStyleSheet(self.getStylesheet())

    def createLoadModelGroup(self) -> QGroupBox:
        group = QGroupBox("模型加载")
        layout = QVBoxLayout()
        self.loadSTLButton = QPushButton("加载STL文件")
        self.loadSTLButton.clicked.connect(self.onLoadSTLClick)
        layout.addWidget(self.loadSTLButton)
        group.setLayout(layout)
        return group

    def createMoldGenerationGroup(self) -> QGroupBox:
        group = QGroupBox("模具生成")
        layout = QVBoxLayout()
        paramLayout = QFormLayout()
        moldSchema = self.configManager.getParameterSchema("mold")
        bboxSpec = moldSchema.get("boundingBoxOffset", {})

        self.boundingBoxOffsetSpinBox = QDoubleSpinBox()
        self.boundingBoxOffsetSpinBox.setRange(bboxSpec.get("min", 0.1), bboxSpec.get("max", 50.0))
        self.boundingBoxOffsetSpinBox.setValue(
            self.currentConfig.get("mold", {}).get("boundingBoxOffset", bboxSpec.get("default", 2.0)))
        self.boundingBoxOffsetSpinBox.setSuffix(f" {bboxSpec.get('unit', 'mm')}")
        self.boundingBoxOffsetSpinBox.setDecimals(1)
        paramLayout.addRow("包围盒偏移:", self.boundingBoxOffsetSpinBox)
        layout.addLayout(paramLayout)

        self.generateMoldButton = QPushButton("生成模具")
        self.generateMoldButton.setEnabled(False)
        self.generateMoldButton.clicked.connect(self.onGenerateMoldClick)
        layout.addWidget(self.generateMoldButton)
        group.setLayout(layout)
        return group

    def createAddGatingGroup(self) -> QGroupBox:
        gatingGroup = QGroupBox("添加浇道")
        layout = QVBoxLayout()
        paramLayout = QFormLayout()
        moldSchema = self.configManager.getParameterSchema("mold")

        fillTimeSpec = moldSchema.get("targetFillTime", {})
        self.fillTimeSpinBox = QDoubleSpinBox()
        self.fillTimeSpinBox.setRange(fillTimeSpec.get("min", 0.1), fillTimeSpec.get("max", 100.0))
        self.fillTimeSpinBox.setValue(
            self.currentConfig.get("mold", {}).get("targetFillTime", fillTimeSpec.get("default", 5.0)))
        self.fillTimeSpinBox.setSuffix(f" {fillTimeSpec.get('unit', 's')}")
        self.fillTimeSpinBox.setDecimals(1)
        paramLayout.addRow("目标填充时间:", self.fillTimeSpinBox)

        sprueSpec = moldSchema.get("sprueInletOffset", {})
        self.sprueOffsetSpinBox = QDoubleSpinBox()
        self.sprueOffsetSpinBox.setRange(sprueSpec.get("min", 0.0), sprueSpec.get("max", 100.0))
        self.sprueOffsetSpinBox.setValue(
            self.currentConfig.get("mold", {}).get("sprueInletOffset", sprueSpec.get("default", 10.0)))
        self.sprueOffsetSpinBox.setSuffix(f" {sprueSpec.get('unit', 'mm')}")
        self.sprueOffsetSpinBox.setDecimals(1)
        paramLayout.addRow("浇口偏移:", self.sprueOffsetSpinBox)
        layout.addLayout(paramLayout)

        self.addGatingButton = QPushButton("添加浇道")
        self.addGatingButton.setEnabled(False)
        self.addGatingButton.clicked.connect(self.onAddGatingClick)
        layout.addWidget(self.addGatingButton)
        gatingGroup.setLayout(layout)
        return gatingGroup

    def createOrientationGroup(self) -> QGroupBox:
        orientationGroup = QGroupBox("方向优化")
        orientationLayout = QVBoxLayout()
        self.orientationButtonGroup = QButtonGroup(self)
        self.millingRadio = QRadioButton("基于铣削可达性")
        self.printingRadio = QRadioButton("基于打印可达性")
        self.millingRadio.setChecked(True)
        self.orientationButtonGroup.addButton(self.millingRadio, 0)
        self.orientationButtonGroup.addButton(self.printingRadio, 1)

        orientationLayout.addWidget(self.millingRadio)
        orientationLayout.addWidget(self.printingRadio)

        self.optimizeOrientationButton = QPushButton("执行优化")
        self.optimizeOrientationButton.setEnabled(False)
        self.optimizeOrientationButton.clicked.connect(self.onOptimizeOrientationClick)
        orientationLayout.addWidget(self.optimizeOrientationButton)
        orientationGroup.setLayout(orientationLayout)
        return orientationGroup

    def createSurfaceOffsetGroup(self) -> QGroupBox:
        surfaceOffsetGroup = QGroupBox("表面偏移")
        surfaceOffsetLayout = QVBoxLayout()
        offsetInputLayout = QHBoxLayout()
        offsetInputLayout.addWidget(QLabel("偏移值:"))
        self.surfaceOffsetSpinBox = QDoubleSpinBox()
        self.surfaceOffsetSpinBox.setRange(-10.0, 10.0)
        self.surfaceOffsetSpinBox.setValue(0.5)
        self.surfaceOffsetSpinBox.setSuffix(" mm")
        self.surfaceOffsetSpinBox.setDecimals(2)
        offsetInputLayout.addWidget(self.surfaceOffsetSpinBox)
        surfaceOffsetLayout.addLayout(offsetInputLayout)

        self.adjustStructureButton = QPushButton("执行偏移")
        self.adjustStructureButton.setEnabled(False)
        self.adjustStructureButton.clicked.connect(self.onAdjustStructureClick)
        surfaceOffsetLayout.addWidget(self.adjustStructureButton)
        surfaceOffsetGroup.setLayout(surfaceOffsetLayout)
        return surfaceOffsetGroup

    def createStatusDisplay(self) -> QGroupBox:
        group = QGroupBox("处理状态")
        layout = QVBoxLayout()
        self.statusLabel = QLabel("等待加载模型...")
        self.statusLabel.setStyleSheet("color: #666; padding: 5px;")
        self.statusLabel.setWordWrap(True)
        layout.addWidget(self.statusLabel)
        group.setLayout(layout)
        return group

    # ====== 1. 用户交互 -> 发射意图信号 (Action) ======
    def onLoadSTLClick(self):
        filePath, _ = QFileDialog.getOpenFileName(self, "打开STL文件", "", "STL Files (*.stl);;All Files (*)")
        if filePath:
            self.statusLabel.setText(f"加载中: {Path(filePath).name}...")
            self.statusLabel.setStyleSheet("color: #0078d4; padding: 5px;")
            self.intentLoadModel.emit(filePath)

    def onGenerateMoldClick(self):
        self.generateMoldButton.setEnabled(False)
        self.addGatingButton.setEnabled(False)
        self.statusLabel.setText("正在生成模具...")
        self.statusLabel.setStyleSheet("color: #0078d4; padding: 5px;")
        config = {
            "boundingBoxOffset": self.boundingBoxOffsetSpinBox.value(),
            "booleanEngine": None
        }
        self.intentGenerateMold.emit(config)

    def onAddGatingClick(self):
        self.addGatingButton.setEnabled(False)
        self.generateMoldButton.setEnabled(False)
        self.statusLabel.setText("正在添加浇道...")
        self.statusLabel.setStyleSheet("color: #0078d4; padding: 5px;")
        config = {
            "targetFillTime": self.fillTimeSpinBox.value(),
            "sprueInletOffset": self.sprueOffsetSpinBox.value(),
            "boundingBoxOffset": self.boundingBoxOffsetSpinBox.value()
        }
        self.intentAddGating.emit(config)

    def onOptimizeOrientationClick(self):
        self.optimizeOrientationButton.setEnabled(False)
        optType = "milling" if self.millingRadio.isChecked() else "printing"
        self.statusLabel.setText(f"正在执行方向优化({optType})...")
        self.statusLabel.setStyleSheet("color: #0078d4; padding: 5px;")
        self.intentOptimizeOrientation.emit({"optimizationType": optType})

    def onAdjustStructureClick(self):
        self.adjustStructureButton.setEnabled(False)
        val = self.surfaceOffsetSpinBox.value()
        self.statusLabel.setText(f"正在执行表面偏移({val} mm)...")
        self.statusLabel.setStyleSheet("color: #0078d4; padding: 5px;")
        self.intentAdjustStructure.emit({"offsetValue": val})

    # ====== 2. 接收 Controller 反馈 -> 更新 UI 状态 (Response) ======
    def onModelLoadedSuccess(self):
        self.statusFlags["loaded"] = True
        self.statusFlags["molded"] = False
        self.statusFlags["gated"] = False
        self.generateMoldButton.setEnabled(True)
        self.addGatingButton.setEnabled(True)
        self.optimizeOrientationButton.setEnabled(False)
        self.adjustStructureButton.setEnabled(False)
        self._updateStatusText("模型已加载, 可以添加浇道或生成模具")

    def onMoldGeneratedSuccess(self):
        self.statusFlags["molded"] = True
        self.generateMoldButton.setEnabled(True)
        self.addGatingButton.setEnabled(True)
        self.optimizeOrientationButton.setEnabled(True)
        self.adjustStructureButton.setEnabled(True)
        self._updateStatusText()
        QMessageBox.information(self, "成功", "模具生成完成")

    def onGatingAddedSuccess(self):
        self.statusFlags["gated"] = True
        self.statusFlags["molded"] = False  # 加了浇道后需要重新生成模具
        self.addGatingButton.setEnabled(True)
        self.generateMoldButton.setEnabled(True)
        self._updateStatusText()
        QMessageBox.information(self, "成功", "浇道添加完成, 请继续生成模具")

    def onOrientationOptimizedSuccess(self, optType: str):
        self.statusFlags["oriented"] = True
        self.optimizeOrientationButton.setEnabled(True)
        self._updateStatusText()
        QMessageBox.information(self, "提示", f"方向优化已完成 ({optType})")

    def onStructureAdjustedSuccess(self, offsetVal: float):
        self.statusFlags["adjusted"] = True
        self.adjustStructureButton.setEnabled(True)
        self._updateStatusText()
        QMessageBox.information(self, "提示", f"表面偏移已完成 ({offsetVal} mm)")

    def onProcessError(self, title: str, errMsg: str):
        # 发生错误时，恢复按钮可用状态
        if self.statusFlags["loaded"]:
            self.generateMoldButton.setEnabled(True)
            self.addGatingButton.setEnabled(True)
        if self.statusFlags["molded"]:
            self.optimizeOrientationButton.setEnabled(True)
            self.adjustStructureButton.setEnabled(True)

        self.statusLabel.setText(f"✗ {title}")
        self.statusLabel.setStyleSheet("color: red; padding: 5px;")
        QMessageBox.critical(self, title, f"{title}:\n{errMsg}")

    # ====== 辅助方法 ======
    def loadConfiguration(self, configDict):
        moldConfig = configDict.get("mold") or {}
        if "mold" not in self.currentConfig:
            self.currentConfig["mold"] = {}
        self.currentConfig["mold"].update(moldConfig)
        self.boundingBoxOffsetSpinBox.setValue(moldConfig.get("boundingBoxOffset", 2.0))
        self.fillTimeSpinBox.setValue(moldConfig.get("targetFillTime", 5.0))
        self.sprueOffsetSpinBox.setValue(moldConfig.get("sprueInletOffset", 5.0))

    def getMoldConfigurationSection(self):
        return {
            "boundingBoxOffset": self.boundingBoxOffsetSpinBox.value(),
            "targetFillTime": self.fillTimeSpinBox.value(),
            "sprueInletOffset": self.sprueOffsetSpinBox.value(),
        }

    def _updateStatusText(self, defaultMsg=None):
        if defaultMsg:
            self.statusLabel.setText(defaultMsg)
            self.statusLabel.setStyleSheet("color: #0078d4; padding: 5px;")
            return

        parts = []
        if self.statusFlags["molded"]: parts.append("✓ 模具已生成")
        if self.statusFlags["gated"]: parts.append("✓ 浇道已添加")
        if self.statusFlags["oriented"]: parts.append("✓ 方向已优化")
        if self.statusFlags["adjusted"]: parts.append("✓ 表面已偏移")

        self.statusLabel.setText(" | ".join(parts))
        self.statusLabel.setStyleSheet("color: green; padding: 5px; font-weight: bold;")

    def getStylesheet(self) -> str:
        return """
        QGroupBox { border: 1px solid #cccccc; border-radius: 5px; margin-top: 10px; padding-top: 10px; font-weight: bold; }
        QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px 0 5px; }
        QPushButton { background-color: #0078d4; color: white; border: none; border-radius: 4px; padding: 8px 16px; font-weight: bold; }
        QPushButton:hover { background-color: #1084d7; }
        QPushButton:pressed { background-color: #005a9e; }
        QPushButton:disabled { background-color: #cccccc; color: #888888; }
        """
