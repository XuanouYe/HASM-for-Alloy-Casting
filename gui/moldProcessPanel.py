from pathlib import Path
from PyQt5.QtCore import pyqtSignal, pyqtSlot
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QFormLayout,
    QPushButton, QLabel, QDoubleSpinBox, QGroupBox,
    QRadioButton, QButtonGroup, QMessageBox, QFileDialog, QScrollArea
)
from controlConfig import ConfigManager


class MoldProcessPanel(QWidget):
    intentLoadModel = pyqtSignal(str)
    intentGenerateMold = pyqtSignal(dict)
    intentAddGating = pyqtSignal(dict)
    intentOptimizeOrientation = pyqtSignal(dict)
    intentAdjustStructure = pyqtSignal(dict)
    intentExportMold = pyqtSignal(str)
    statusMessageChanged = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.configManager = ConfigManager()
        self.currentConfig = self.configManager.getDefaultConfig()
        self.statusFlags = {
            "loaded": False,
            "gated": False,
            "molded": False,
            "oriented": False,
            "adjusted": False
        }
        self.initUI()

    def initUI(self):
        outerLayout = QVBoxLayout()
        outerLayout.setContentsMargins(0, 0, 0, 0)
        outerLayout.setSpacing(0)

        titleLabel = QLabel("模具生成与处理")
        titleFont = QFont()
        titleFont.setPointSize(12)
        titleFont.setBold(True)
        titleLabel.setFont(titleFont)
        outerLayout.addWidget(titleLabel)

        scrollArea = QScrollArea()
        scrollArea.setWidgetResizable(True)
        scrollArea.setFrameShape(QScrollArea.NoFrame)

        contentWidget = QWidget()
        mainLayout = QVBoxLayout(contentWidget)
        mainLayout.setSpacing(15)
        mainLayout.setContentsMargins(10, 10, 10, 10)

        mainLayout.addWidget(self.createLoadModelGroup())
        mainLayout.addWidget(self.createAddGatingGroup())
        mainLayout.addWidget(self.createMoldGenerationGroup())
        mainLayout.addWidget(self.createOrientationGroup())
        mainLayout.addWidget(self.createSurfaceOffsetGroup())
        mainLayout.addStretch()

        scrollArea.setWidget(contentWidget)
        outerLayout.addWidget(scrollArea, 1)

        self.setLayout(outerLayout)
        self.setStyleSheet(self.getStylesheet())

    def createLoadModelGroup(self) -> QGroupBox:
        group = QGroupBox("模型加载")
        layout = QVBoxLayout()
        self.loadSTLButton = QPushButton("加载STL文件")
        self.loadSTLButton.clicked.connect(self.onLoadSTLClick)
        layout.addWidget(self.loadSTLButton)
        group.setLayout(layout)
        return group

    def createAddGatingGroup(self) -> QGroupBox:
        gatingGroup = QGroupBox("添加浇道")
        layout = QVBoxLayout()
        paramLayout = QFormLayout()
        moldSchema = self.configManager.getParameterSchema("mold")

        runnerSpec = moldSchema.get("runnerDiameter", {})
        self.runnerDiameterSpinBox = QDoubleSpinBox()
        self.runnerDiameterSpinBox.setRange(runnerSpec.get("min", 1.0), runnerSpec.get("max", 100.0))
        self.runnerDiameterSpinBox.setValue(
            self.currentConfig.get("mold", {}).get("runnerDiameter", runnerSpec.get("default", 6.0)))
        self.runnerDiameterSpinBox.setSuffix(f" {runnerSpec.get('unit', 'mm')}")
        self.runnerDiameterSpinBox.setDecimals(2)
        self.runnerDiameterSpinBox.setSingleStep(0.5)
        paramLayout.addRow("浇道直径:", self.runnerDiameterSpinBox)

        sprueSpec = moldSchema.get("sprueInletOffset", {})
        self.sprueOffsetSpinBox = QDoubleSpinBox()
        self.sprueOffsetSpinBox.setRange(sprueSpec.get("min", 0.0), sprueSpec.get("max", 100.0))
        self.sprueOffsetSpinBox.setValue(
            self.currentConfig.get("mold", {}).get("sprueInletOffset", sprueSpec.get("default", 2.0)))
        self.sprueOffsetSpinBox.setSuffix(f" {sprueSpec.get('unit', 'mm')}")
        self.sprueOffsetSpinBox.setDecimals(1)
        paramLayout.addRow("浇口偏移:", self.sprueOffsetSpinBox)
        layout.addLayout(paramLayout)

        self.addGatingButton = QPushButton("添加浇道")
        self.addGatingButton.setEnabled(False)
        self.addGatingButton.clicked.connect(self.onAddGatingClick)
        layout.addWidget(self.addGatingButton)

        self.cavityVolumeGroup = QGroupBox("模腔体积估算")
        self.cavityVolumeGroup.setVisible(False)
        cavityLayout = QFormLayout()
        cavityLayout.setContentsMargins(8, 8, 8, 8)
        cavityLayout.setSpacing(6)

        valueFont = QFont()
        valueFont.setFamily("Courier New")
        valueFont.setPointSize(9)

        self.cavityVolumeLabel = QLabel("—")
        self.cavityVolumeLabel.setFont(valueFont)
        self.cavityVolumeLabel.setStyleSheet("color: #333333;")
        cavityLayout.addRow("模腔体积:", self.cavityVolumeLabel)

        self.cavityVolumeMassLabel = QLabel("—")
        self.cavityVolumeMassLabel.setFont(valueFont)
        self.cavityVolumeMassLabel.setStyleSheet("color: #555555;")
        cavityLayout.addRow("预估质量:", self.cavityVolumeMassLabel)

        hintLabel = QLabel("铸件 + 浇道 + 冒口之和")
        hintLabel.setStyleSheet("color: #888888; font-size: 8pt;")
        cavityLayout.addRow("来源:", hintLabel)

        self.cavityVolumeGroup.setLayout(cavityLayout)
        layout.addWidget(self.cavityVolumeGroup)

        gatingGroup.setLayout(layout)
        return gatingGroup

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

        self.exportMoldButton = QPushButton("导出模具 STL...")
        self.exportMoldButton.setEnabled(False)
        self.exportMoldButton.clicked.connect(self.onExportMoldClick)
        self.exportMoldButton.setStyleSheet(
            "QPushButton { background-color: #107c10; }"
            "QPushButton:hover { background-color: #0e6b0e; }"
            "QPushButton:pressed { background-color: #0a550a; }"
            "QPushButton:disabled { background-color: #cccccc; color: #888888; }"
        )
        layout.addWidget(self.exportMoldButton)

        self.moldBoundsGroup = QGroupBox("模具包围盒规模")
        self.moldBoundsGroup.setVisible(False)
        boundsLayout = QFormLayout()
        boundsLayout.setContentsMargins(8, 8, 8, 8)
        boundsLayout.setSpacing(4)

        boundsLabelFont = QFont()
        boundsLabelFont.setFamily("Courier New")
        boundsLabelFont.setPointSize(9)

        self.boundsXLabel = QLabel("—")
        self.boundsYLabel = QLabel("—")
        self.boundsZLabel = QLabel("—")
        for lbl in (self.boundsXLabel, self.boundsYLabel, self.boundsZLabel):
            lbl.setFont(boundsLabelFont)
            lbl.setStyleSheet("color: #333333;")

        boundsLayout.addRow("X:", self.boundsXLabel)
        boundsLayout.addRow("Y:", self.boundsYLabel)
        boundsLayout.addRow("Z:", self.boundsZLabel)
        self.moldBoundsGroup.setLayout(boundsLayout)
        layout.addWidget(self.moldBoundsGroup)

        group.setLayout(layout)
        return group

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

    def onLoadSTLClick(self):
        filePath, _ = QFileDialog.getOpenFileName(self, "打开STL文件", "", "STL Files (*.stl);;All Files (*)")
        if filePath:
            self.statusMessageChanged.emit(f"加载中: {Path(filePath).name}...")
            self.intentLoadModel.emit(filePath)

    def onGenerateMoldClick(self):
        self.generateMoldButton.setEnabled(False)
        self.addGatingButton.setEnabled(False)
        self.exportMoldButton.setEnabled(False)
        self.moldBoundsGroup.setVisible(False)
        self.statusMessageChanged.emit("正在生成模具...")
        config = {
            "boundingBoxOffset": self.boundingBoxOffsetSpinBox.value(),
            "booleanEngine": None
        }
        self.intentGenerateMold.emit(config)

    def onAddGatingClick(self):
        self.addGatingButton.setEnabled(False)
        self.generateMoldButton.setEnabled(False)
        self.cavityVolumeGroup.setVisible(False)
        self.statusMessageChanged.emit("正在添加浇道...")
        config = {
            "runnerDiameter": self.runnerDiameterSpinBox.value(),
            "sprueInletOffset": self.sprueOffsetSpinBox.value(),
            "boundingBoxOffset": self.boundingBoxOffsetSpinBox.value()
        }
        self.intentAddGating.emit(config)

    def onExportMoldClick(self):
        filePath, _ = QFileDialog.getSaveFileName(
            self, "导出模具STL", "mold.stl", "STL Files (*.stl);;All Files (*)"
        )
        if filePath:
            self.intentExportMold.emit(filePath)

    def onOptimizeOrientationClick(self):
        self.optimizeOrientationButton.setEnabled(False)
        optType = "milling" if self.millingRadio.isChecked() else "printing"
        self.statusMessageChanged.emit(f"正在执行方向优化({optType})...")
        self.intentOptimizeOrientation.emit({"optimizationType": optType})

    def onAdjustStructureClick(self):
        self.adjustStructureButton.setEnabled(False)
        val = self.surfaceOffsetSpinBox.value()
        self.statusMessageChanged.emit(f"正在执行表面偏移({val} mm)...")
        self.intentAdjustStructure.emit({"offsetValue": val})

    def onModelLoadedSuccess(self):
        self.statusFlags["loaded"] = True
        self.statusFlags["molded"] = False
        self.statusFlags["gated"] = False
        self.moldBoundsGroup.setVisible(False)
        self.cavityVolumeGroup.setVisible(False)
        self.exportMoldButton.setEnabled(False)
        self.generateMoldButton.setEnabled(True)
        self.addGatingButton.setEnabled(True)
        self.optimizeOrientationButton.setEnabled(False)
        self.adjustStructureButton.setEnabled(False)
        self.statusMessageChanged.emit("模型已加载，可以添加浇道或生成模具")

    def onMoldGeneratedSuccess(self):
        self.statusFlags["molded"] = True
        self.generateMoldButton.setEnabled(True)
        self.addGatingButton.setEnabled(True)
        self.optimizeOrientationButton.setEnabled(True)
        self.adjustStructureButton.setEnabled(True)
        self.exportMoldButton.setEnabled(True)
        self.statusMessageChanged.emit(self._buildStatusText())
        QMessageBox.information(self, "成功", "模具生成完成")

    def onGatingAddedSuccess(self):
        self.statusFlags["gated"] = True
        self.statusFlags["molded"] = False
        self.addGatingButton.setEnabled(True)
        self.generateMoldButton.setEnabled(True)
        self.statusMessageChanged.emit(self._buildStatusText())
        QMessageBox.information(self, "成功", "浇道添加完成，请继续生成模具")

    def onOrientationOptimizedSuccess(self, optType: str):
        self.statusFlags["oriented"] = True
        self.optimizeOrientationButton.setEnabled(True)
        self.statusMessageChanged.emit(self._buildStatusText())
        QMessageBox.information(self, "提示", f"方向优化已完成 ({optType})")

    def onStructureAdjustedSuccess(self, offsetVal: float):
        self.statusFlags["adjusted"] = True
        self.adjustStructureButton.setEnabled(True)
        self.statusMessageChanged.emit(self._buildStatusText())
        QMessageBox.information(self, "提示", f"表面偏移已完成 ({offsetVal} mm)")

    def onProcessError(self, title: str, errMsg: str):
        if self.statusFlags["loaded"]:
            self.generateMoldButton.setEnabled(True)
            self.addGatingButton.setEnabled(True)
        if self.statusFlags["molded"]:
            self.optimizeOrientationButton.setEnabled(True)
            self.adjustStructureButton.setEnabled(True)
            self.exportMoldButton.setEnabled(True)
        self.statusMessageChanged.emit(f"✗ {title}")
        QMessageBox.critical(self, title, f"{title}:\n{errMsg}")

    @pyqtSlot(dict)
    def onMoldBoundsReady(self, boundsData: dict):
        xText = f"[{boundsData['xMin']:.2f}, {boundsData['xMax']:.2f}]  Δ {boundsData['xSize']:.2f} mm"
        yText = f"[{boundsData['yMin']:.2f}, {boundsData['yMax']:.2f}]  Δ {boundsData['ySize']:.2f} mm"
        zText = f"[{boundsData['zMin']:.2f}, {boundsData['zMax']:.2f}]  Δ {boundsData['zSize']:.2f} mm"
        self.boundsXLabel.setText(xText)
        self.boundsYLabel.setText(yText)
        self.boundsZLabel.setText(zText)
        self.moldBoundsGroup.setVisible(True)

    @pyqtSlot(float)
    def onCavityVolumeReady(self, totalVolume: float):
        volumeCm3 = totalVolume / 1000.0
        self.cavityVolumeLabel.setText(f"{totalVolume:.2f} mm³  ({volumeCm3:.4f} cm³)")
        densityGaIn = 6.440
        massG = volumeCm3 * densityGaIn
        self.cavityVolumeMassLabel.setText(f"{massG:.3f} g")
        self.cavityVolumeGroup.setVisible(True)

    def loadConfiguration(self, configDict):
        moldConfig = configDict.get("mold") or {}
        if "mold" not in self.currentConfig:
            self.currentConfig["mold"] = {}
        self.currentConfig["mold"].update(moldConfig)
        self.boundingBoxOffsetSpinBox.setValue(moldConfig.get("boundingBoxOffset", 2.0))
        self.runnerDiameterSpinBox.setValue(moldConfig.get("runnerDiameter", 6.0))
        self.sprueOffsetSpinBox.setValue(moldConfig.get("sprueInletOffset", 2.0))

    def getMoldConfigurationSection(self):
        return {
            "boundingBoxOffset": self.boundingBoxOffsetSpinBox.value(),
            "runnerDiameter": self.runnerDiameterSpinBox.value(),
            "sprueInletOffset": self.sprueOffsetSpinBox.value(),
        }

    def _buildStatusText(self) -> str:
        parts = []
        if self.statusFlags["molded"]: parts.append("✓ 模具已生成")
        if self.statusFlags["gated"]: parts.append("✓ 浇道已添加")
        if self.statusFlags["oriented"]: parts.append("✓ 方向已优化")
        if self.statusFlags["adjusted"]: parts.append("✓ 表面已偏移")
        return " | ".join(parts)

    def getStylesheet(self) -> str:
        return """
        QGroupBox { border: 1px solid #cccccc; border-radius: 5px; margin-top: 10px; padding-top: 10px; font-weight: bold; }
        QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px 0 5px; }
        QPushButton { background-color: #0078d4; color: white; border: none; border-radius: 4px; padding: 8px 16px; font-weight: bold; }
        QPushButton:hover { background-color: #1084d7; }
        QPushButton:pressed { background-color: #005a9e; }
        QPushButton:disabled { background-color: #cccccc; color: #888888; }
        """
