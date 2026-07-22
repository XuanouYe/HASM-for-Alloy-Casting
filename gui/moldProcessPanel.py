from pathlib import Path
from PyQt5.QtCore import pyqtSignal, pyqtSlot, QTimer, Qt
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QFormLayout,
    QPushButton, QLabel, QDoubleSpinBox, QGroupBox,
    QMessageBox, QFileDialog, QScrollArea, QSizePolicy
)
from controlConfig import ConfigManager
from gui.languageManager import languageManager, tr


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
        self.orientationDotCount = 0
        self.orientationTimer = QTimer(self)
        self.orientationTimer.setInterval(600)
        self.orientationTimer.timeout.connect(self.onOrientationTimerTick)
        self.initUi()
        languageManager.languageChanged.connect(self.retranslateUi)
        self.retranslateUi()

    def initUi(self):
        outerLayout = QVBoxLayout()
        outerLayout.setContentsMargins(0, 0, 0, 0)
        outerLayout.setSpacing(0)
        self.titleLabel = QLabel(tr("titleMoldProcess"))
        titleFont = QFont()
        titleFont.setPointSize(12)
        titleFont.setBold(True)
        self.titleLabel.setFont(titleFont)
        outerLayout.addWidget(self.titleLabel)

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

    def createLoadModelGroup(self):
        self.modelLoadingGroup = QGroupBox(tr("groupModelLoading")); group = self.modelLoadingGroup
        layout = QVBoxLayout()
        self.loadStlButton = QPushButton(tr("buttonLoadStl"))
        self.loadStlButton.clicked.connect(self.onLoadStlClick)
        layout.addWidget(self.loadStlButton)
        group.setLayout(layout)
        return group

    def createAddGatingGroup(self):
        self.addGatingGroup = QGroupBox(tr("groupAddGating")); gatingGroup = self.addGatingGroup
        layout = QVBoxLayout()
        paramLayout = QFormLayout()
        moldSchema = self.configManager.getParameterSchema("mold")

        runnerSpec = moldSchema.get("runnerDiameter", {})
        self.runnerDiameterSpinBox = QDoubleSpinBox()
        self.runnerDiameterSpinBox.setRange(runnerSpec.get("min", 1.0), runnerSpec.get("max", 100.0))
        self.runnerDiameterSpinBox.setValue(
            self.currentConfig.get("mold", {}).get("runnerDiameter", runnerSpec.get("default", 6.0))
        )
        self.runnerDiameterSpinBox.setSuffix(f" {runnerSpec.get('unit', 'mm')}")
        self.runnerDiameterSpinBox.setDecimals(2)
        self.runnerDiameterSpinBox.setSingleStep(0.5)
        self.runnerDiameterLabel = QLabel(tr("labelRunnerDiameter")); paramLayout.addRow(self.runnerDiameterLabel, self.runnerDiameterSpinBox)

        sprueSpec = moldSchema.get("sprueInletOffset", {})
        self.sprueOffsetSpinBox = QDoubleSpinBox()
        self.sprueOffsetSpinBox.setRange(sprueSpec.get("min", 0.0), sprueSpec.get("max", 100.0))
        self.sprueOffsetSpinBox.setValue(
            self.currentConfig.get("mold", {}).get("sprueInletOffset", sprueSpec.get("default", 2.0))
        )
        self.sprueOffsetSpinBox.setSuffix(f" {sprueSpec.get('unit', 'mm')}")
        self.sprueOffsetSpinBox.setDecimals(1)
        self.sprueOffsetLabel = QLabel(tr("labelSprueOffset")); paramLayout.addRow(self.sprueOffsetLabel, self.sprueOffsetSpinBox)
        layout.addLayout(paramLayout)

        self.addGatingButton = QPushButton(tr("buttonAddGating"))
        self.addGatingButton.setEnabled(False)
        self.addGatingButton.clicked.connect(self.onAddGatingClick)
        layout.addWidget(self.addGatingButton)

        self.cavityVolumeGroup = QGroupBox(tr("groupCavityVolume"))
        self.cavityVolumeGroup.setVisible(False)
        cavityLayout = QFormLayout()
        cavityLayout.setContentsMargins(8, 8, 8, 8)
        cavityLayout.setSpacing(6)

        valueFont = QFont()
        valueFont.setFamily("Courier New")
        valueFont.setPointSize(9)

        self.cavityVolumeLabel = QLabel("—")
        self.cavityVolumeLabel.setFont(valueFont)
        # 修复1: 移除导致文字隐形的深色主题控制
        self.cavityVolumeLabel.setTextFormat(Qt.PlainText)
        self.cavityVolumeLabel.setWordWrap(True)
        self.cavityVolumeLabel.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        self.cavityVolumeTitleLabel = QLabel(tr("labelCavityVolume")); cavityLayout.addRow(self.cavityVolumeTitleLabel, self.cavityVolumeLabel)

        self.cavityVolumeMassLabel = QLabel("—")
        self.cavityVolumeMassLabel.setFont(valueFont)
        self.cavityVolumeMassLabel.setTextFormat(Qt.PlainText)
        self.cavityVolumeMassLabel.setWordWrap(True)
        self.cavityVolumeMassLabel.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        self.cavityVolumeMassTitleLabel = QLabel(tr("labelEstimatedMass")); cavityLayout.addRow(self.cavityVolumeMassTitleLabel, self.cavityVolumeMassLabel)

        self.cavityVolumeGroup.setLayout(cavityLayout)
        layout.addWidget(self.cavityVolumeGroup)
        gatingGroup.setLayout(layout)
        return gatingGroup

    def createMoldGenerationGroup(self):
        self.moldGenerationGroup = QGroupBox(tr("groupMoldGeneration")); group = self.moldGenerationGroup
        layout = QVBoxLayout()
        paramLayout = QFormLayout()
        moldSchema = self.configManager.getParameterSchema("mold")
        bboxSpec = moldSchema.get("boundingBoxOffset", {})

        self.boundingBoxOffsetSpinBox = QDoubleSpinBox()
        self.boundingBoxOffsetSpinBox.setRange(bboxSpec.get("min", 0.1), bboxSpec.get("max", 50.0))
        self.boundingBoxOffsetSpinBox.setValue(
            self.currentConfig.get("mold", {}).get("boundingBoxOffset", bboxSpec.get("default", 2.0))
        )
        self.boundingBoxOffsetSpinBox.setSuffix(f" {bboxSpec.get('unit', 'mm')}")
        self.boundingBoxOffsetSpinBox.setDecimals(1)
        self.boundingBoxOffsetLabel = QLabel(tr("labelBoundingBoxOffset")); paramLayout.addRow(self.boundingBoxOffsetLabel, self.boundingBoxOffsetSpinBox)
        layout.addLayout(paramLayout)

        self.generateMoldButton = QPushButton(tr("buttonGenerateMold"))
        self.generateMoldButton.setEnabled(False)
        self.generateMoldButton.clicked.connect(self.onGenerateMoldClick)
        layout.addWidget(self.generateMoldButton)

        self.exportMoldButton = QPushButton(tr("buttonExportMold"))
        self.exportMoldButton.setEnabled(False)
        self.exportMoldButton.clicked.connect(self.onExportMoldClick)
        self.exportMoldButton.setStyleSheet(
            "QPushButton { background-color: #107c10; }"
            "QPushButton:hover { background-color: #0e6b0e; }"
            "QPushButton:pressed { background-color: #0a550a; }"
            "QPushButton:disabled { background-color: #cccccc; color: #888888; }"
        )
        layout.addWidget(self.exportMoldButton)

        self.moldBoundsGroup = QGroupBox(tr("groupMoldBounds"))
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
        for currentLabel in (self.boundsXLabel, self.boundsYLabel, self.boundsZLabel):
            currentLabel.setFont(boundsLabelFont)
            # 同样移除强制颜色，适应自适应主题
            currentLabel.setTextFormat(Qt.PlainText)
            currentLabel.setWordWrap(True)
            currentLabel.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)

        boundsLayout.addRow("X:", self.boundsXLabel)
        boundsLayout.addRow("Y:", self.boundsYLabel)
        boundsLayout.addRow("Z:", self.boundsZLabel)
        self.moldBoundsGroup.setLayout(boundsLayout)
        layout.addWidget(self.moldBoundsGroup)
        group.setLayout(layout)
        return group

    def createOrientationGroup(self):
        self.printOrientationGroup = QGroupBox(tr("groupPrintOrientation")); orientationGroup = self.printOrientationGroup
        orientationLayout = QVBoxLayout()
        self.optimizeOrientationButton = QPushButton(tr("buttonOptimizeOrientation"))
        self.optimizeOrientationButton.setEnabled(False)
        self.optimizeOrientationButton.clicked.connect(self.onOptimizeOrientationClick)
        orientationLayout.addWidget(self.optimizeOrientationButton)
        orientationGroup.setLayout(orientationLayout)
        return orientationGroup

    def createSurfaceOffsetGroup(self):
        self.surfaceOffsetGroup = QGroupBox(tr("groupSurfaceOffset")); surfaceOffsetGroup = self.surfaceOffsetGroup
        surfaceOffsetLayout = QVBoxLayout()
        offsetInputLayout = QHBoxLayout()
        self.offsetValueLabel = QLabel(tr("labelOffsetValue")); offsetInputLayout.addWidget(self.offsetValueLabel)
        self.surfaceOffsetSpinBox = QDoubleSpinBox()
        self.surfaceOffsetSpinBox.setRange(-10.0, 10.0)
        self.surfaceOffsetSpinBox.setValue(0.5)
        self.surfaceOffsetSpinBox.setSuffix(" mm")
        self.surfaceOffsetSpinBox.setDecimals(2)
        offsetInputLayout.addWidget(self.surfaceOffsetSpinBox)
        surfaceOffsetLayout.addLayout(offsetInputLayout)
        self.adjustStructureButton = QPushButton(tr("buttonExecuteOffset"))
        self.adjustStructureButton.setEnabled(False)
        self.adjustStructureButton.clicked.connect(self.onAdjustStructureClick)
        surfaceOffsetLayout.addWidget(self.adjustStructureButton)
        surfaceOffsetGroup.setLayout(surfaceOffsetLayout)
        return surfaceOffsetGroup

    def retranslateUi(self):
        self.titleLabel.setText(tr("titleMoldProcess"))
        self.loadStlButton.setText(tr("buttonLoadStl"))
        self.addGatingButton.setText(tr("buttonAddGating"))
        self.generateMoldButton.setText(tr("buttonGenerateMold"))
        self.exportMoldButton.setText(tr("buttonExportMold"))
        self.optimizeOrientationButton.setText(tr("buttonOptimizeOrientation"))
        self.adjustStructureButton.setText(tr("buttonExecuteOffset"))
        for group, key in [(self.modelLoadingGroup, "groupModelLoading"), (self.addGatingGroup, "groupAddGating"), (self.cavityVolumeGroup, "groupCavityVolume"), (self.moldGenerationGroup, "groupMoldGeneration"), (self.moldBoundsGroup, "groupMoldBounds"), (self.printOrientationGroup, "groupPrintOrientation"), (self.surfaceOffsetGroup, "groupSurfaceOffset")]:
            group.setTitle(tr(key))
        self.runnerDiameterLabel.setText(tr("labelRunnerDiameter"))
        self.sprueOffsetLabel.setText(tr("labelSprueOffset"))
        self.cavityVolumeTitleLabel.setText(tr("labelCavityVolume"))
        self.cavityVolumeMassTitleLabel.setText(tr("labelEstimatedMass"))
        self.boundingBoxOffsetLabel.setText(tr("labelBoundingBoxOffset"))
        self.offsetValueLabel.setText(tr("labelOffsetValue"))

    def onLoadStlClick(self):
        filePath, _ = QFileDialog.getOpenFileName(self, tr("dialogOpenStl"), "", "STL Files (*.stl);;All Files (*)")
        if filePath:
            self.statusMessageChanged.emit(tr("statusLoadingFile", fileName=Path(filePath).name))
            self.intentLoadModel.emit(filePath)

    def onGenerateMoldClick(self):
        self.generateMoldButton.setEnabled(False)
        self.addGatingButton.setEnabled(False)
        self.exportMoldButton.setEnabled(False)
        self.moldBoundsGroup.setVisible(False)
        self.statusMessageChanged.emit(tr("statusGeneratingMold"))
        config = {
            "boundingBoxOffset": self.boundingBoxOffsetSpinBox.value(),
            "booleanEngine": None
        }
        self.intentGenerateMold.emit(config)

    def onAddGatingClick(self):
        self.addGatingButton.setEnabled(False)
        self.generateMoldButton.setEnabled(False)
        self.cavityVolumeGroup.setVisible(False)
        self.statusMessageChanged.emit(tr("statusAddingGating"))
        config = {
            "runnerDiameter": self.runnerDiameterSpinBox.value(),
            "sprueInletOffset": self.sprueOffsetSpinBox.value(),
            "boundingBoxOffset": self.boundingBoxOffsetSpinBox.value()
        }
        self.intentAddGating.emit(config)

    def onExportMoldClick(self):
        filePath, _ = QFileDialog.getSaveFileName(
            self, tr("dialogExportMoldStl"), "mold.stl", "STL Files (*.stl);;All Files (*)"
        )
        if filePath:
            self.intentExportMold.emit(filePath)

    def onOptimizeOrientationClick(self):
        self.optimizeOrientationButton.setEnabled(False)
        self.addGatingButton.setEnabled(False)
        self.generateMoldButton.setEnabled(False)
        self.orientationDotCount = 0
        self.orientationTimer.start()
        self.statusMessageChanged.emit(tr("statusOrientationCalculating", dots=""))
        self.intentOptimizeOrientation.emit({})

    def onOrientationTimerTick(self):
        self.orientationDotCount = (self.orientationDotCount + 1) % 4
        dots = "." * self.orientationDotCount
        self.statusMessageChanged.emit(tr("statusOrientationCalculating", dots=dots))

    def onAdjustStructureClick(self):
        self.adjustStructureButton.setEnabled(False)
        offsetValue = self.surfaceOffsetSpinBox.value()
        self.statusMessageChanged.emit(tr("statusExecutingOffset", offsetValue=offsetValue))
        self.intentAdjustStructure.emit({"offsetValue": offsetValue})

    def onModelLoadedSuccess(self):
        self.statusFlags["loaded"] = True
        self.statusFlags["molded"] = False
        self.statusFlags["gated"] = False
        self.moldBoundsGroup.setVisible(False)
        self.cavityVolumeGroup.setVisible(False)
        self.exportMoldButton.setEnabled(False)
        self.generateMoldButton.setEnabled(True)
        self.addGatingButton.setEnabled(True)
        self.optimizeOrientationButton.setEnabled(True)
        self.adjustStructureButton.setEnabled(False)
        self.statusMessageChanged.emit(tr("statusModelLoadedReady"))

    def onMoldGeneratedSuccess(self):
        self.statusFlags["molded"] = True
        self.generateMoldButton.setEnabled(True)
        self.addGatingButton.setEnabled(True)
        self.optimizeOrientationButton.setEnabled(False)
        self.adjustStructureButton.setEnabled(True)
        self.exportMoldButton.setEnabled(True)
        self.statusMessageChanged.emit(self.buildStatusText())
        QMessageBox.information(self, tr("success"), tr("messageMoldGenerated"))

    def onGatingAddedSuccess(self):
        self.statusFlags["gated"] = True
        self.statusFlags["molded"] = False
        self.addGatingButton.setEnabled(True)
        self.generateMoldButton.setEnabled(True)
        self.statusMessageChanged.emit(self.buildStatusText())
        QMessageBox.information(self, tr("success"), tr("messageGatingAdded"))

    def onOrientationOptimizedSuccess(self, summaryMsg: str):
        self.orientationTimer.stop()
        self.statusFlags["oriented"] = True
        self.optimizeOrientationButton.setEnabled(True)
        self.addGatingButton.setEnabled(True)
        self.generateMoldButton.setEnabled(True)
        self.statusMessageChanged.emit(self.buildStatusText())
        QMessageBox.information(self, tr("prompt"), tr("messageOrientationDone", summaryMsg=summaryMsg))

    def onStructureAdjustedSuccess(self, offsetValue: float):
        self.statusFlags["adjusted"] = True
        self.adjustStructureButton.setEnabled(True)
        self.statusMessageChanged.emit(self.buildStatusText())
        QMessageBox.information(self, tr("prompt"), tr("messageOffsetDone", offsetValue=offsetValue))

    def onProcessError(self, title: str, errMsg: str):
        self.orientationTimer.stop()
        if self.statusFlags["loaded"] and not self.statusFlags["molded"]:
            self.generateMoldButton.setEnabled(True)
            self.addGatingButton.setEnabled(True)
            self.optimizeOrientationButton.setEnabled(True)
        if self.statusFlags["molded"]:
            self.adjustStructureButton.setEnabled(True)
            self.exportMoldButton.setEnabled(True)
        self.statusMessageChanged.emit(f"✗ {title}")
        QMessageBox.critical(self, title, f"{title}:\n{errMsg}")

    @pyqtSlot(dict)
    def onMoldBoundsReady(self, boundsData: dict):
        xText = f"[{boundsData['xMin']:.2f}, {boundsData['xMax']:.2f}]  size {boundsData['xSize']:.2f} mm"
        yText = f"[{boundsData['yMin']:.2f}, {boundsData['yMax']:.2f}]  size {boundsData['ySize']:.2f} mm"
        zText = f"[{boundsData['zMin']:.2f}, {boundsData['zMax']:.2f}]  size {boundsData['zSize']:.2f} mm"
        self.boundsXLabel.setText(xText)
        self.boundsYLabel.setText(yText)
        self.boundsZLabel.setText(zText)
        self.moldBoundsGroup.setVisible(True)

    @pyqtSlot(float)
    def onCavityVolumeReady(self, totalVolume: float):
        volumeCm3 = totalVolume / 1000.0
        self.cavityVolumeLabel.setText(f"{totalVolume:.2f} mm^3  ({volumeCm3:.4f} cm^3)")
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

    def buildStatusText(self):
        parts = []
        if self.statusFlags["molded"]: parts.append(tr("statusMolded"))
        if self.statusFlags["gated"]: parts.append(tr("statusGated"))
        if self.statusFlags["oriented"]: parts.append(tr("statusOriented"))
        if self.statusFlags["adjusted"]: parts.append(tr("statusAdjusted"))
        return " | ".join(parts)

    def getStylesheet(self):
        return """
        QGroupBox { border: 1px solid #cccccc; border-radius: 5px; margin-top: 10px; padding-top: 10px; font-weight: bold; }
        QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px 0 5px; }
        QPushButton { background-color: #0078d4; color: white; border: none; border-radius: 4px; padding: 8px 16px; font-weight: bold; }
        QPushButton:hover { background-color: #1084d7; }
        QPushButton:pressed { background-color: #005a9e; }
        QPushButton:disabled { background-color: #cccccc; color: #888888; }
        """