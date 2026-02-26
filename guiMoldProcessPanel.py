import os
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QFormLayout, QPushButton,
    QLabel, QDoubleSpinBox, QGroupBox, QRadioButton, QButtonGroup, QFileDialog
)


class MoldProcessPanel(QWidget):
    requestLoadModel = pyqtSignal(str)
    requestMoldGenerate = pyqtSignal(dict)
    requestAddGating = pyqtSignal(dict)
    requestOrientationOptimize = pyqtSignal(dict)
    requestAdjustStructure = pyqtSignal(dict)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.initUI()

    def initUI(self):
        mainLayout = QVBoxLayout()
        mainLayout.setSpacing(15)
        mainLayout.setContentsMargins(10, 10, 10, 10)

        titleLabel = QLabel("模具生成向导")
        titleFont = QFont()
        titleFont.setPointSize(12)
        titleFont.setBold(True)
        titleLabel.setFont(titleFont)

        mainLayout.addWidget(titleLabel)
        mainLayout.addWidget(self.createLoadModelGroup())
        mainLayout.addWidget(self.createMoldGenerationGroup())
        mainLayout.addWidget(self.createAddGatingGroup())
        mainLayout.addWidget(self.createOrientationGroup())
        mainLayout.addWidget(self.createSurfaceOffsetGroup())
        mainLayout.addWidget(self.createStatusDisplay())
        mainLayout.addStretch()

        self.setLayout(mainLayout)

    def createLoadModelGroup(self) -> QGroupBox:
        group = QGroupBox("1. 输入模型")
        layout = QVBoxLayout()
        self.loadSTLButton = QPushButton("加载零件 STL")
        self.loadSTLButton.clicked.connect(self.onLoadSTL)
        layout.addWidget(self.loadSTLButton)
        group.setLayout(layout)
        return group

    def createMoldGenerationGroup(self) -> QGroupBox:
        group = QGroupBox("2. 生成模具壳")
        layout = QVBoxLayout()
        paramLayout = QFormLayout()

        self.boundingBoxOffsetSpinBox = QDoubleSpinBox()
        self.boundingBoxOffsetSpinBox.setRange(0.1, 50.0)
        self.boundingBoxOffsetSpinBox.setValue(2.0)
        paramLayout.addRow("外壳偏置(mm):", self.boundingBoxOffsetSpinBox)
        layout.addLayout(paramLayout)

        self.generateMoldButton = QPushButton("生成外壳")
        self.generateMoldButton.setEnabled(False)
        self.generateMoldButton.clicked.connect(self.onGenerateMold)
        layout.addWidget(self.generateMoldButton)
        group.setLayout(layout)
        return group

    def createAddGatingGroup(self) -> QGroupBox:
        group = QGroupBox("3. 浇注系统")
        layout = QVBoxLayout()
        paramLayout = QFormLayout()

        self.fillTimeSpinBox = QDoubleSpinBox()
        self.fillTimeSpinBox.setRange(0.1, 100.0)
        self.fillTimeSpinBox.setValue(5.0)
        paramLayout.addRow("填充时间(s):", self.fillTimeSpinBox)

        self.sprueOffsetSpinBox = QDoubleSpinBox()
        self.sprueOffsetSpinBox.setRange(0.0, 100.0)
        self.sprueOffsetSpinBox.setValue(10.0)
        paramLayout.addRow("注入口偏移(mm):", self.sprueOffsetSpinBox)

        layout.addLayout(paramLayout)
        self.addGatingButton = QPushButton("添加浇注口")
        self.addGatingButton.setEnabled(False)
        self.addGatingButton.clicked.connect(self.onAddGating)
        layout.addWidget(self.addGatingButton)
        group.setLayout(layout)
        return group

    def createOrientationGroup(self) -> QGroupBox:
        group = QGroupBox("4. 摆放优化")
        layout = QVBoxLayout()
        self.orientationButtonGroup = QButtonGroup(self)
        self.millingRadio = QRadioButton("倾向于机加工可行性")
        self.printingRadio = QRadioButton("倾向于减少打印支撑")
        self.millingRadio.setChecked(True)
        self.orientationButtonGroup.addButton(self.millingRadio, 0)
        self.orientationButtonGroup.addButton(self.printingRadio, 1)
        layout.addWidget(self.millingRadio)
        layout.addWidget(self.printingRadio)

        self.optimizeOrientationButton = QPushButton("执行摆放优化")
        self.optimizeOrientationButton.setEnabled(False)
        self.optimizeOrientationButton.clicked.connect(self.onOptimizeOrientation)
        layout.addWidget(self.optimizeOrientationButton)
        group.setLayout(layout)
        return group

    def createSurfaceOffsetGroup(self) -> QGroupBox:
        group = QGroupBox("5. 内腔顶面支撑补偿")
        layout = QVBoxLayout()
        hLayout = QHBoxLayout()
        hLayout.addWidget(QLabel("补偿偏移(mm):"))
        self.surfaceOffsetSpinBox = QDoubleSpinBox()
        self.surfaceOffsetSpinBox.setValue(0.5)
        hLayout.addWidget(self.surfaceOffsetSpinBox)
        layout.addLayout(hLayout)

        self.adjustStructureButton = QPushButton("自适应修改支撑结构")
        self.adjustStructureButton.setEnabled(False)
        self.adjustStructureButton.clicked.connect(self.onAdjustStructure)
        layout.addWidget(self.adjustStructureButton)
        group.setLayout(layout)
        return group

    def createStatusDisplay(self) -> QGroupBox:
        group = QGroupBox("状态信息")
        layout = QVBoxLayout()
        self.statusLabel = QLabel("等待加载模型...")
        self.statusLabel.setWordWrap(True)
        layout.addWidget(self.statusLabel)
        group.setLayout(layout)
        return group

    # === 事件触发，仅组装面板Overrides向上派发 ===
    def onLoadSTL(self):
        filePath, _ = QFileDialog.getOpenFileName(self, "选择输入模型", "", "STL Files (*.stl)")
        if filePath:
            self.statusLabel.setText(f"已选择文件: {os.path.basename(filePath)}")
            self.generateMoldButton.setEnabled(True)
            self.requestLoadModel.emit(filePath)

    def getMoldOverrides(self) -> dict:
        return {
            "boundingBoxOffset": self.boundingBoxOffsetSpinBox.value(),
            "targetFillTime": self.fillTimeSpinBox.value(),
            "sprueInletOffset": self.sprueOffsetSpinBox.value(),
            "optimizationType": "milling" if self.millingRadio.isChecked() else "printing",
            "supportOffset": self.surfaceOffsetSpinBox.value()
        }

    def onGenerateMold(self):
        self.statusLabel.setText("正在生成模具...")
        self.requestMoldGenerate.emit(self.getMoldOverrides())

    def onAddGating(self):
        self.statusLabel.setText("正在添加浇注系统...")
        self.requestAddGating.emit(self.getMoldOverrides())

    def onOptimizeOrientation(self):
        self.statusLabel.setText("正在优化摆放姿态...")
        self.requestOrientationOptimize.emit(self.getMoldOverrides())

    def onAdjustStructure(self):
        self.statusLabel.setText("正在处理内腔支撑补偿...")
        self.requestAdjustStructure.emit(self.getMoldOverrides())

    def updateStatus(self, text: str, enableNext: bool = False):
        self.statusLabel.setText(text)
        if enableNext:
            self.addGatingButton.setEnabled(True)
            self.optimizeOrientationButton.setEnabled(True)
            self.adjustStructureButton.setEnabled(True)
