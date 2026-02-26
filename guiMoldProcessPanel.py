from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPushButton,
    QLabel, QGroupBox, QCheckBox, QSpinBox, QDoubleSpinBox,
    QProgressBar, QMessageBox
)
from PyQt5.QtCore import Qt, pyqtSignal

class GuiMoldProcessPanel(QWidget):
    processRequested = pyqtSignal(dict)
    visualizationRequested = pyqtSignal(str, bool)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.initUI()

    def initUI(self):
        layout = QVBoxLayout(self)

        # 1. Process Steps Configuration
        stepsGroup = QGroupBox("Mold Process Pipeline")
        stepsLayout = QVBoxLayout()

        # Step toggles with visualization controls
        self.steps = {}
        
        # Orientation Optimization
        orientLayout = QHBoxLayout()
        self.chkOptimize = QCheckBox("1. Orientation Optimization")
        self.chkOptimize.setChecked(True)
        self.btnVisOrient = QPushButton("👁")
        self.btnVisOrient.setCheckable(True)
        self.btnVisOrient.setToolTip("Visualize orientation results")
        orientLayout.addWidget(self.chkOptimize)
        orientLayout.addStretch()
        orientLayout.addWidget(self.btnVisOrient)
        self.steps["optimize"] = {"chk": self.chkOptimize, "vis": self.btnVisOrient}

        # Mold Shell Generation
        shellLayout = QHBoxLayout()
        self.chkShell = QCheckBox("2. Mold Shell Generation")
        self.chkShell.setChecked(True)
        self.chkShell.setEnabled(False) # Mandatory
        self.btnVisShell = QPushButton("👁")
        self.btnVisShell.setCheckable(True)
        self.btnVisShell.setToolTip("Visualize mold shell")
        shellLayout.addWidget(self.chkShell)
        shellLayout.addStretch()
        shellLayout.addWidget(self.btnVisShell)
        self.steps["shell"] = {"chk": self.chkShell, "vis": self.btnVisShell}

        # Inner Surface Offset (Overhangs)
        offsetLayout = QHBoxLayout()
        self.chkOffset = QCheckBox("3. Inner Surface Offset (Overhangs)")
        self.chkOffset.setChecked(True)
        self.btnVisOffset = QPushButton("👁")
        self.btnVisOffset.setCheckable(True)
        self.btnVisOffset.setToolTip("Visualize surface offsets")
        offsetLayout.addWidget(self.chkOffset)
        offsetLayout.addStretch()
        offsetLayout.addWidget(self.btnVisOffset)
        self.steps["offset"] = {"chk": self.chkOffset, "vis": self.btnVisOffset}

        # Gating System
        gatingLayout = QHBoxLayout()
        self.chkGating = QCheckBox("4. Gating System Generation")
        self.chkGating.setChecked(True)
        self.btnVisGating = QPushButton("👁")
        self.btnVisGating.setCheckable(True)
        self.btnVisGating.setToolTip("Visualize gating system")
        gatingLayout.addWidget(self.chkGating)
        gatingLayout.addStretch()
        gatingLayout.addWidget(self.btnVisGating)
        self.steps["gating"] = {"chk": self.chkGating, "vis": self.btnVisGating}

        # Add to layout
        stepsLayout.addLayout(orientLayout)
        stepsLayout.addLayout(shellLayout)
        stepsLayout.addLayout(offsetLayout)
        stepsLayout.addLayout(gatingLayout)
        stepsGroup.setLayout(stepsLayout)
        layout.addWidget(stepsGroup)

        # 2. Key Parameters
        paramsGroup = QGroupBox("Key Parameters")
        paramsLayout = QVBoxLayout()
        
        # Bounding Box Offset
        boxLayout = QHBoxLayout()
        boxLayout.addWidget(QLabel("Shell Offset (mm):"))
        self.spinBoxOffset = QDoubleSpinBox()
        self.spinBoxOffset.setRange(1.0, 20.0)
        self.spinBoxOffset.setValue(2.0)
        boxLayout.addWidget(self.spinBoxOffset)
        paramsLayout.addLayout(boxLayout)
        
        # Support Angle
        angleLayout = QHBoxLayout()
        angleLayout.addWidget(QLabel("Support Angle (°):"))
        self.spinAngle = QDoubleSpinBox()
        self.spinAngle.setRange(0.0, 90.0)
        self.spinAngle.setValue(45.0)
        angleLayout.addWidget(self.spinAngle)
        paramsLayout.addLayout(angleLayout)

        paramsGroup.setLayout(paramsLayout)
        layout.addWidget(paramsGroup)

        # 3. Execution
        execLayout = QVBoxLayout()
        self.btnRun = QPushButton("Run Pipeline")
        self.btnRun.setMinimumHeight(40)
        self.btnRun.setStyleSheet("background-color: #2E86C1; color: white; font-weight: bold;")
        
        self.progressBar = QProgressBar()
        self.progressBar.setValue(0)
        
        self.lblStatus = QLabel("Ready")
        
        execLayout.addWidget(self.btnRun)
        execLayout.addWidget(self.progressBar)
        execLayout.addWidget(self.lblStatus)
        layout.addLayout(execLayout)
        
        layout.addStretch()

        # Connections
        self.btnRun.clicked.connect(self.onRunClicked)
        
        # Connect visualization toggles
        self.btnVisOrient.toggled.connect(lambda c: self.visualizationRequested.emit("optimize", c))
        self.btnVisShell.toggled.connect(lambda c: self.visualizationRequested.emit("shell", c))
        self.btnVisOffset.toggled.connect(lambda c: self.visualizationRequested.emit("offset", c))
        self.btnVisGating.toggled.connect(lambda c: self.visualizationRequested.emit("gating", c))

    def onRunClicked(self):
        config = {
            "optimizeOrientation": self.chkOptimize.isChecked(),
            "adjustStructure": self.chkOffset.isChecked(),
            "addGating": self.chkGating.isChecked(),
            "boundingBoxOffset": self.spinBoxOffset.value(),
            "supportAngle": self.spinAngle.value(),
        }
        self.processRequested.emit(config)

    def setProgress(self, value, text=""):
        self.progressBar.setValue(value)
        if text:
            self.lblStatus.setText(text)
            
    def setVisualizerState(self, step: str, active: bool):
        if step in self.steps:
            self.steps[step]["vis"].setChecked(active)