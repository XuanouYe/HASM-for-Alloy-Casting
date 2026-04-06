import json
from typing import Dict, Callable, Optional
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QFormLayout,
    QLabel, QGroupBox, QTabWidget, QScrollArea,
    QCheckBox, QSpinBox, QDoubleSpinBox, QComboBox, QLineEdit,
)
from controlConfig import parameterSchema, ConfigManager


class ProcessParameterPanel(QWidget):
    parametersChanged = pyqtSignal(dict)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.parameters = {}
        self.parameterControls = {}
        self.changeCallbacks = []
        self.configFilePath = None
        self.configManager = ConfigManager()
        self.initUI()
        self.loadDefaultConfig()

    def initUI(self):
        mainLayout = QVBoxLayout()
        mainLayout.setSpacing(10)
        mainLayout.setContentsMargins(10, 10, 10, 10)
        titleLabel = QLabel("工艺参数配置")
        titleFont = QFont()
        titleFont.setPointSize(12)
        titleFont.setBold(True)
        titleLabel.setFont(titleFont)
        mainLayout.addWidget(titleLabel)
        tabWidget = self.createParameterTabs()
        mainLayout.addWidget(tabWidget)
        self.setLayout(mainLayout)
        self.setStyleSheet(self.getStylesheet())

    def createParameterTabs(self) -> QTabWidget:
        tabWidget = QTabWidget()

        if "additive" in parameterSchema:
            tabWidget.addTab(self.createAdditiveParametersTab(), "增材工艺")

        if "casting" in parameterSchema:
            tabWidget.addTab(self.createCastingParametersTab(), "铸造工艺")

        if "subtractive" in parameterSchema:
            tabWidget.addTab(self.createSubtractiveParametersTab(), "减材加工")

        return tabWidget

    def createAdditiveParametersTab(self) -> QWidget:
        widget = QWidget()
        mainLayout = QVBoxLayout(widget)
        mainLayout.setContentsMargins(0, 0, 0, 0)
        scrollArea = QScrollArea()
        scrollArea.setWidgetResizable(True)
        scrollArea.setFrameShape(QScrollArea.NoFrame)
        contentWidget = QWidget()
        layout = QFormLayout(contentWidget)
        layout.setFieldGrowthPolicy(QFormLayout.ExpandingFieldsGrow)
        layout.setLabelAlignment(Qt.AlignRight)
        additiveSchema = parameterSchema.get("additive", {})
        for paramName, paramSpec in additiveSchema.items():
            control = self.createControlForParameter("additive", paramName, paramSpec)
            if control:
                label = QLabel(f"{paramSpec.get('description', paramName)}:")
                layout.addRow(label, control)
        axisLimitsGroup = self.createAxisLimitsGroup("additive")
        layout.addRow(QLabel(""), axisLimitsGroup)
        scrollArea.setWidget(contentWidget)
        mainLayout.addWidget(scrollArea)
        return widget

    def createCastingParametersTab(self) -> QWidget:
        widget = QWidget()
        mainLayout = QVBoxLayout(widget)
        mainLayout.setContentsMargins(0, 0, 0, 0)
        scrollArea = QScrollArea()
        scrollArea.setWidgetResizable(True)
        scrollArea.setFrameShape(QScrollArea.NoFrame)
        contentWidget = QWidget()
        layout = QFormLayout(contentWidget)
        layout.setFieldGrowthPolicy(QFormLayout.ExpandingFieldsGrow)
        layout.setLabelAlignment(Qt.AlignRight)
        castingSchema = parameterSchema.get("casting", {})
        for paramName, paramSpec in castingSchema.items():
            control = self.createControlForParameter("casting", paramName, paramSpec)
            if control:
                label = QLabel(f"{paramSpec.get('description', paramName)}:")
                layout.addRow(label, control)
        scrollArea.setWidget(contentWidget)
        mainLayout.addWidget(scrollArea)
        return widget

    def createSubtractiveParametersTab(self) -> QWidget:
        widget = QWidget()
        mainLayout = QVBoxLayout(widget)
        mainLayout.setContentsMargins(0, 0, 0, 0)
        scrollArea = QScrollArea()
        scrollArea.setWidgetResizable(True)
        scrollArea.setFrameShape(QScrollArea.NoFrame)
        contentWidget = QWidget()
        contentLayout = QVBoxLayout(contentWidget)
        contentLayout.setContentsMargins(4, 4, 4, 4)
        contentLayout.setSpacing(6)

        stepEnableGroup = self.createStepEnableGroup()
        contentLayout.addWidget(stepEnableGroup)

        safeGroup = self.createSafeAndPerfGroup()
        contentLayout.addWidget(safeGroup)

        paramForm = QWidget()
        layout = QFormLayout(paramForm)
        layout.setFieldGrowthPolicy(QFormLayout.ExpandingFieldsGrow)
        layout.setLabelAlignment(Qt.AlignRight)
        subtractiveSchema = parameterSchema.get("subtractive", {})
        skipKeys = {"enableStep1ShellRemoval", "enableStep2RiserRemoval", "enableStep3PartFinishing", "enableStep4GateRemoval"}
        for paramName, paramSpec in subtractiveSchema.items():
            if paramName in skipKeys:
                continue
            control = self.createControlForParameter("subtractive", paramName, paramSpec)
            if control:
                label = QLabel(f"{paramSpec.get('description', paramName)}:")
                layout.addRow(label, control)
        contentLayout.addWidget(paramForm)
        contentLayout.addStretch()

        scrollArea.setWidget(contentWidget)
        mainLayout.addWidget(scrollArea)
        return widget

    def createStepEnableGroup(self) -> QGroupBox:
        groupBox = QGroupBox("加工步骤选择")
        groupLayout = QFormLayout()
        stepItems = [
            ("enableStep1ShellRemoval",  "Step 1  模壳去除"),
            ("enableStep2RiserRemoval",  "Step 2  冒口去除"),
            ("enableStep3PartFinishing", "Step 3  零件精加工"),
            ("enableStep4GateRemoval",   "Step 4  浇口去除"),
        ]
        subtractiveSchema = parameterSchema.get("subtractive", {})
        for paramName, labelText in stepItems:
            paramSpec = subtractiveSchema.get(paramName, {})
            checkBox = QCheckBox()
            checkBox.setChecked(paramSpec.get("default", True))
            checkBox.setToolTip(paramSpec.get("description", labelText))
            checkBox.stateChanged.connect(self.onParameterChanged)
            controlKey = f"subtractive.{paramName}"
            self.parameterControls[controlKey] = checkBox
            groupLayout.addRow(QLabel(f"{labelText}:"), checkBox)
        groupBox.setLayout(groupLayout)
        return groupBox

    def createSafeAndPerfGroup(self) -> QGroupBox:
        groupBox = QGroupBox("性能与安全设置")
        groupLayout = QFormLayout()
        safeItems = [
            ("sdfBackend",       "SDF后端选择"),
            ("sweptDiskCount",   "刀具Disk采样数"),
            ("sweptRingCount",   "刀具Ring采样数"),
            ("sweptSafeBuffer",  "远距快通阈值 (mm)"),
            ("sweptMaxDepth",    "Swept最大递归深度"),
            ("bottomSafeOffset", "底面安全间隙 (mm)"),
            ("stepSafeClearance","CL点安全距离 (mm)"),
            ("gateSafeClearance","浇口CL安全距离 (mm)"),
        ]
        subtractiveSchema = parameterSchema.get("subtractive", {})
        for paramName, labelText in safeItems:
            paramSpec = subtractiveSchema.get(paramName, {})
            control = self.createControlForParameter("subtractive", paramName, paramSpec)
            if control:
                groupLayout.addRow(QLabel(f"{labelText}:"), control)
        groupBox.setLayout(groupLayout)
        return groupBox

    def createControlForParameter(self, section: str, paramName: str, paramSpec: Dict) -> Optional[QWidget]:
        paramType = paramSpec.get("type")
        defaultValue = paramSpec.get("default")
        unit = paramSpec.get("unit", "")
        controlKey = f"{section}.{paramName}"
        control = None

        if paramType == bool:
            control = QCheckBox()
            control.setChecked(defaultValue if defaultValue is not None else False)
            control.stateChanged.connect(self.onParameterChanged)

        elif paramType == int:
            control = QSpinBox()
            minVal = paramSpec.get("min", 0)
            maxVal = paramSpec.get("max", 9999)
            control.setRange(minVal, maxVal)
            control.setValue(defaultValue if defaultValue is not None else minVal)
            if unit:
                control.setSuffix(f" {unit}")
            control.valueChanged.connect(self.onParameterChanged)

        elif paramType == float:
            control = QDoubleSpinBox()
            minVal = paramSpec.get("min", 0.0)
            maxVal = paramSpec.get("max", 9999.0)
            control.setRange(minVal, maxVal)
            control.setDecimals(2)
            control.setSingleStep(0.1)
            control.setValue(defaultValue if defaultValue is not None else minVal)
            if unit:
                control.setSuffix(f" {unit}")
            control.valueChanged.connect(self.onParameterChanged)

        elif paramType == str:
            options = paramSpec.get("options")
            if options:
                control = QComboBox()
                control.addItems(options)
                if defaultValue and defaultValue in options:
                    control.setCurrentText(defaultValue)
                control.currentTextChanged.connect(self.onParameterChanged)
            else:
                control = QLineEdit()
                control.setText(defaultValue if defaultValue is not None else "")
                control.textChanged.connect(self.onParameterChanged)

        elif paramType == list:
            control = QLineEdit()
            control.setText(json.dumps(defaultValue if defaultValue is not None else []))
            control.textChanged.connect(self.onParameterChanged)

        if control:
            self.parameterControls[controlKey] = control
            control.setToolTip(paramSpec.get("description", ""))

        return control

    def createAxisLimitsGroup(self, section: str) -> QWidget:
        groupBox = QGroupBox("Axis Limits")
        groupLayout = QFormLayout()

        schema = parameterSchema.get(section, {}).get("axisLimits", {}).get("default", {})
        defaultLimits = {
            "X": schema.get("X", [-100.0, 100.0]),
            "Y": schema.get("Y", [-100.0, 100.0]),
            "Z": schema.get("Z", [0.0, 100.0])
        }

        axisNames = ["X", "Y", "Z"]
        for axis in axisNames:
            minSpinBox = QDoubleSpinBox()
            minSpinBox.setRange(-1000.0, 1000.0)
            minSpinBox.setDecimals(2)
            minSpinBox.setSingleStep(1.0)
            minSpinBox.setSuffix(" mm")

            maxSpinBox = QDoubleSpinBox()
            maxSpinBox.setRange(-1000.0, 1000.0)
            maxSpinBox.setDecimals(2)
            maxSpinBox.setSingleStep(1.0)
            maxSpinBox.setSuffix(" mm")

            if axis in defaultLimits:
                minSpinBox.setValue(float(defaultLimits[axis][0]))
                maxSpinBox.setValue(float(defaultLimits[axis][1]))

            axisLayout = QHBoxLayout()
            axisLayout.addWidget(QLabel("Min:"))
            axisLayout.addWidget(minSpinBox)
            axisLayout.addWidget(QLabel("Max:"))
            axisLayout.addWidget(maxSpinBox)

            axisContainer = QWidget()
            axisContainer.setLayout(axisLayout)

            groupLayout.addRow(QLabel(f"{axis}:"), axisContainer)

            minControlKey = f"{section}.axisLimits.{axis}.min"
            maxControlKey = f"{section}.axisLimits.{axis}.max"
            self.parameterControls[minControlKey] = minSpinBox
            self.parameterControls[maxControlKey] = maxSpinBox

            minSpinBox.valueChanged.connect(self.onParameterChanged)
            maxSpinBox.valueChanged.connect(self.onParameterChanged)

        groupBox.setLayout(groupLayout)
        return groupBox

    def onParameterChanged(self) -> None:
        self.updateCurrentConfig()
        self.parametersChanged.emit(self.parameters)
        for callback in self.changeCallbacks:
            callback(self.parameters)

    def loadDefaultConfig(self) -> None:
        if self.configManager:
            defaultConfig = self.configManager.getDefaultConfig()
            self.loadConfiguration(defaultConfig)

    def updateCurrentConfig(self) -> None:
        self.parameters = {
            "additive": {},
            "casting": {},
            "subtractive": {},
            "mold": {}
        }

        for controlKey, control in self.parameterControls.items():
            if ".axisLimits." in controlKey:
                parts = controlKey.split(".")
                section = parts[0]
                axis = parts[2]
                minMax = parts[3]
                if section not in self.parameters:
                    continue
                if "axisLimits" not in self.parameters[section]:
                    self.parameters[section]["axisLimits"] = {}
                if axis not in self.parameters[section]["axisLimits"]:
                    self.parameters[section]["axisLimits"][axis] = [0.0, 0.0]
                value = control.value()
                if minMax == "min":
                    self.parameters[section]["axisLimits"][axis][0] = value
                elif minMax == "max":
                    self.parameters[section]["axisLimits"][axis][1] = value
                continue
            section, paramName = controlKey.split(".", 1)
            if isinstance(control, QCheckBox):
                value = control.isChecked()
            elif isinstance(control, (QSpinBox, QDoubleSpinBox)):
                value = control.value()
            elif isinstance(control, QComboBox):
                value = control.currentText()
            elif isinstance(control, QLineEdit):
                text = control.text()
                try:
                    value = json.loads(text)
                except:
                    value = text
            else:
                continue
            if section in self.parameters:
                self.parameters[section][paramName] = value

    def loadConfiguration(self, configDict: Dict) -> None:
        try:
            for section in ["additive", "casting", "subtractive", "mold"]:
                if section not in configDict:
                    continue
                sectionConfig = configDict[section]
                if not isinstance(sectionConfig, dict):
                    continue

                for paramName, value in sectionConfig.items():
                    controlKey = f"{section}.{paramName}"
                    control = self.parameterControls.get(controlKey)
                    if ".axisLimits." in controlKey:
                        parts = controlKey.split(".")
                        section = parts[0]
                        axis = parts[2]
                        minMax = parts[3]

                        if section not in self.parameters:
                            continue
                        if "axisLimits" not in self.parameters[section]:
                            self.parameters[section]["axisLimits"] = {}
                        if axis not in self.parameters[section]["axisLimits"]:
                            self.parameters[section]["axisLimits"][axis] = [0.0, 0.0]

                        value = control.value()
                        if minMax == "min":
                            self.parameters[section]["axisLimits"][axis][0] = value
                        elif minMax == "max":
                            self.parameters[section]["axisLimits"][axis][1] = value
                        continue

                    if not control:
                        continue

                    if isinstance(control, QCheckBox):
                        control.setChecked(bool(value))
                    elif isinstance(control, QSpinBox):
                        control.setValue(int(value))
                    elif isinstance(control, QDoubleSpinBox):
                        control.setValue(float(value))
                    elif isinstance(control, QComboBox):
                        control.setCurrentText(str(value))
                    elif isinstance(control, QLineEdit):
                        if isinstance(value, (list, dict)):
                            control.setText(json.dumps(value))
                        else:
                            control.setText(str(value))

            self.updateCurrentConfig()
        except Exception as e:
            print(f"加载配置时出错: {e}")

    def getConfiguration(self) -> Dict:
        self.updateCurrentConfig()
        return self.parameters.copy()

    def registerChangeCallback(self, callback: Callable) -> None:
        self.changeCallbacks.append(callback)

    def getStylesheet(self) -> str:
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