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
        scrollArea.setWidget(contentWidget)
        mainLayout.addWidget(scrollArea)
        return widget

    def createCastingParametersTab(self) -> QWidget:
        widget = QWidget()
        layout = QFormLayout()

        castingSchema = parameterSchema.get("casting", {})
        for paramName, paramSpec in castingSchema.items():
            control = self.createControlForParameter("casting", paramName, paramSpec)
            if control:
                label = QLabel(f"{paramSpec.get('description', paramName)}:")
                layout.addRow(label, control)

        widget.setLayout(layout)
        return widget

    def createSubtractiveParametersTab(self) -> QWidget:
        widget = QWidget()
        layout = QFormLayout()

        subtractiveSchema = parameterSchema.get("subtractive", {})
        for paramName, paramSpec in subtractiveSchema.items():
            control = self.createControlForParameter("subtractive", paramName, paramSpec)
            if control:
                label = QLabel(f"{paramSpec.get('description', paramName)}:")
                layout.addRow(label, control)

        widget.setLayout(layout)
        return widget

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