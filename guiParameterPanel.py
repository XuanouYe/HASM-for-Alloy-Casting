# guiParameterPanel.py
import json
from typing import Dict, Callable, Optional
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QFormLayout, QLabel, QTabWidget,
    QScrollArea, QCheckBox, QSpinBox, QDoubleSpinBox, QComboBox, QLineEdit
)
from controlConfig import parameterSchema, ConfigManager

class ProcessParameterPanel(QWidget):
    parametersChanged = pyqtSignal(dict)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.parameters = {}
        self.parameterControls = {}
        self.changeCallbacks = []
        self.configManager = ConfigManager() # 只保留作为 schema 的默认值来源
        self.initUI()
        self.loadDefaultConfig()

    def initUI(self):
        mainLayout = QVBoxLayout()
        mainLayout.setSpacing(10)
        mainLayout.setContentsMargins(10, 10, 10, 10)

        titleLabel = QLabel("工艺与配置参数")
        titleFont = QFont()
        titleFont.setPointSize(12)
        titleFont.setBold(True)
        titleLabel.setFont(titleFont)
        mainLayout.addWidget(titleLabel)

        mainLayout.addWidget(self.createParameterTabs())
        mainLayout.addStretch()

        self.setLayout(mainLayout)

    def createParameterTabs(self) -> QTabWidget:
        tabWidget = QTabWidget()
        if "additive" in parameterSchema:
            tabWidget.addTab(self.createParameterTab("additive"), "增材(FDM)")
        if "casting" in parameterSchema:
            tabWidget.addTab(self.createParameterTab("casting"), "浇铸")
        if "subtractive" in parameterSchema:
            tabWidget.addTab(self.createParameterTab("subtractive"), "减材(CNC)")
        return tabWidget

    def createParameterTab(self, section: str) -> QWidget:
        widget = QWidget()
        scrollArea = QScrollArea()
        scrollArea.setWidgetResizable(True)

        contentWidget = QWidget()
        layout = QFormLayout()

        schema = parameterSchema.get(section, {})
        for paramName, paramSpec in schema.items():
            control = self.createControlForParameter(section, paramName, paramSpec)
            if control:
                label = QLabel(f"{paramSpec.get('description', paramName)}:")
                layout.addRow(label, control)

        contentWidget.setLayout(layout)
        scrollArea.setWidget(contentWidget)

        mainLayout = QVBoxLayout()
        mainLayout.addWidget(scrollArea)
        widget.setLayout(mainLayout)
        return widget

    def createControlForParameter(self, section: str, paramName: str, paramSpec: Dict) -> Optional[QWidget]:
        paramType = paramSpec.get('type')
        defaultValue = paramSpec.get('default')
        unit = paramSpec.get('unit', '')
        controlKey = f"{section}.{paramName}"
        control = None

        if paramType == bool:
            control = QCheckBox()
            control.setChecked(defaultValue if defaultValue is not None else False)
            control.stateChanged.connect(self.onParameterChanged)
        elif paramType == int:
            control = QSpinBox()
            control.setRange(paramSpec.get('min', 0), paramSpec.get('max', 9999))
            control.setValue(defaultValue if defaultValue is not None else 0)
            if unit: control.setSuffix(f" {unit}")
            control.valueChanged.connect(self.onParameterChanged)
        elif paramType == float:
            control = QDoubleSpinBox()
            control.setRange(paramSpec.get('min', 0.0), paramSpec.get('max', 9999.0))
            control.setDecimals(2)
            control.setValue(defaultValue if defaultValue is not None else 0.0)
            if unit: control.setSuffix(f" {unit}")
            control.valueChanged.connect(self.onParameterChanged)
        elif paramType == str:
            options = paramSpec.get('options')
            if options:
                control = QComboBox()
                control.addItems(options)
                if defaultValue in options:
                    control.setCurrentText(defaultValue)
                control.currentTextChanged.connect(self.onParameterChanged)
            else:
                control = QLineEdit()
                control.setText(defaultValue if defaultValue is not None else "")
                control.textChanged.connect(self.onParameterChanged)
        elif paramType == list:
            control = QLineEdit()
            control.setText(json.dumps(defaultValue) if defaultValue is not None else "[]")
            control.textChanged.connect(self.onParameterChanged)

        if control:
            self.parameterControls[controlKey] = control
        return control

    def onParameterChanged(self):
        self.updateCurrentConfig()
        self.parametersChanged.emit(self.parameters)

    def loadDefaultConfig(self):
        defaultConfig = self.configManager.getDefaultConfig()
        self.loadConfiguration(defaultConfig)

    def updateCurrentConfig(self):
        self.parameters = {"additive": {}, "casting": {}, "subtractive": {}, "mold": {}}
        for controlKey, control in self.parameterControls.items():
            section, paramName = controlKey.split('.', 1)
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

    def loadConfiguration(self, configDict: Dict):
        for section in ["additive", "casting", "subtractive", "mold"]:
            if section not in configDict: continue
            for paramName, value in configDict[section].items():
                controlKey = f"{section}.{paramName}"
                control = self.parameterControls.get(controlKey)
                if not control: continue

                if isinstance(control, QCheckBox):
                    control.setChecked(bool(value))
                elif isinstance(control, (QSpinBox, QDoubleSpinBox)):
                    control.setValue(value)
                elif isinstance(control, QComboBox):
                    control.setCurrentText(str(value))
                elif isinstance(control, QLineEdit):
                    control.setText(json.dumps(value) if isinstance(value, (list, dict)) else str(value))
        self.updateCurrentConfig()

    def getConfiguration(self) -> Dict:
        self.updateCurrentConfig()
        return self.parameters.copy()
