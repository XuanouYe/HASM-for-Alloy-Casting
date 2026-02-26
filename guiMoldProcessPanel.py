from pathlib import Path
import trimesh
import numpy as np
import pyvista as pv
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QFormLayout,
    QPushButton, QLabel, QDoubleSpinBox, QGroupBox,
    QRadioButton, QButtonGroup, QMessageBox, QFileDialog,
    QApplication
)
from guiWorkerThread import WorkerThread
from moldGenerator import MoldGenerator
from controlConfig import ConfigManager


class MoldProcessPanel(QWidget):
    moldGenerated = pyqtSignal(object)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.moldGeneratedFlag = False
        self.gatingAppliedFlag = False
        self.orientationOptimizedFlag = False
        self.structureAdjustedFlag = False
        self.currentMoldShell = None
        self.currentCastingMesh = None
        self.configManager = ConfigManager()
        self.currentConfig = self.configManager.getDefaultConfig()
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
        self.loadSTLButton.clicked.connect(self.onLoadSTL)
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
        self.boundingBoxOffsetSpinBox.setRange(
            bboxSpec.get("min", 0.1),
            bboxSpec.get("max", 50.0)
        )
        self.boundingBoxOffsetSpinBox.setValue(
            self.currentConfig.get("mold", {}).get("boundingBoxOffset", bboxSpec.get("default", 2.0))
        )
        self.boundingBoxOffsetSpinBox.setSuffix(f" {bboxSpec.get('unit', 'mm')}")
        self.boundingBoxOffsetSpinBox.setDecimals(1)
        self.boundingBoxOffsetSpinBox.valueChanged.connect(self.onBoundingBoxOffsetChanged)
        paramLayout.addRow("包围盒偏移:", self.boundingBoxOffsetSpinBox)

        layout.addLayout(paramLayout)

        self.generateMoldButton = QPushButton("生成模具")
        self.generateMoldButton.setEnabled(False)
        self.generateMoldButton.clicked.connect(self.onGenerateMold)
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
        self.fillTimeSpinBox.setRange(
            fillTimeSpec.get("min", 0.1),
            fillTimeSpec.get("max", 100.0)
        )
        self.fillTimeSpinBox.setValue(
            self.currentConfig.get("mold", {}).get("targetFillTime", fillTimeSpec.get("default", 5.0))
        )
        self.fillTimeSpinBox.setSuffix(f" {fillTimeSpec.get('unit', 's')}")
        self.fillTimeSpinBox.setDecimals(1)
        self.fillTimeSpinBox.valueChanged.connect(self.onFillTimeChanged)
        paramLayout.addRow("目标填充时间:", self.fillTimeSpinBox)

        sprueSpec = moldSchema.get("sprueInletOffset", {})
        self.sprueOffsetSpinBox = QDoubleSpinBox()
        self.sprueOffsetSpinBox.setRange(
            sprueSpec.get("min", 0.0),
            sprueSpec.get("max", 100.0)
        )
        self.sprueOffsetSpinBox.setValue(
            self.currentConfig.get("mold", {}).get("sprueInletOffset", sprueSpec.get("default", 10.0))
        )
        self.sprueOffsetSpinBox.setSuffix(f" {sprueSpec.get('unit', 'mm')}")
        self.sprueOffsetSpinBox.setDecimals(1)
        self.sprueOffsetSpinBox.valueChanged.connect(self.onSprueOffsetChanged)
        paramLayout.addRow("浇口偏移:", self.sprueOffsetSpinBox)

        layout.addLayout(paramLayout)

        self.addGatingButton = QPushButton("添加浇道")
        self.addGatingButton.setEnabled(False)
        self.addGatingButton.clicked.connect(self.onAddGating)
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
        self.optimizeOrientationButton.clicked.connect(self.onOptimizeOrientation)
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
        self.adjustStructureButton.clicked.connect(self.onAdjustStructure)
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

    def onBoundingBoxOffsetChanged(self):
        if "mold" not in self.currentConfig:
            self.currentConfig["mold"] = {}
        self.currentConfig["mold"]["boundingBoxOffset"] = self.boundingBoxOffsetSpinBox.value()

    def onFillTimeChanged(self):
        if "mold" not in self.currentConfig:
            self.currentConfig["mold"] = {}
        self.currentConfig["mold"]["targetFillTime"] = self.fillTimeSpinBox.value()

    def onSprueOffsetChanged(self):
        if "mold" not in self.currentConfig:
            self.currentConfig["mold"] = {}
        self.currentConfig["mold"]["sprueInletOffset"] = self.sprueOffsetSpinBox.value()

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

    def clearMoldViewer(self):
        mainWindow = self.window()
        viewer = getattr(mainWindow, "dualModelViewer", None)
        if viewer is None:
            return

        moldPlotter = getattr(viewer, "moldPlotter", None)
        if moldPlotter is None:
            return

        try:
            moldPlotter.clear()
            viewer.moldModel = None
            moldPlotter.render()
        except Exception:
            return

    def resetMoldProcessState(self, clearViewer=True):
        self.moldGeneratedFlag = False
        self.gatingAppliedFlag = False
        self.orientationOptimizedFlag = False
        self.structureAdjustedFlag = False
        self.currentMoldShell = None
        self.addGatingButton.setEnabled(False)
        self.optimizeOrientationButton.setEnabled(False)
        self.adjustStructureButton.setEnabled(False)

        if clearViewer:
            self.clearMoldViewer()

    def onLoadSTL(self):
        filePath, _ = QFileDialog.getOpenFileName(
            self, "打开STL文件", "", "STL Files (*.stl);;All Files (*)"
        )
        if not filePath:
            return

        self.generateMoldButton.setEnabled(False)
        self.currentCastingMesh = None
        self.resetMoldProcessState(clearViewer=True)

        self.statusLabel.setText(f"加载中: {Path(filePath).name}...")
        self.statusLabel.setStyleSheet("color: #0078d4; padding: 5px;")
        QApplication.setOverrideCursor(Qt.WaitCursor)

        try:
            mainWindow = self.window()
            ok = mainWindow.dualModelViewer.loadCastingModel(filePath)
            if not ok:
                raise RuntimeError("模型加载失败")

            castingMesh = trimesh.load(filePath)
            self.setCastingMesh(castingMesh)

            self.statusLabel.setText(f"已加载: {Path(filePath).name}")
            self.statusLabel.setStyleSheet("color: green; padding: 5px; font-weight: bold;")
        except Exception as err:
            self.statusLabel.setText("加载失败")
            self.statusLabel.setStyleSheet("color: red; padding: 5px;")
            QMessageBox.critical(self, "加载失败", f"错误: {err}")
        finally:
            QApplication.restoreOverrideCursor()

    def onGenerateMold(self):
        if self.currentCastingMesh is None:
            QMessageBox.warning(self, "警告", "请先加载铸件模型")
            return

        self.generateMoldButton.setEnabled(False)
        self.addGatingButton.setEnabled(False)
        self.statusLabel.setText("正在生成模具...")
        self.statusLabel.setStyleSheet("color: #0078d4; padding: 5px;")

        def generateTask():
            config = {
                "boundingBoxOffset": self.boundingBoxOffsetSpinBox.value(),
                "booleanEngine": None
            }
            moldGen = MoldGenerator(config=config)
            moldShell = moldGen.generateMoldShell(self.currentCastingMesh)
            return {"moldShell": moldShell}

        worker = WorkerThread(generateTask)
        worker.taskCompleted.connect(self.onMoldGenerationCompleted)
        worker.taskError.connect(self.onMoldGenerationError)
        worker.start()
        self.currentWorker = worker

    def onMoldGenerationCompleted(self, result):
        self.currentMoldShell = result["moldShell"]
        self.moldGeneratedFlag = True
        self.addGatingButton.setEnabled(True)
        self.optimizeOrientationButton.setEnabled(True)
        self.adjustStructureButton.setEnabled(True)
        self.generateMoldButton.setEnabled(True)

        self.statusLabel.setText("✓ 模具生成完成")
        self.statusLabel.setStyleSheet("color: green; padding: 5px; font-weight: bold;")
        self.moldGenerated.emit(self.currentMoldShell)
        QMessageBox.information(self, "成功", "模具生成完成")

    def onMoldGenerationError(self, errorMsg):
        self.generateMoldButton.setEnabled(True)
        self.addGatingButton.setEnabled(True)
        self.statusLabel.setText(f"✗ 模具生成失败: {errorMsg}")
        self.statusLabel.setStyleSheet("color: red; padding: 5px;")
        QMessageBox.critical(self, "错误", f"模具生成失败:\n{errorMsg}")

    def onAddGating(self):
        if self.currentCastingMesh is None:
            return

        self.addGatingButton.setEnabled(False)
        self.generateMoldButton.setEnabled(False)
        self.statusLabel.setText("正在添加浇道...")
        self.statusLabel.setStyleSheet("color: #0078d4; padding: 5px;")

        targetFillTime = self.fillTimeSpinBox.value()
        sprueOffset = self.sprueOffsetSpinBox.value()
        boundingBoxOffset = self.boundingBoxOffsetSpinBox.value()

        def gatingTask():
            config = {
                "targetFillTime": targetFillTime,
                "sprueInletOffset": sprueOffset,
                "boundingBoxOffset": boundingBoxOffset
            }
            moldGen = MoldGenerator(config=config)
            gatingMesh = moldGen.generateGating(self.currentCastingMesh, config)
            combinedMesh = trimesh.util.concatenate([self.currentCastingMesh, gatingMesh])
            return {"castingMesh": combinedMesh}

        worker = WorkerThread(gatingTask)
        worker.taskCompleted.connect(self.onGatingCompleted)
        worker.taskError.connect(self.onGatingError)
        worker.start()
        self.currentWorker = worker

    def onGatingCompleted(self, result):
        self.currentCastingMesh = result["castingMesh"]
        self.gatingAppliedFlag = True
        self.addGatingButton.setEnabled(True)
        self.generateMoldButton.setEnabled(True)

        try:
            mainWindow = self.window()
            viewer = getattr(mainWindow, "dualModelViewer", None)
            if viewer and viewer.castingPlotter:
                mesh = self.currentCastingMesh
                vertices = mesh.vertices
                faces = mesh.faces
                pvFaces = np.column_stack([np.full(len(faces), 3), faces]).flatten()
                pvMesh = pv.PolyData(vertices, pvFaces)

                viewer.castingPlotter.clear()
                viewer.castingPlotter.add_mesh(
                    pvMesh,
                    color="lightblue",
                    opacity=1.0,
                    show_edges=True,
                    edge_color="navy",
                    line_width=2.0,
                    lighting=True,
                    smooth_shading=True,
                    specular=0.5,
                    specular_power=15,
                    ambient=0.3,
                    diffuse=0.8,
                    metallic=0.2,
                    roughness=0.5
                )
                viewer.castingPlotter.enable_shadows()
                viewer.castingPlotter.enable_anti_aliasing()
                viewer.castingPlotter.render()
        except Exception as e:
            print(f"Update casting view failed: {e}")

        statusText = "✓ 浇道已添加"
        if self.moldGeneratedFlag:
            statusText += " | (需重新生成模具)"

        self.statusLabel.setText(statusText)
        self.statusLabel.setStyleSheet("color: green; padding: 5px; font-weight: bold;")
        QMessageBox.information(self, "成功", "浇道添加完成,请继续生成模具")

    def onGatingError(self, errorMsg):
        self.addGatingButton.setEnabled(True)
        self.generateMoldButton.setEnabled(True)
        self.statusLabel.setText(f"✗ 浇道添加失败: {errorMsg}")
        self.statusLabel.setStyleSheet("color: red; padding: 5px;")
        QMessageBox.critical(self, "错误", f"浇道添加失败:\n{errorMsg}")

    def onOptimizeOrientation(self):
        if self.currentMoldShell is None:
            return

        self.optimizeOrientationButton.setEnabled(False)
        optimizationType = "milling" if self.millingRadio.isChecked() else "printing"
        self.statusLabel.setText(f"正在执行方向优化({optimizationType})...")
        self.statusLabel.setStyleSheet("color: #0078d4; padding: 5px;")

        def orientationTask():
            config = {"optimizationType": optimizationType}
            moldGen = MoldGenerator(config=config)
            updatedMold = moldGen.optimizeOrientation(self.currentMoldShell, config)
            return {"moldShell": updatedMold, "type": optimizationType}

        worker = WorkerThread(orientationTask)
        worker.taskCompleted.connect(self.onOrientationCompleted)
        worker.taskError.connect(self.onOrientationError)
        worker.start()
        self.currentWorker = worker

    def onOrientationCompleted(self, result):
        self.currentMoldShell = result["moldShell"]
        self.orientationOptimizedFlag = True
        self.optimizeOrientationButton.setEnabled(True)

        statusText = "✓ 模具生成完成"
        if self.gatingAppliedFlag:
            statusText += " | ✓ 浇道已添加"
        if self.orientationOptimizedFlag:
            statusText += f" | ✓ 方向已优化({result['type']})"
        if self.structureAdjustedFlag:
            statusText += " | ✓ 表面已偏移"

        self.statusLabel.setText(statusText)
        self.statusLabel.setStyleSheet("color: green; padding: 5px; font-weight: bold;")
        self.moldGenerated.emit(self.currentMoldShell)
        QMessageBox.information(self, "提示", f"方向优化功能为占位实现\n优化类型: {result['type']}\n当前版本返回原模具")

    def onOrientationError(self, errorMsg):
        self.optimizeOrientationButton.setEnabled(True)
        self.statusLabel.setText(f"✗ 方向优化失败: {errorMsg}")
        self.statusLabel.setStyleSheet("color: red; padding: 5px;")
        QMessageBox.critical(self, "错误", f"方向优化失败:\n{errorMsg}")

    def onAdjustStructure(self):
        if self.currentMoldShell is None:
            return

        self.adjustStructureButton.setEnabled(False)
        offsetValue = self.surfaceOffsetSpinBox.value()
        self.statusLabel.setText(f"正在执行表面偏移({offsetValue} mm)...")
        self.statusLabel.setStyleSheet("color: #0078d4; padding: 5px;")

        def adjustTask():
            config = {"offsetValue": offsetValue}
            moldGen = MoldGenerator(config=config)
            updatedMold = moldGen.adjustStructure(self.currentMoldShell, config)
            return {"moldShell": updatedMold, "offset": offsetValue}

        worker = WorkerThread(adjustTask)
        worker.taskCompleted.connect(self.onAdjustCompleted)
        worker.taskError.connect(self.onAdjustError)
        worker.start()
        self.currentWorker = worker

    def onAdjustCompleted(self, result):
        self.currentMoldShell = result["moldShell"]
        self.structureAdjustedFlag = True
        self.adjustStructureButton.setEnabled(True)

        statusText = "✓ 模具生成完成"
        if self.gatingAppliedFlag:
            statusText += " | ✓ 浇道已添加"
        if self.orientationOptimizedFlag:
            statusText += " | ✓ 方向已优化"
        if self.structureAdjustedFlag:
            statusText += f" | ✓ 表面已偏移({result['offset']} mm)"

        self.statusLabel.setText(statusText)
        self.statusLabel.setStyleSheet("color: green; padding: 5px; font-weight: bold;")
        self.moldGenerated.emit(self.currentMoldShell)
        QMessageBox.information(self, "提示",
                                f"表面偏移功能为占位实现\n偏移值: {result['offset']} mm\n当前版本返回原模具")

    def onAdjustError(self, errorMsg):
        self.adjustStructureButton.setEnabled(True)
        self.statusLabel.setText(f"✗ 表面偏移失败: {errorMsg}")
        self.statusLabel.setStyleSheet("color: red; padding: 5px;")
        QMessageBox.critical(self, "错误", f"表面偏移失败:\n{errorMsg}")

    def setCastingMesh(self, mesh):
        self.generateMoldButton.setEnabled(False)
        self.currentCastingMesh = None
        self.resetMoldProcessState(clearViewer=True)
        self.currentCastingMesh = mesh
        self.generateMoldButton.setEnabled(True)
        self.addGatingButton.setEnabled(True)

        self.statusLabel.setText("铸件模型已加载,可以添加浇道或生成模具")
        self.statusLabel.setStyleSheet("color: #0078d4; padding: 5px;")

    def getStylesheet(self) -> str:
        return """
        QGroupBox {
            border: 1px solid #cccccc;
            border-radius: 5px;
            margin-top: 10px;
            padding-top: 10px;
            font-weight: bold;
        }
        QGroupBox::title {
            subcontrol-origin: margin;
            left: 10px;
            padding: 0 5px 0 5px;
        }
        QPushButton {
            background-color: #0078d4;
            color: white;
            border: none;
            border-radius: 4px;
            padding: 8px 16px;
            font-weight: bold;
        }
        QPushButton:hover {
            background-color: #1084d7;
        }
        QPushButton:pressed {
            background-color: #005a9e;
        }
        QPushButton:disabled {
            background-color: #cccccc;
            color: #888888;
        }
        """
