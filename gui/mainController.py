import json
import os
import tempfile
from datetime import timedelta
from PyQt5.QtCore import QObject, QTimer, Qt
from gui.workerThread import WorkerThread
from controlConfig import ConfigManager
from fdmExecutor import generateGcodeInterface
from cncPathDesigner import generateCncJobInterface
from geometryAdapters import exportMeshToStl


class MainController(QObject):
    def __init__(self, mainWindow, moldController, parameterPanel):
        super().__init__()
        self.mainWindow = mainWindow
        self.moldController = moldController
        self.parameterPanel = parameterPanel
        self.configManager = ConfigManager()
        self.currentConfigDict = self.configManager.getDefaultConfig()
        self.elapsedTime = timedelta(0)
        self.currentManifestData = None

        self.partStlPath = None
        self.moldStlPath = None
        self.gateStlPath = None
        self.riserStlPath = None
        self.currentStlPath = None

        self.timer = QTimer()
        self.timer.timeout.connect(self._updateTimer)
        self.timer.start(1000)
        self._initConnections()

    def _initConnections(self):
        self.mainWindow.intentNewProject.connect(self.handleNewProject)
        self.mainWindow.intentSaveProject.connect(self.handleSaveProject)
        self.mainWindow.intentLoadManifest.connect(self.handleLoadManifest)
        self.mainWindow.intentLoadConfig.connect(self.handleLoadConfig)
        self.mainWindow.intentSaveConfig.connect(self.handleSaveConfig)
        self.mainWindow.intentResetConfig.connect(self.handleResetConfig)
        self.mainWindow.intentGenerateGcode.connect(self.handleGenerateGcode)
        self.mainWindow.intentGenerateCnc.connect(self.handleGenerateCnc)
        self.moldController.modelLoadedPath.connect(self._onModelLoaded)

    def _updateTimer(self):
        self.elapsedTime += timedelta(seconds=1)
        totalSeconds = int(self.elapsedTime.total_seconds())
        hours, remainder = divmod(totalSeconds, 3600)
        minutes, seconds = divmod(remainder, 60)
        self.mainWindow.setTimerText(f"{hours:02d}:{minutes:02d}:{seconds:02d}")

    def handleNewProject(self):
        self.elapsedTime = timedelta(0)
        self.currentManifestData = None
        self.partStlPath = None
        self.moldStlPath = None
        self.gateStlPath = None
        self.riserStlPath = None
        self.currentStlPath = None
        self.mainWindow.setCncButtonEnabled(False)
        self.mainWindow.setGcodeButtonEnabled(False)
        self.mainWindow.setStatusText("新项目已创建")

    def handleSaveProject(self):
        self.mainWindow.setStatusText("项目正在保存...")
        self.mainWindow.showMessage("保存成功", "项目已保存")

    def handleLoadManifest(self, filePath):
        with open(filePath, "r", encoding="utf-8") as f:
            manifestData = json.load(f)
        self.currentManifestData = manifestData
        files = manifestData.get("files", {})
        self.partStlPath = files.get("partStl")
        self.moldStlPath = files.get("moldStl")
        self.gateStlPath = files.get("gateStl")
        self.riserStlPath = files.get("riserStl")

        if self.partStlPath and self.moldStlPath:
            self.mainWindow.setCncButtonEnabled(True)
            self.mainWindow.showMessage("成功", "制造清单加载成功，可以生成CNC路径")
        else:
            self.mainWindow.showMessage("警告", "制造清单缺少必要的部件模型或模具模型路径", isError=True)

    def _onModelLoaded(self, filePath):
        self.currentStlPath = filePath
        self.mainWindow.setGcodeButtonEnabled(True)
        self.mainWindow.setStatusText("铸件模型已加载")

    def handleLoadConfig(self, filePath):
        with open(filePath, "r", encoding="utf-8") as f:
            configDict = json.load(f)
        errors = self.configManager.validate(configDict)
        if errors:
            self.mainWindow.showMessage("配置验证失败", "配置文件存在问题:\\n" + "\\n".join(errors[:10]),
                                        isError=True)
            return
        self.currentConfigDict = configDict
        self.parameterPanel.loadConfiguration(configDict)
        self.mainWindow.moldProcessPanel.loadConfiguration(configDict)
        self.mainWindow.showMessage("成功", f"配置已加载: {filePath}")

    def handleSaveConfig(self, filePath):
        panelConfig = self.parameterPanel.getConfiguration()
        moldSection = self.mainWindow.moldProcessPanel.getMoldConfigurationSection()
        self.currentConfigDict.update({
            "additive": panelConfig.get("additive", {}),
            "casting": panelConfig.get("casting", {}),
            "subtractive": panelConfig.get("subtractive", {}),
            "mold": moldSection
        })
        with open(filePath, "w", encoding="utf-8") as f:
            json.dump(self.currentConfigDict, f, indent=2, ensure_ascii=False)
        self.mainWindow.showMessage("成功", f"配置已保存: {filePath}")

    def handleResetConfig(self):
        self.currentConfigDict = self.configManager.getDefaultConfig()
        self.parameterPanel.loadConfiguration(self.currentConfigDict)
        self.mainWindow.moldProcessPanel.loadConfiguration(self.currentConfigDict)
        self.mainWindow.showMessage("成功", "已重置为默认配置")

    def handleGenerateGcode(self, outputPath):
        stlToSlice = None
        if self.moldController.currentMoldShell is not None:
            tempStlPath = os.path.join(tempfile.gettempdir(), "temp_mold.stl")
            exportMeshToStl(self.moldController.currentMoldShell, tempStlPath)
            stlToSlice = tempStlPath
        elif self.moldStlPath:
            stlToSlice = self.moldStlPath
        elif self.currentStlPath:
            stlToSlice = self.currentStlPath
        else:
            self.mainWindow.setGcodeButtonEnabled(True)
            return

        config = self.parameterPanel.getConfiguration()

        def taskCallable():
            return generateGcodeInterface(
                stlPath=stlToSlice,
                outputPath=outputPath,
                processConfig=config,
            )

        self.gcodeWorker = WorkerThread(taskCallable)
        self.gcodeWorker.taskCompleted.connect(self._onGenerateGcodeCompleted)
        self.gcodeWorker.taskError.connect(lambda err: self._onGenerateError("生成G代码失败", err, "Gcode"))
        self.gcodeWorker.start()

    def _onGenerateGcodeCompleted(self, result):
        self.mainWindow.setGcodeButtonEnabled(True)
        gcodePath = result.get("result") if "result" in result else result.get("gcodePath", "")
        self.mainWindow.showMessage("成功", f"G代码已生成:\\n{gcodePath}" if gcodePath else "生成完成")

    def handleGenerateCnc(self, outputPath, visualize):
        if not self.partStlPath or not self.moldStlPath:
            self.mainWindow.showMessage("错误", "缺少必要的标准化STL模型路径。请先加载Manufacturing Manifest。",
                                        isError=True)
            self.mainWindow.setCncButtonEnabled(True)
            return
        config = self.parameterPanel.getConfiguration()
        self.mainWindow.setStatusText("正在计算CNC刀位...")

        def taskCallable():
            gatePathToUse = self.gateStlPath if self.gateStlPath else self.partStlPath
            riserPathToUse = self.riserStlPath if self.riserStlPath else self.partStlPath

            return generateCncJobInterface(
                partStl=self.partStlPath,
                moldStl=self.moldStlPath,
                gateStl=gatePathToUse,
                riserStl=riserPathToUse,
                outputJsonPath=outputPath,
                processConfig=config,
                visualize=visualize
            )

        self.cncWorker = WorkerThread(taskCallable)
        self.cncWorker.taskCompleted.connect(self._onGenerateCncCompleted)
        self.cncWorker.taskError.connect(lambda err: self._onGenerateError("生成CNC刀位失败", err, "Cnc"))
        self.cncWorker.start()

    def _onGenerateCncCompleted(self, result):
        self.mainWindow.setCncButtonEnabled(True)
        self.mainWindow.setStatusText("CNC刀位生成完成")
        self.mainWindow.showMessage("完成", "CNC刀位JSON已成功生成。")

    def _onGenerateError(self, title, errorMsg, btnType):
        if btnType == "Gcode":
            self.mainWindow.setGcodeButtonEnabled(True)
        else:
            self.mainWindow.setCncButtonEnabled(True)
            self.mainWindow.setStatusText("生成失败")
        self.mainWindow.showMessage("错误", f"{title}:\\n{errorMsg}", isError=True)
