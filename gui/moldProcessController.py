import trimesh
from PyQt5.QtCore import QObject, pyqtSignal
from gui.workerThread import WorkerThread
from mold.moldGenerator import MoldGenerator


class MoldProcessController(QObject):
    modelLoaded = pyqtSignal()
    moldGenerated = pyqtSignal()
    gatingAdded = pyqtSignal()
    orientationOptimized = pyqtSignal(str)
    structureAdjusted = pyqtSignal(float)
    processError = pyqtSignal(str, str)
    updateCastingView = pyqtSignal(object)
    updateMoldView = pyqtSignal(object)
    modelLoadedPath = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.currentCastingMesh = None
        self.currentMoldShell = None
        self.currentWorker = None

    def handleLoadModel(self, filePath: str):
        self.currentCastingMesh = trimesh.load(filePath)
        self.modelLoaded.emit()
        self.modelLoadedPath.emit(filePath)
        self.updateCastingView.emit(self.currentCastingMesh)

    def handleGenerateMold(self, config: dict):
        if self.currentCastingMesh is None:
            self.processError.emit("校验错误", "请先加载铸件模型")
            return

        def task():
            moldGen = MoldGenerator(config=config)
            return moldGen.generateMoldShell(self.currentCastingMesh)

        self.currentWorker = WorkerThread(task)
        self.currentWorker.taskCompleted.connect(self._onMoldGenerated)
        self.currentWorker.taskError.connect(lambda err: self.processError.emit("模具生成失败", err))
        self.currentWorker.start()

    def _onMoldGenerated(self, result):
        moldShell = result.get("result") if isinstance(result, dict) else result
        self.currentMoldShell = moldShell
        self.moldGenerated.emit()
        self.updateMoldView.emit(self.currentMoldShell)

    def handleAddGating(self, config: dict):
        if self.currentCastingMesh is None:
            self.processError.emit("校验错误", "请先加载铸件模型")
            return

        def task():
            moldGen = MoldGenerator(config=config)
            gatingComponents = moldGen.generateGating(self.currentCastingMesh)
            return gatingComponents.castingWithSystemMesh

        self.currentWorker = WorkerThread(task)
        self.currentWorker.taskCompleted.connect(self._onGatingAdded)
        self.currentWorker.taskError.connect(lambda err: self.processError.emit("浇道添加失败", err))
        self.currentWorker.start()

    def _onGatingAdded(self, result):
        combined = result.get("result") if isinstance(result, dict) else result
        self.currentCastingMesh = combined
        self.gatingAdded.emit()
        self.updateCastingView.emit(self.currentCastingMesh)

    def handleOptimizeOrientation(self, config: dict):
        if self.currentMoldShell is None:
            self.processError.emit("校验错误", "请先生成模具")
            return

        optType = config.get("optimizationType", "milling")

        def task():
            moldGen = MoldGenerator(config=config)
            return moldGen.optimizeOrientation(self.currentMoldShell)

        self.currentWorker = WorkerThread(task)
        self.currentWorker.taskCompleted.connect(lambda res: self._onOrientationOptimized(res, optType))
        self.currentWorker.taskError.connect(lambda err: self.processError.emit("方向优化失败", err))
        self.currentWorker.start()

    def _onOrientationOptimized(self, result, optType):
        shell = result.get("result") if isinstance(result, dict) else result
        self.currentMoldShell = shell
        self.orientationOptimized.emit(optType)
        self.updateMoldView.emit(self.currentMoldShell)

    def handleAdjustStructure(self, config: dict):
        if self.currentMoldShell is None:
            self.processError.emit("校验错误", "请先生成模具")
            return

        offsetVal = config.get("offsetValue", 0.0)

        def task():
            moldGen = MoldGenerator(config=config)
            return moldGen.adjustStructure(self.currentMoldShell)

        self.currentWorker = WorkerThread(task)
        self.currentWorker.taskCompleted.connect(lambda res: self._onStructureAdjusted(res, offsetVal))
        self.currentWorker.taskError.connect(lambda err: self.processError.emit("表面偏移失败", err))
        self.currentWorker.start()

    def _onStructureAdjusted(self, result, offsetVal):
        shell = result.get("result") if isinstance(result, dict) else result
        self.currentMoldShell = shell
        self.structureAdjusted.emit(offsetVal)
        self.updateMoldView.emit(self.currentMoldShell)
