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
    moldBoundsReady = pyqtSignal(dict)
    cavityVolumeReady = pyqtSignal(float)

    def __init__(self):
        super().__init__()
        self.currentCastingMesh = None
        self.currentMoldShell = None
        self.currentGateMesh = None
        self.currentRiserMesh = None
        self.currentWorker = None

    def handleLoadModel(self, filePath: str):
        self.currentCastingMesh = trimesh.load(filePath)
        self.currentGateMesh = None
        self.currentRiserMesh = None
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
        bounds = self.currentMoldShell.bounds
        minPt, maxPt = bounds[0], bounds[1]
        self.moldBoundsReady.emit({
            "xMin": float(minPt[0]), "xMax": float(maxPt[0]),
            "yMin": float(minPt[1]), "yMax": float(maxPt[1]),
            "zMin": float(minPt[2]), "zMax": float(maxPt[2]),
            "xSize": float(maxPt[0] - minPt[0]),
            "ySize": float(maxPt[1] - minPt[1]),
            "zSize": float(maxPt[2] - minPt[2]),
        })

    def handleAddGating(self, config: dict):
        if self.currentCastingMesh is None:
            self.processError.emit("校验错误", "请先加载铸件模型")
            return

        def task():
            moldGen = MoldGenerator(config=config)
            gatingComponents = moldGen.generateGating(self.currentCastingMesh)
            return {
                "combined": gatingComponents.castingWithSystemMesh,
                "gate": gatingComponents.gateMesh,
                "riser": gatingComponents.riserMesh
            }

        self.currentWorker = WorkerThread(task)
        self.currentWorker.taskCompleted.connect(self._onGatingAdded)
        self.currentWorker.taskError.connect(lambda err: self.processError.emit("浇道添加失败", err))
        self.currentWorker.start()

    def _onGatingAdded(self, result):
        data = result.get("result") if isinstance(result, dict) and "result" in result else result
        self.currentCastingMesh = data["combined"]
        self.currentGateMesh = data["gate"]
        self.currentRiserMesh = data["riser"]
        self.gatingAdded.emit()
        self.updateCastingView.emit(self.currentCastingMesh)
        self._emitCavityVolume()

    def _emitCavityVolume(self):
        totalVolume = 0.0
        if self.currentCastingMesh is not None:
            totalVolume += float(self.currentCastingMesh.volume)
        if self.currentGateMesh is not None:
            totalVolume += float(self.currentGateMesh.volume)
        if self.currentRiserMesh is not None:
            totalVolume += float(self.currentRiserMesh.volume)
        self.cavityVolumeReady.emit(totalVolume)

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
