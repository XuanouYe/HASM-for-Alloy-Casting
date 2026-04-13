import trimesh
from PyQt5.QtCore import QObject, pyqtSignal, pyqtSlot
from gui.workerThread import WorkerThread
from mold.moldGenerator import MoldGenerator
from mold.orientationOptimizer import MoldOrientationOptimizer, OptimizerConfig
from geometryAdapters import exportMeshToStl


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
    moldExported = pyqtSignal(str)
    orientationProgress = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.currentCastingMesh = None
        self.currentMoldShell = None
        self.currentGateMesh = None
        self.currentRiserMesh = None
        self.currentWorker = None

    def _unwrapResult(self, result):
        if isinstance(result, dict) and "result" in result:
            return result["result"]
        return result

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
        moldShell = self._unwrapResult(result)
        if not isinstance(moldShell, trimesh.Trimesh):
            self.processError.emit("模具生成失败", f"返回值类型异常: {type(moldShell)}")
            return
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
        data = self._unwrapResult(result)
        if not isinstance(data, dict):
            self.processError.emit("浇道添加失败", f"返回值类型异常: {type(data)}")
            return
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

    @pyqtSlot(str)
    def exportMoldMesh(self, filePath: str):
        exportMeshToStl(self.currentMoldShell, filePath)
        self.moldExported.emit(filePath)

    def handleOptimizeOrientation(self, config: dict):
        if self.currentCastingMesh is None:
            self.processError.emit("校验错误", "请先加载铸件模型")
            return

        castingSnapshot = self.currentCastingMesh.copy()
        optimizerConfig = OptimizerConfig(
            boundingBoxOffset=float(config.get("boundingBoxOffset", 2.0)),
            booleanEngine=config.get("booleanEngine", None),
            gaPopulationSize=int(config.get("gaPopulationSize", 30)),
            gaGenerations=int(config.get("gaGenerations", 25)),
            gaMutationRate=float(config.get("gaMutationRate", 0.1)),
            gaCrossoverRate=float(config.get("gaCrossoverRate", 0.8)),
            gaTournamentSize=int(config.get("gaTournamentSize", 3)),
            gaEliteRatio=float(config.get("gaEliteRatio", 0.1)),
            gaMutationStep=float(config.get("gaMutationStep", 0.5)),
            numSamples=int(config.get("numSamples", 2000)),
            raysPerPoint=int(config.get("raysPerPoint", 64)),
            supportAngle=float(config.get("supportAngle", 45.0)),
            layerHeight=float(config.get("layerHeight", 0.5)),
            smoothHeight=float(config.get("smoothHeight", 1.0)),
            optimizationPriority=config.get("optimizationPriority", "machining"),
            logCsvPath=config.get("logCsvPath", None),
            saveBestMoldPath=config.get("saveBestMoldPath", None),
        )

        def task():
            optimizer = MoldOrientationOptimizer(optimizerConfig)
            result = optimizer.optimize(castingSnapshot)
            return result

        self.currentWorker = WorkerThread(task)
        self.currentWorker.taskCompleted.connect(self._onOrientationOptimized)
        self.currentWorker.taskError.connect(lambda err: self.processError.emit("方向调整失败", err))
        self.currentWorker.start()

    def _onOrientationOptimized(self, result):
        data = self._unwrapResult(result)
        optimizedMesh = data.get("bestMesh") if isinstance(data, dict) else data
        if isinstance(optimizedMesh, trimesh.Scene):
            optimizedMesh = optimizedMesh.dump(concatenate=True)
        self.currentCastingMesh = optimizedMesh
        bestAlpha = data.get("bestAlpha", 0.0) if isinstance(data, dict) else 0.0
        bestBeta = data.get("bestBeta", 0.0) if isinstance(data, dict) else 0.0
        summaryMsg = f"alpha={bestAlpha:.2f}° beta={bestBeta:.2f}°"
        self.orientationOptimized.emit(summaryMsg)
        self.updateCastingView.emit(self.currentCastingMesh)

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
        shell = self._unwrapResult(result)
        if not isinstance(shell, trimesh.Trimesh):
            self.processError.emit("表面偏移失败", f"返回值类型异常: {type(shell)}")
            return
        self.currentMoldShell = shell
        self.structureAdjusted.emit(offsetVal)
        self.updateMoldView.emit(self.currentMoldShell)
