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

    def unwrapResult(self, result):
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
        def task():
            moldGen = MoldGenerator(config=config)
            return moldGen.generateMoldShell(self.currentCastingMesh)

        self.currentWorker = WorkerThread(task)
        self.currentWorker.taskCompleted.connect(self.onMoldGenerated)
        self.currentWorker.start()

    def onMoldGenerated(self, result):
        moldShell = self.unwrapResult(result)
        self.currentMoldShell = moldShell
        self.updateMoldView.emit(self.currentMoldShell)

        # 修复2: 先发射数据统计信号，确保在弹窗阻塞前更新UI
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
        # 最后发射完成信号 (会触发弹窗)
        self.moldGenerated.emit()

    def handleAddGating(self, config: dict):
        def task():
            moldGen = MoldGenerator(config=config)
            gatingComponents = moldGen.generateGating(self.currentCastingMesh)
            return {
                "combined": gatingComponents.castingWithSystemMesh,
                "gate": gatingComponents.gateMesh,
                "riser": gatingComponents.riserMesh
            }

        self.currentWorker = WorkerThread(task)
        self.currentWorker.taskCompleted.connect(self.onGatingAdded)
        self.currentWorker.start()

    def onGatingAdded(self, result):
        data = self.unwrapResult(result)
        self.currentCastingMesh = data["combined"]
        self.currentGateMesh = data["gate"]
        self.currentRiserMesh = data["riser"]
        self.updateCastingView.emit(self.currentCastingMesh)

        # 先发射数据更新信号
        self.emitCavityVolume()
        # 后发射弹窗信号
        self.gatingAdded.emit()

    def emitCavityVolume(self):
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
            return optimizer.optimize(castingSnapshot)

        self.currentWorker = WorkerThread(task)
        self.currentWorker.taskCompleted.connect(self.onOrientationOptimized)
        self.currentWorker.start()

    def onOrientationOptimized(self, result):
        data = self.unwrapResult(result)
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
        offsetVal = config.get("offsetValue", 0.0)

        def task():
            moldGen = MoldGenerator(config=config)
            return moldGen.adjustStructure(self.currentMoldShell)

        self.currentWorker = WorkerThread(task)
        self.currentWorker.taskCompleted.connect(lambda res: self.onStructureAdjusted(res, offsetVal))
        self.currentWorker.start()

    def onStructureAdjusted(self, result, offsetVal):
        shell = self.unwrapResult(result)
        self.currentMoldShell = shell
        self.structureAdjusted.emit(offsetVal)
        self.updateMoldView.emit(self.currentMoldShell)