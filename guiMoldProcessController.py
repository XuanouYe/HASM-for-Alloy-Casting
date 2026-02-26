import trimesh
from PyQt5.QtCore import QObject, pyqtSignal
from guiWorkerThread import WorkerThread
from moldGenerator import MoldGenerator


class MoldProcessController(QObject):
    # ==== 状态信号 (State Signals) ====
    # 向 UI (Panel) 发送业务执行状态
    modelLoaded = pyqtSignal()
    moldGenerated = pyqtSignal()
    gatingAdded = pyqtSignal()
    orientationOptimized = pyqtSignal(str)
    structureAdjusted = pyqtSignal(float)
    processError = pyqtSignal(str, str)  # title, message

    # ==== 视图更新信号 (View Update Signals) ====
    # 专门用于触发 Viewer 更新 3D 画面 (传递 Trimesh 对象)
    updateCastingView = pyqtSignal(object)
    updateMoldView = pyqtSignal(object)

    # ==== 数据共享信号 ====
    # 传递给全局 Controller (例如告诉 FDM 生成器使用哪一个文件路径)
    modelLoadedPath = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        # 后台持有的业务数据 (模型状态)
        self.currentCastingMesh = None
        self.currentMoldShell = None

        # 引用当前的 Worker 避免被垃圾回收
        self.currentWorker = None

    def handleLoadModel(self, filePath: str):
        try:
            self.currentCastingMesh = trimesh.load(filePath)
            self.modelLoaded.emit()
            self.modelLoadedPath.emit(filePath)
            self.updateCastingView.emit(self.currentCastingMesh)
        except Exception as e:
            self.processError.emit("加载模型失败", str(e))

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
        moldShell = result.get("result")
        if moldShell is None:
            self.processError.emit("模具生成失败", "布尔引擎未能生成有效的模具")
            return
        self.currentMoldShell = moldShell
        self.moldGenerated.emit()
        self.updateMoldView.emit(self.currentMoldShell)

    def handleAddGating(self, config: dict):
        if self.currentCastingMesh is None:
            self.processError.emit("校验错误", "请先加载铸件模型")
            return

        def task():
            moldGen = MoldGenerator(config=config)
            # 1. 这里返回的是 GatingComponents 对象
            gatingComponents = moldGen.generateGating(self.currentCastingMesh, config)

            # 2. 直接从 components 提取已经拼接好的 mesh
            combinedMesh = gatingComponents.castingWithSystemMesh

            if combinedMesh is None or combinedMesh.is_empty:
                raise RuntimeError("生成的浇道模型为空")

            return combinedMesh

        self.currentWorker = WorkerThread(task)
        self.currentWorker.taskCompleted.connect(self._onGatingAdded)
        self.currentWorker.taskError.connect(lambda err: self.processError.emit("浇道添加失败", err))
        self.currentWorker.start()

    def _onGatingAdded(self, result):
        # 3. 拿到组装好的模型，更新给视图
        combined = result.get("result")
        if combined is None:
            self.processError.emit("错误", "浇道合并返回了空结果")
            return

        self.currentCastingMesh = combined
        self.gatingAdded.emit()
        self.updateCastingView.emit(self.currentCastingMesh)

    def handleOptimizeOrientation(self, config: dict):
        if self.currentMoldShell is None:
            self.processError.emit("校验错误", "请先生成模具")
            return

        optimizationType = config.get("optimizationType", "milling")

        def task():
            moldGen = MoldGenerator(config=config)
            return moldGen.optimizeOrientation(self.currentMoldShell, config)

        self.currentWorker = WorkerThread(task)
        # 使用 lambda 将附带参数一并传给回调函数
        self.currentWorker.taskCompleted.connect(lambda res: self._onOrientationOptimized(res, optimizationType))
        self.currentWorker.taskError.connect(lambda err: self.processError.emit("方向优化失败", err))
        self.currentWorker.start()

    def _onOrientationOptimized(self, result, optType):
        self.currentMoldShell = result["result"]
        self.orientationOptimized.emit(optType)
        self.updateMoldView.emit(self.currentMoldShell)

    def handleAdjustStructure(self, config: dict):
        if self.currentMoldShell is None:
            self.processError.emit("校验错误", "请先生成模具")
            return

        offsetValue = config.get("offsetValue", 0.0)

        def task():
            moldGen = MoldGenerator(config=config)
            return moldGen.adjustStructure(self.currentMoldShell, config)

        self.currentWorker = WorkerThread(task)
        self.currentWorker.taskCompleted.connect(lambda res: self._onStructureAdjusted(res, offsetValue))
        self.currentWorker.taskError.connect(lambda err: self.processError.emit("表面偏移失败", err))
        self.currentWorker.start()

    def _onStructureAdjusted(self, result, offsetValue):
        self.currentMoldShell = result["result"]
        self.structureAdjusted.emit(offsetValue)
        self.updateMoldView.emit(self.currentMoldShell)
