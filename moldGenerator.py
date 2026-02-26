import numpy as np
import trimesh
from typing import Tuple, Optional
from moldGatingSystem import createGatingSystem


# [Task D] 新增统一工作流入口
def generateMold(jobContext):
    import configEngine
    from geometryAdapters import loadMeshFromFile, exportMeshToStl
    from moldOrientationOptimizer import optimizeMoldOrientation
    from moldInnerSurfaceOffset import removeInnerSurfaceOverhangs

    snapshot = jobContext.configSnapshot
    moldCfg = configEngine.getSection("mold", jobContext)

    # 路径从快照中读取
    inputPath = jobContext.inputGeometryRef
    outputPartPath = snapshot.paths.partStl
    outputMoldPath = snapshot.paths.moldShellStl
    outputGatingPath = snapshot.paths.gatingStl

    # 加载输入
    partMesh = loadMeshFromFile(inputPath)

    # 初始化旧版类实例
    moldGen = MoldGenerator(config=moldCfg)

    # 1. 模具壳生成
    moldShell = moldGen.generateMoldShell(partMesh)

    # 2. 浇注系统
    gatingMesh = None
    if moldCfg.get("addGating", False):
        gatingMesh = moldGen.generateGating(partMesh, moldCfg)

    # 3. 摆放优化 (适配新参数结构)
    if moldCfg.get("optimizeOrientation", False):
        optResult = optimizeMoldOrientation(
            inputCasting=partMesh,
            outputCastingPath=outputPartPath,
            saveBestMoldPath=None,  # 由统一出口保存
            **moldCfg
        )
        partMesh = optResult["bestMesh"]
        moldShell = optResult["bestMold"]

    # 4. 内部支撑面偏置
    if moldCfg.get("adjustStructure", False):
        moldShell = removeInnerSurfaceOverhangs(moldShell, moldCfg)

    # 5. WCS 对齐与最终输出
    partMesh, moldShell, gatingMesh, transformMatrix = moldGen.normalizeMeshesToWcs(
        partMesh, moldShell, gatingMesh
    )

    # 写入产物
    exportMeshToStl(partMesh, outputPartPath)
    exportMeshToStl(moldShell, outputMoldPath)
    if gatingMesh is not None:
        exportMeshToStl(gatingMesh, outputGatingPath)

    # [Task A] 直接登记产物
    jobContext.registerArtifact("partMesh", outputPartPath, "generateMold")
    jobContext.registerArtifact("moldShell", outputMoldPath, "generateMold")
    if gatingMesh is not None:
        jobContext.registerArtifact("gatingMesh", outputGatingPath, "generateMold")

    if jobContext.moldPlan is None:
        from dataModel import MoldPlan
        jobContext.moldPlan = MoldPlan()
    jobContext.moldPlan.wcsTranslation = transformMatrix[:3, 3].tolist()

    return {
        "moldShellPath": outputMoldPath,
        "gatingPath": outputGatingPath,
        "partPath": outputPartPath,
    }


class MoldGenerator:
    def __init__(self, config=None):
        self.config = config or {}
        self.boundingBoxOffset = float(self.config.get('boundingBoxOffset', 2.0))
        self.booleanEngine = self.config.get('booleanEngine', None)

    def generateMoldShell(self, inputMesh: trimesh.Trimesh) -> trimesh.Trimesh:
        inputMesh = self.ensureTrimesh(inputMesh)
        boundingBox = self.calculateBoundingBox(inputMesh)
        blankMesh = self.createBlankMesh(boundingBox)
        return self.booleanDifference(blankMesh, inputMesh)

    def ensureTrimesh(self, meshOrScene) -> trimesh.Trimesh:
        if isinstance(meshOrScene, trimesh.Trimesh):
            return meshOrScene
        if isinstance(meshOrScene, trimesh.Scene):
            dumped = meshOrScene.dump(concatenate=True)
            if isinstance(dumped, trimesh.Trimesh):
                return dumped
        raise TypeError(f"Unsupported mesh type: {type(meshOrScene)}")

    def calculateBoundingBox(self, mesh: trimesh.Trimesh) -> np.ndarray:
        bounds = np.array(mesh.bounds, dtype=float)
        offset = float(self.boundingBoxOffset)
        bounds[0, :] -= offset
        bounds[1, 0] += offset
        bounds[1, 1] += offset
        eps = 1e-6
        bounds[1, :] = np.maximum(bounds[1, :], bounds[0, :] + eps)
        return bounds

    def createBlankMesh(self, bbox: np.ndarray) -> trimesh.Trimesh:
        blankMesh = trimesh.creation.box(bounds=bbox)
        blankMesh.process(validate=True)
        return blankMesh

    def booleanDifference(self, blankMesh: trimesh.Trimesh, inputMesh: trimesh.Trimesh) -> trimesh.Trimesh:
        blank = blankMesh.copy()
        part = inputMesh.copy()
        blank.process(validate=True)
        part.process(validate=True)

        enginesToTry = []
        if self.booleanEngine is not None:
            enginesToTry.append(self.booleanEngine)
        for e in ["manifold", "blender", "scad", None]:
            if e not in enginesToTry:
                enginesToTry.append(e)

        lastError = None
        for engine in enginesToTry:
            try:
                result = trimesh.boolean.difference(blank, part, engine=engine, check_volume=False)
                if result is None:
                    raise RuntimeError(f"Boolean returned None (engine={engine})")
                if isinstance(result, trimesh.Scene):
                    result = result.dump(concatenate=True)
                if not isinstance(result, trimesh.Trimesh):
                    raise RuntimeError(f"Unexpected boolean result type: {type(result)}")
                result.process(validate=True)
                return result
            except Exception as exc:
                lastError = exc

        raise RuntimeError("Boolean difference failed with all engines tried.") from lastError

    def generateGating(self, castingMesh: trimesh.Trimesh, config: dict) -> trimesh.Trimesh:
        gatingConfig = {
            "targetFillTime": config.get("targetFillTime", 5.0),
            "sprueInletOffset": config.get("sprueInletOffset", 5.0),
            "boundingBoxOffset": config.get("boundingBoxOffset", 2.0)
        }
        return createGatingSystem(castingMesh=castingMesh, config=gatingConfig)

    def normalizeMeshesToWcs(self, partMesh: trimesh.Trimesh, moldMesh: trimesh.Trimesh,
                             gatingMesh: Optional[trimesh.Trimesh]) -> Tuple[
        trimesh.Trimesh, trimesh.Trimesh, Optional[trimesh.Trimesh], np.ndarray]:
        meshesToConsider = [partMesh, moldMesh]
        if gatingMesh is not None:
            meshesToConsider.append(gatingMesh)

        minZ = min([mesh.bounds[0][2] for mesh in meshesToConsider])
        minX = min([mesh.bounds[0][0] for mesh in meshesToConsider])
        maxX = max([mesh.bounds[1][0] for mesh in meshesToConsider])
        minY = min([mesh.bounds[0][1] for mesh in meshesToConsider])
        maxY = max([mesh.bounds[1][1] for mesh in meshesToConsider])

        centerX = (minX + maxX) / 2.0
        centerY = (minY + maxY) / 2.0

        translation = np.array([-centerX, -centerY, -minZ])
        matrix = np.eye(4)
        matrix[:3, 3] = translation

        partMesh.apply_transform(matrix)
        moldMesh.apply_transform(matrix)
        if gatingMesh is not None:
            gatingMesh.apply_transform(matrix)

        return partMesh, moldMesh, gatingMesh, matrix
