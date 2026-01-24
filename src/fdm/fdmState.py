"""
增材制造工作流状态 - 集成CuraEngine切片模块

功能：
- 封装切片在工作流中的执行逻辑
- 管理状态转移（STL→G代码→后处理）
- 支持预览和生产模式切换
- 记录中间产物和工作流数据
"""

import logging
from typing import Dict, Optional, Any
from datetime import datetime
from pathlib import Path

from fdmCuraEngineController import CuraEngineController, SliceException
from fdmSliceManager import SliceConfigManager


class AdditiveManufacturingState:
    """
    增材制造工作流状态类

    职责：
    - 接收上游的模具STL
    - 配置切片参数
    - 执行CuraEngine切片
    - 将G代码传递到下游后处理

    属性:
        context: 工作流上下文字典
        curaController: CuraEngine控制器实例
        configManager: 切片配置管理器实例
        logger: 日志记录器
    """

    def __init__(self, context: Dict = None):
        """
        初始化增材制造状态

        参数:
            context: 工作流上下文字典，包含必要的配置和数据路径
        """
        self.context = context or {}
        self.logger = logging.getLogger("AdditiveManufacturingState")

        # 初始化控制器和配置管理器
        self._initializeControllers()

        # 默认参数
        self.previewMode = False
        self.sliceTimestamp = None

        self.logger.info("AdditiveManufacturingState initialized")

    def _initializeControllers(self) -> None:
        """
        初始化CuraEngine控制器和配置管理器

        异常:
            RuntimeError: 控制器初始化失败
        """
        try:
            # 从context获取CuraEngine路径，默认值为".CuraEngine"
            enginePath = self.context.get("curaEnginePath", "./CuraEngine")
            self.curaController = CuraEngineController(enginePath)

            self.configManager = SliceConfigManager()

            self.logger.info("Controllers initialized successfully")

        except SliceException as e:
            self.logger.error(f"Failed to initialize CuraEngine: {str(e)}")
            raise RuntimeError(f"CuraEngine initialization failed: {str(e)}")

    def setPreviewMode(self, enabled: bool) -> None:
        """
        设置预览模式

        参数:
            enabled: 是否启用预览模式（不执行实际切片）
        """
        self.previewMode = enabled
        mode = "PREVIEW" if enabled else "PRODUCTION"
        self.logger.info(f"Mode set to: {mode}")

    def configureSliceParameters(
            self,
            preset: Optional[str] = None,
            customParams: Optional[Dict] = None
    ) -> None:
        """
        配置切片参数

        参数:
            preset: 预设配置名称（如"high_temp_450"）
            customParams: 自定义参数覆盖

        异常:
            ValueError: 参数验证失败
        """
        try:
            # 加载预设配置
            if preset:
                self.configManager.loadPreset(preset)
                self.logger.info(f"Loaded preset: {preset}")

            # 应用自定义参数
            if customParams:
                self.configManager.setBatchUpdate(customParams)
                self.logger.info(f"Applied custom parameters: {list(customParams.keys())}")

            # 验证配置
            isValid, message = self.configManager.validateConfig()
            if not isValid:
                raise ValueError(f"Configuration validation failed: {message}")

            self.logger.info("Slice configuration configured successfully")

        except Exception as e:
            self.logger.error(f"Configuration failed: {str(e)}")
            raise

    def execute(self) -> 'NextManufacturingState':
        """
        执行增材制造（切片）工作流

        工作流:
        1. 获取上游模具STL
        2. 配置切片参数（若未配置则使用默认）
        3. 执行CuraEngine切片
        4. 验证G代码输出
        5. 更新context传递到下游
        6. 返回下一个工作流状态

        返回:
            下一个工作流状态实例

        异常:
            ValueError: 模具STL不存在
            SliceException: 切片执行失败
        """
        self.logger.info("=" * 60)
        self.logger.info("Executing Additive Manufacturing (Slicing) Stage")
        self.logger.info("=" * 60)

        try:
            # 第一步：获取上游模具STL
            moldStlPath = self._getMoldStlPath()
            self.logger.info(f"Mold STL acquired: {moldStlPath}")

            # 第二步：生成输出文件路径
            outputGcodePath = self._generateOutputPath()
            self.logger.info(f"Output G-code path: {outputGcodePath}")

            # 第三步：执行切片
            self.logger.info("Starting CuraEngine slice operation...")

            finalGcodePath = self.curaController.generateGcode(
                stlPath=moldStlPath,
                configDict=self.configManager.toDict(),
                outputPath=outputGcodePath,
                previewOnly=self.previewMode
            )

            # 记录切片时间戳
            self.sliceTimestamp = datetime.now().isoformat()

            # 第四步：更新context
            self.context["rawGcodePath"] = finalGcodePath
            self.context["sliceTimestamp"] = self.sliceTimestamp
            self.context["sliceConfig"] = self.configManager.toDict()
            self.context["executionTime"] = self.curaController.getLastExecutionTime()

            self.logger.info(f"Slice completed in {self.context['executionTime']:.2f}s")
            self.logger.info(f"G-code saved to: {finalGcodePath}")

            # 第五步：返回下一个工作流状态
            nextState = self._getNextState()
            self.logger.info(f"Transitioning to: {nextState.__class__.__name__}")

            return nextState

        except Exception as e:
            self.logger.error(f"Additive Manufacturing stage failed: {str(e)}")
            raise

    def _getMoldStlPath(self) -> str:
        """
        获取模具STL文件路径（从上游工作流）

        返回:
            STL文件绝对路径

        异常:
            ValueError: 路径不存在或未指定
        """
        # 支持多个可能的context key
        possibleKeys = [
            "optimizedMoldPath",
            "moldStlPath",
            "stlPath",
            "inputStl"
        ]

        moldPath = None
        for key in possibleKeys:
            if key in self.context:
                moldPath = self.context[key]
                break

        if not moldPath:
            raise ValueError(
                f"Mold STL path not found in context. "
                f"Expected one of: {possibleKeys}"
            )

        # 验证路径存在
        if not Path(moldPath).exists():
            raise ValueError(f"Mold STL file not found: {moldPath}")

        return str(Path(moldPath).resolve())

    def _generateOutputPath(self) -> str:
        """
        生成G代码输出文件路径

        返回:
            输出文件路径
        """
        # 从context获取输出目录，默认为当前目录
        outputDir = self.context.get("outputDir", "./gcode_output")

        # 从context获取模具ID（用于文件命名）
        moldId = self.context.get("moldId", "mold")

        # 创建输出目录
        Path(outputDir).mkdir(parents=True, exist_ok=True)

        outputPath = str(Path(outputDir) / f"{moldId}.gcode")

        return outputPath

    def _getNextState(self) -> Any:
        """
        获取下一个工作流状态

        返回:
            下一个工作流状态实例（可根据实际工作流扩展）
        """

        # 示例：返回占位符状态
        # 实际应用中应返回真实的后处理状态
        class GcodePostProcessingState:
            def __init__(self, context: Dict):
                self.context = context

            def execute(self):
                logger = logging.getLogger("GcodePostProcessingState")
                logger.info("G-code post-processing stage would execute here")
                return None

        return GcodePostProcessingState(self.context)

    def getSliceReport(self) -> Dict[str, Any]:
        """
        获取切片工作流报告

        返回:
            包含切片过程信息的字典
        """
        return {
            "timestamp": self.sliceTimestamp,
            "moldStlPath": self.context.get("optimizedMoldPath", "N/A"),
            "outputGcodePath": self.context.get("rawGcodePath", "N/A"),
            "executionTime": self.context.get("executionTime", 0),
            "sliceConfig": self.context.get("sliceConfig", {}),
            "previewMode": self.previewMode,
        }

    def setContext(self, contextDict: Dict) -> None:
        """
        更新工作流上下文

        参数:
            contextDict: 上下文字典
        """
        self.context.update(contextDict)
        self.logger.debug(f"Context updated with keys: {list(contextDict.keys())}")
