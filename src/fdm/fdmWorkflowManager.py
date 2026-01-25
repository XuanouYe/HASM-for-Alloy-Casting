import logging
from typing import Dict, Optional, Any
from pathlib import Path

from src.control.controlConfigManager import ConfigManager
from src.control.controlDataExchangeInterface import DataExchangeFormat, MessageRouter
from fdmCuraEngineExecutor import CuraEngineController, SliceException
from fdmSliceConfigAdaptor import generateCuraConfig


class FDMAdditiveState:
    """
    改进版增材制造工作流状态类

    职责:
    - 工作流编排和执行
    - 配置加载与验证
    - 调用CuraEngineController执行切片
    - 消息路由和中间产物记录

    属性:
        projectId: 项目标识符
        context: 工作流上下文字典（标准化键名）
        configManager: 中央配置管理器
        curaController: CuraEngine控制器实例
        messageRouter: 模块间消息路由器
        logger: 日志记录器
    """

    # 标准Context键名
    CONTEXT_KEYS = {
        "wslCuraEnginePath": "/mnt/c/CuraEngine/CuraEngine",  # WSL中CuraEngine路径
        "curaDefinitionFiles": [],  # Cura定义文件列表
        "logLevel": "INFO",  # 日志级别
    }

    def __init__(
        self,
        projectId: str,
        context: Dict = None,
        configManager: ConfigManager = None,
        messageRouter: MessageRouter = None
    ):
        """
        初始化增材制造工作流状态

        参数:
            projectId: 项目标识符
            context: 工作流上下文字典（将验证和标准化）
            configManager: 中央配置管理器实例（可选）
            messageRouter: 消息路由器实例（可选）
        """
        self.projectId = projectId
        self.context = self._validateAndNormalizeContext(context or {})
        self.logger = logging.getLogger("FDMAdditiveState")

        # 初始化配置和消息系统
        self.configManager = configManager or ConfigManager()
        self.messageRouter = messageRouter
        if self.messageRouter:
            self.messageRouter.registerModule("additive")

        # 初始化控制器
        self.curaController = CuraEngineController(
            self.context["wslCuraEnginePath"],
            logLevel=self.context["logLevel"]
        )

        self.previewMode = False
        self.logger.info(f"FDMAdditiveState initialized for project: {projectId}")

    @staticmethod
    def _validateAndNormalizeContext(context: Dict) -> Dict:
        """
        验证并标准化Context结构

        标准键名:
        - wslCuraEnginePath: WSL中CuraEngine的路径
        - curaDefinitionFiles: Cura定义文件列表
        - logLevel: 日志级别
        - inputStlPath: 输入STL文件路径（会在execute()中验证）
        - outputGcodePath: 输出G代码路径（会在execute()中验证）

        参数:
            context: 传入的上下文字典

        返回:
            标准化后的上下文字典

        异常:
            ValueError: 必要键值缺失或无效
        """
        normalized = dict(FDMAdditiveState.CONTEXT_KEYS)
        normalized.update(context)

        # 验证WSL CuraEngine路径
        if not isinstance(normalized.get("wslCuraEnginePath"), str):
            raise ValueError("wslCuraEnginePath must be a non-empty string")

        # 验证Cura定义文件列表
        if not isinstance(normalized.get("curaDefinitionFiles"), list):
            normalized["curaDefinitionFiles"] = []

        # 验证日志级别
        validLevels = {"DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"}
        if normalized.get("logLevel") not in validLevels:
            normalized["logLevel"] = "INFO"

        return normalized

    def setPreviewMode(self, enabled: bool) -> None:
        """
        设置预览模式

        参数:
            enabled: 是否启用预览模式
        """
        self.previewMode = enabled
        mode = "PREVIEW" if enabled else "PRODUCTION"
        self.logger.info(f"Mode set to: {mode}")

    def setTimestampMode(self, enabled: bool) -> None:
        """
        设置是否添加时间戳

        参数:
            enabled: 是否启用时间戳
        """
        self.context["addTimestamp"] = enabled
        self.logger.info(f"Timestamp mode: {'ENABLED' if enabled else 'DISABLED'}")

    # ====== 不再包装configManager方法 ======
    # 外部代码直接使用: state.configManager.applyPreset(...)
    #                  state.configManager.validateConfig(...)

    def _getMoldStlPath(self) -> str:
        """
        获取上游模具STL路径（标准化为单一键名）

        返回:
            STL文件的完整路径（Windows格式）

        异常:
            ValueError: STL路径未在context中指定或不存在
        """
        stlPath = self.context.get("inputStlPath")

        if not stlPath:
            raise ValueError(
                "inputStlPath must be specified in context. "
                "Standard context keys: inputStlPath, outputGcodePath, "
                "wslCuraEnginePath, curaDefinitionFiles"
            )

        # 验证文件存在
        if not Path(stlPath).exists():
            raise ValueError(f"Input STL file not found: {stlPath}")

        self.logger.info(f"Mold STL path resolved: {stlPath}")
        return stlPath

    def _getOutputGcodePath(self) -> str:
        """
        获取输出G代码路径

        返回:
            G代码输出文件路径（Windows格式）

        异常:
            ValueError: 输出路径未在context中指定
        """
        outputPath = self.context.get("outputGcodePath")

        if not outputPath:
            raise ValueError(
                "outputGcodePath must be specified in context. "
                "Standard context keys: inputStlPath, outputGcodePath, "
                "wslCuraEnginePath, curaDefinitionFiles"
            )

        self.logger.info(f"Output G-code path resolved: {outputPath}")
        return outputPath

    def execute(self) -> Optional['CastingState']:
        """
        执行增材制造工作流

        工作流步骤:
        1. 加载项目配置
        2. 验证配置完整性
        3. 获取输入STL和输出路径
        4. 转换配置格式（camelCase → snake_case）
        5. 调用CuraEngineController执行切片（路径转换由Controller负责）
        6. 验证G代码输出
        7. 发送消息到下游模块
        8. 返回下一个工作流状态

        返回:
            下一个工作流状态实例

        异常:
            ValueError: 配置验证失败
            SliceException: 切片执行失败
        """
        self.logger.info("=" * 60)
        self.logger.info(f"Executing Additive Manufacturing Stage - Project: {self.projectId}")
        self.logger.info("=" * 60)

        # 第一步：加载项目配置
        projectConfig = self.configManager.loadProjectConfig(self.projectId)
        self.logger.info(f"Project configuration loaded: {self.projectId}")

        # 第二步：验证配置
        isValid, errors = self.configManager.validateConfig(self.projectId)
        if not isValid:
            raise ValueError(f"Configuration validation failed: {errors}")
        self.logger.info("Configuration validation passed")

        # 第三步：获取输入和输出路径
        moldStlPath = self._getMoldStlPath()  # Windows路径
        outputGcodePath = self._getOutputGcodePath()  # Windows路径

        # 第四步：获取切片参数（配置格式转换在此进行）
        additiveConfig = projectConfig.get("additiveConfig", {})
        curaSettings = generateCuraConfig(additiveConfig)  # 使用工具函数转换

        self.logger.info(f"Generated Cura settings: {list(curaSettings.keys())}")

        # 第五步：执行切片（CuraEngineController内部处理路径转换）
        try:
            gcodeOutputPath = self.curaController.generateGcode(
                stlPath=moldStlPath,
                outputPath=outputGcodePath,
                settings=curaSettings,
                definitionFiles=self.context.get("curaDefinitionFiles"),
                previewOnly=self.previewMode,
                addTimestamp=self.context.get("addTimestamp", False)
            )
            self.logger.info(f"G-code generated: {gcodeOutputPath}")
        except SliceException as e:
            self.logger.error(f"Slicing failed: {str(e)}")
            raise

        # 第六步：发送消息到下游
        if self.messageRouter:
            message = DataExchangeFormat.create(
                source="additive",
                target="casting",
                payload={
                    "projectId": self.projectId,
                    "gcodeOutputPath": gcodeOutputPath,
                    "status": "success"
                }
            )
            self.messageRouter.sendMessage(message)
            self.logger.info("Result message sent to downstream module (casting)")

        # 第七步：记录中间产物
        self.context["gcodeOutputPath"] = gcodeOutputPath
        self.context["lastSliceTime"] = self.curaController.lastExecutionTime

        self.logger.info("=" * 60)
        self.logger.info(f"Additive Manufacturing Stage completed - Project: {self.projectId}")
        self.logger.info("=" * 60)

        # 返回下一个工作流状态
        # (实际应用中需要import CastingState)
        return None  # 占位符
