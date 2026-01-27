import json
from pathlib import Path
from typing import Dict, Any, List, Optional, Callable
from datetime import datetime
import logging
from enum import Enum

logger = logging.getLogger(__name__)

# ===================== 参数定义 =====================
parameterSchema = {
    "additive": {
        # 基础参数
        "layerHeight": {
            "type": float,
            "min": 0.08,
            "max": 0.4,
            "default": 0.2,
            "unit": "mm",
            "description": "3D打印层高",
            "required": True
        },
        "wallThickness": {
            "type": float,
            "min": 0.4,
            "max": 2.0,
            "default": 0.8,
            "unit": "mm",
            "description": "模具壁厚",
            "required": True
        },
        "wallLineCount": {
            "type": int,
            "min": 1,
            "max": 10,
            "default": 2,
            "unit": "lines",
            "description": "壁线数",
            "required": True
        },
        "infillDensity": {
            "type": int,
            "min": 0,
            "max": 100,
            "default": 20,
            "unit": "%",
            "description": "填充密度",
            "required": True
        },
        "infillPattern": {
            "type": str,
            "default": "grid",
            "options": ["grid", "honeycomb", "gyroid", "cubic", "tetrahedral"],
            "description": "填充模式",
            "required": True
        },
        "topLayers": {
            "type": int,
            "min": 0,
            "max": 50,
            "default": 4,
            "description": "顶层数",
            "required": True
        },
        "bottomLayers": {
            "type": int,
            "min": 0,
            "max": 50,
            "default": 4,
            "description": "底层数",
            "required": True
        },
        "roofingLayerCount": {
            "type": int,
            "min": 0,
            "max": 10,
            "default": 0,
            "description": "顶层覆盖层数",
            "required": False
        },
        "flooringLayerCount": {
            "type": int,
            "min": 0,
            "max": 10,
            "default": 0,
            "description": "底层覆盖层数",
            "required": False
        },
        # 温度参数
        "nozzleTemperature": {
            "type": int,
            "min": 180,
            "max": 450,
            "default": 210,
            "unit": "°C",
            "description": "喷嘴温度",
            "required": True
        },
        "bedTemperature": {
            "type": int,
            "min": 0,
            "max": 150,
            "default": 60,
            "unit": "°C",
            "description": "热床温度",
            "required": True
        },
        "nozzleTemperatureLayer0": {
            "type": int,
            "min": 180,
            "max": 450,
            "default": 210,
            "unit": "°C",
            "description": "首层喷嘴温度",
            "required": False
        },
        "bedTemperatureLayer0": {
            "type": int,
            "min": 0,
            "max": 150,
            "default": 60,
            "unit": "°C",
            "description": "首层热床温度",
            "required": False
        },
        # 速度参数
        "printSpeed": {
            "type": int,
            "min": 10,
            "max": 150,
            "default": 50,
            "unit": "mm/s",
            "description": "打印速度",
            "required": True
        },
        "printSpeedLayer0": {
            "type": int,
            "min": 5,
            "max": 100,
            "default": 20,
            "unit": "mm/s",
            "description": "首层打印速度",
            "required": False
        },
        "travelSpeed": {
            "type": int,
            "min": 50,
            "max": 300,
            "default": 150,
            "unit": "mm/s",
            "description": "移动速度",
            "required": False
        },
        # 冷却设置
        "fanEnabled": {
            "type": bool,
            "default": True,
            "description": "启用风扇",
            "required": False
        },
        "coolFanSpeed": {
            "type": int,
            "min": 0,
            "max": 100,
            "default": 100,
            "unit": "%",
            "description": "冷却风扇速度",
            "required": False
        },
        "coolMinLayerTime": {
            "type": float,
            "min": 1.0,
            "max": 60.0,
            "default": 10.0,
            "unit": "s",
            "description": "最小层时间",
            "required": False
        },
        # 回抽设置
        "retractionEnabled": {
            "type": bool,
            "default": True,
            "description": "启用回抽",
            "required": False
        },
        "retractionDistance": {
            "type": float,
            "min": 0.0,
            "max": 20.0,
            "default": 5.0,
            "unit": "mm",
            "description": "回抽距离",
            "required": False
        },
        "retractionSpeed": {
            "type": float,
            "min": 10.0,
            "max": 100.0,
            "default": 45.0,
            "unit": "mm/s",
            "description": "回抽速度",
            "required": False
        },
        # 支撑和附着
        "supportEnabled": {
            "type": bool,
            "default": False,
            "description": "启用支撑",
            "required": False
        },
        "adhesionType": {
            "type": str,
            "default": "none",
            "options": ["none", "raft", "brim", "skirt"],
            "description": "附着类型",
            "required": False
        }
    },
    "casting": {
        "temperature": {
            "type": float,
            "min": 50,
            "max": 500,
            "default": 200.0,
            "unit": "°C",
            "description": "铸造温度",
            "required": True
        },
        "volume": {
            "type": float,
            "min": 1,
            "max": 10000,
            "default": 100.0,
            "unit": "cm³",
            "description": "铸造体积",
            "required": True
        },
        "solidificationTime": {
            "type": int,
            "min": 10,
            "max": 7200,
            "default": 1800,
            "unit": "s",
            "description": "凝固时间",
            "required": True
        },
        "pressure": {
            "type": int,
            "min": 0,
            "max": 100,
            "default": 5,
            "unit": "bar",
            "description": "铸造压力",
            "required": True
        }
    },
    "subtractive": {
        "machineModel": {
            "type": str,
            "default": "Default_CNC",
            "description": "机床型号",
            "required": True
        },
        "spindleSpeed": {
            "type": int,
            "min": 100,
            "max": 60000,
            "default": 5000,
            "unit": "rpm",
            "description": "主轴转速",
            "required": True
        },
        "feedRate": {
            "type": int,
            "min": 1,
            "max": 5000,
            "default": 100,
            "unit": "mm/min",
            "description": "进给速度",
            "required": True
        },
        "depthOfCut": {
            "type": float,
            "min": 0.05,
            "max": 20.0,
            "default": 1.0,
            "unit": "mm",
            "description": "切削深度",
            "required": True
        },
        "stepOver": {
            "type": float,
            "min": 0.1,
            "max": 100.0,
            "default": 10.0,
            "unit": "%",
            "description": "步进百分比",
            "required": True
        }
    }
}

# ===================== 预设配置 =====================
presets = {
    "standard": {
        "additive": {
            "layerHeight": 0.2,
            "wallThickness": 0.8,
            "wallLineCount": 2,
            "infillDensity": 20,
            "infillPattern": "grid",
            "topLayers": 4,
            "bottomLayers": 4,
            "roofingLayerCount": 0,
            "flooringLayerCount": 0,
            "nozzleTemperature": 210,
            "bedTemperature": 60,
            "nozzleTemperatureLayer0": 210,
            "bedTemperatureLayer0": 60,
            "printSpeed": 50,
            "printSpeedLayer0": 20,
            "travelSpeed": 150,
            "fanEnabled": True,
            "coolFanSpeed": 100,
            "coolMinLayerTime": 10.0,
            "retractionEnabled": True,
            "retractionDistance": 5.0,
            "retractionSpeed": 45.0,
            "supportEnabled": False,
            "adhesionType": "none"
        },
        "casting": {
            "temperature": 200.0,
            "volume": 100.0,
            "solidificationTime": 1800,
            "pressure": 5
        },
        "subtractive": {
            "machineModel": "Default_CNC",
            "spindleSpeed": 5000,
            "feedRate": 100,
            "depthOfCut": 1.0,
            "stepOver": 10.0
        }
    },
    "fast": {
        "additive": {
            "layerHeight": 0.3,
            "wallThickness": 0.6,
            "wallLineCount": 1,
            "infillDensity": 30,
            "infillPattern": "grid",
            "topLayers": 2,
            "bottomLayers": 2,
            "roofingLayerCount": 0,
            "flooringLayerCount": 0,
            "nozzleTemperature": 210,
            "bedTemperature": 60,
            "nozzleTemperatureLayer0": 210,
            "bedTemperatureLayer0": 60,
            "printSpeed": 100,
            "printSpeedLayer0": 30,
            "travelSpeed": 200,
            "fanEnabled": True,
            "coolFanSpeed": 100,
            "coolMinLayerTime": 5.0,
            "retractionEnabled": True,
            "retractionDistance": 5.0,
            "retractionSpeed": 60.0,
            "supportEnabled": False,
            "adhesionType": "none"
        },
        "casting": {
            "temperature": 180.0,
            "volume": 100.0,
            "solidificationTime": 900,
            "pressure": 3
        },
        "subtractive": {
            "machineModel": "Default_CNC",
            "spindleSpeed": 8000,
            "feedRate": 200,
            "depthOfCut": 2.0,
            "stepOver": 20.0
        }
    },
    "quality": {
        "additive": {
            "layerHeight": 0.1,
            "wallThickness": 1.2,
            "wallLineCount": 4,
            "infillDensity": 100,
            "infillPattern": "honeycomb",
            "topLayers": 6,
            "bottomLayers": 6,
            "roofingLayerCount": 1,
            "flooringLayerCount": 1,
            "nozzleTemperature": 210,
            "bedTemperature": 60,
            "nozzleTemperatureLayer0": 220,
            "bedTemperatureLayer0": 65,
            "printSpeed": 30,
            "printSpeedLayer0": 15,
            "travelSpeed": 100,
            "fanEnabled": True,
            "coolFanSpeed": 100,
            "coolMinLayerTime": 15.0,
            "retractionEnabled": True,
            "retractionDistance": 6.0,
            "retractionSpeed": 40.0,
            "supportEnabled": True,
            "adhesionType": "brim"
        },
        "casting": {
            "temperature": 220.0,
            "volume": 100.0,
            "solidificationTime": 2400,
            "pressure": 8
        },
        "subtractive": {
            "machineModel": "Default_CNC",
            "spindleSpeed": 3000,
            "feedRate": 50,
            "depthOfCut": 0.5,
            "stepOver": 5.0
        }
    },
    "highTemp450": {
        "additive": {
            "layerHeight": 0.2,
            "wallThickness": 1.0,
            "wallLineCount": 3,
            "infillDensity": 100,
            "infillPattern": "grid",
            "topLayers": 5,
            "bottomLayers": 5,
            "roofingLayerCount": 0,
            "flooringLayerCount": 0,
            "nozzleTemperature": 450,
            "bedTemperature": 100,
            "nozzleTemperatureLayer0": 450,
            "bedTemperatureLayer0": 100,
            "printSpeed": 40,
            "printSpeedLayer0": 15,
            "travelSpeed": 120,
            "fanEnabled": False,
            "coolFanSpeed": 0,
            "coolMinLayerTime": 20.0,
            "retractionEnabled": True,
            "retractionDistance": 6.0,
            "retractionSpeed": 40.0,
            "supportEnabled": False,
            "adhesionType": "raft"
        },
        "casting": {
            "temperature": 450.0,
            "volume": 100.0,
            "solidificationTime": 3600,
            "pressure": 10
        },
        "subtractive": {
            "machineModel": "Default_CNC",
            "spindleSpeed": 5000,
            "feedRate": 100,
            "depthOfCut": 1.0,
            "stepOver": 10.0
        }
    }
}


# ===================== 配置管理器 =====================
class ConfigManager:
    """
    统一配置管理器
    - 管理增材、铸造、减材工艺配置
    - 支持预设和应用
    - 提供切片机配置转换
    - 配置验证和持久化
    """

    def __init__(self, configDir: str = "./configs"):
        """初始化配置管理器"""
        self.configDir = Path(configDir)
        self.configDir.mkdir(exist_ok=True, parents=True)
        logger.info(f"配置管理器初始化，配置目录: {self.configDir}")

    def loadConfig(self, projectId: str) -> Dict[str, Any]:
        """
        加载配置

        Args:
            projectId: 项目ID

        Returns:
            配置字典
        """
        configFile = self.configDir / f"{projectId}.json"

        if configFile.exists():
            try:
                with open(configFile, 'r', encoding='utf-8') as f:
                    config = json.load(f)
                logger.info(f"配置已加载: {projectId}")
                return config
            except Exception as e:
                logger.error(f"加载配置失败: {e}")
                return self._getDefaultConfig()
        else:
            logger.info(f"配置文件不存在，使用默认配置: {projectId}")
            return self._getDefaultConfig()

    def saveConfig(self, projectId: str, config: Dict[str, Any]) -> bool:
        """
        保存配置

        Args:
            projectId: 项目ID
            config: 配置字典

        Returns:
            保存成功返回True
        """
        # 验证配置
        errors = self.validate(config)
        if errors:
            logger.error(f"配置验证失败: {errors}")
            return False

        configFile = self.configDir / f"{projectId}.json"

        try:
            with open(configFile, 'w', encoding='utf-8') as f:
                json.dump(config, f, indent=2, ensure_ascii=False)
            logger.info(f"配置已保存: {projectId}")
            return True
        except Exception as e:
            logger.error(f"保存配置失败: {e}")
            return False

    def validate(self, config: Dict[str, Any]) -> List[str]:
        """
        验证配置

        Args:
            config: 配置字典

        Returns:
            错误列表（为空表示验证通过）
        """
        errors = []

        for section in ["additive", "casting", "subtractive"]:
            if section not in config:
                errors.append(f"缺少工艺配置: {section}")
                continue

            if not isinstance(config[section], dict):
                errors.append(f"工艺配置格式错误: {section}")
                continue

            sectionErrors = self._validateSection(section, config[section])
            errors.extend(sectionErrors)

        return errors

    def _validateSection(self, section: str, params: Dict[str, Any]) -> List[str]:
        """验证单个工艺配置"""
        errors = []
        schema = parameterSchema.get(section, {})

        for paramName, paramSchema in schema.items():
            value = params.get(paramName)

            # 处理缺失的参数
            if value is None:
                if paramSchema.get('default') is not None:
                    params[paramName] = paramSchema['default']
                elif paramSchema.get('required', False):
                    errors.append(f"{section}.{paramName}: 必需参数缺失")
                continue

            # 类型检查
            expectedType = paramSchema['type']
            if not isinstance(value, expectedType):
                # 特殊处理：float可以接受int，bool可以接受字符串
                if expectedType == float and isinstance(value, int):
                    params[paramName] = float(value)
                elif expectedType == bool and isinstance(value, str):
                    try:
                        params[paramName] = value.lower() in ['true', '1', 'yes', 'on']
                    except:
                        errors.append(
                            f"{section}.{paramName}: "
                            f"类型错误 (期望 {expectedType.__name__}, "
                            f"实际 {type(value).__name__})"
                        )
                else:
                    errors.append(
                        f"{section}.{paramName}: "
                        f"类型错误 (期望 {expectedType.__name__}, "
                        f"实际 {type(value).__name__})"
                    )
                    continue

            # 范围检查 (仅对数值)
            if isinstance(value, (int, float)):
                minVal = paramSchema.get('min')
                maxVal = paramSchema.get('max')

                if minVal is not None and value < minVal:
                    errors.append(
                        f"{section}.{paramName}: 值过小 "
                        f"({value} < {minVal})"
                    )

                if maxVal is not None and value > maxVal:
                    errors.append(
                        f"{section}.{paramName}: 值过大 "
                        f"({value} > {maxVal})"
                    )

            # 选项检查 (对枚举值)
            if 'options' in paramSchema:
                if value not in paramSchema['options']:
                    errors.append(
                        f"{section}.{paramName}: "
                        f"无效的选项值 {value}"
                    )

        return errors

    def applyPreset(self, presetName: str, baseConfig: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        """
        应用预设配置

        Args:
            presetName: 预设名称 ('standard', 'fast', 'quality', 'highTemp450')
            baseConfig: 基础配置（可选）

        Returns:
            应用预设后的配置字典
        """
        if presetName not in presets:
            logger.warning(f"未知预设: {presetName}, 使用默认预设")
            presetName = "standard"

        presetConfig = json.loads(json.dumps(presets[presetName]))  # 深拷贝

        if baseConfig:
            # 合并基础配置和预设配置
            resultConfig = {}
            for section in ["additive", "casting", "subtractive"]:
                sectionBase = baseConfig.get(section, {})
                sectionPreset = presetConfig.get(section, {})
                resultConfig[section] = {**sectionBase, **sectionPreset}
            logger.info(f"预设 '{presetName}' 已应用到基础配置")
            return resultConfig
        else:
            logger.info(f"预设 '{presetName}' 已应用")
            return presetConfig

    def generateCuraConfig(self, additiveConfig: Dict[str, Any]) -> Dict[str, str]:
        """
        生成Cura切片机配置

        Args:
            additiveConfig: 增材工艺配置

        Returns:
            Cura格式的配置字典
        """
        curaConfig = {
            # 基础参数
            "layer_height": str(additiveConfig.get("layerHeight", 0.2)),
            "wall_thickness": str(additiveConfig.get("wallThickness", 0.8)),
            "wall_line_count": str(additiveConfig.get("wallLineCount", 2)),
            "infill_density": str(additiveConfig.get("infillDensity", 20)),
            "infill_pattern": additiveConfig.get("infillPattern", "grid"),

            # 层设置
            "top_layers": str(additiveConfig.get("topLayers", 4)),
            "bottom_layers": str(additiveConfig.get("bottomLayers", 4)),
            "roofing_layer_count": str(additiveConfig.get("roofingLayerCount", 0)),
            "flooring_layer_count": str(additiveConfig.get("flooringLayerCount", 0)),

            # 温度设置
            "print_temperature": str(additiveConfig.get("nozzleTemperature", 210)),
            "bed_temperature": str(additiveConfig.get("bedTemperature", 60)),
            "print_temperature_layer_0": str(additiveConfig.get("nozzleTemperatureLayer0", 210)),
            "bed_temperature_layer_0": str(additiveConfig.get("bedTemperatureLayer0", 60)),

            # 速度设置
            "print_speed": str(additiveConfig.get("printSpeed", 50)),
            "print_speed_layer_0": str(additiveConfig.get("printSpeedLayer0", 20)),
            "travel_speed": str(additiveConfig.get("travelSpeed", 150)),

            # 冷却设置
            "fan_enabled": str(additiveConfig.get("fanEnabled", True)).lower(),
            "cool_fan_speed": str(additiveConfig.get("coolFanSpeed", 100)),
            "cool_min_layer_time": str(additiveConfig.get("coolMinLayerTime", 10.0)),

            # 回抽设置
            "retraction_enabled": str(additiveConfig.get("retractionEnabled", True)).lower(),
            "retraction_distance": str(additiveConfig.get("retractionDistance", 5.0)),
            "retraction_speed": str(additiveConfig.get("retractionSpeed", 45.0)),

            # 其他设置
            "support_enabled": str(additiveConfig.get("supportEnabled", False)).lower(),
            "adhesion_type": additiveConfig.get("adhesionType", "none"),
        }

        logger.info("生成Cura切片机配置")
        return curaConfig

    def validateSlicingReadiness(self, config: Dict[str, Any]) -> tuple[bool, list]:
        """
        快速检查切片参数的完整性

        Args:
            config: 配置字典

        Returns:
            (是否就绪, 缺失参数列表)
        """
        requiredParams = [
            "layerHeight",
            "wallThickness",
            "nozzleTemperature"
        ]

        missingParams = []
        additiveConfig = config.get("additive", {})

        for param in requiredParams:
            if param not in additiveConfig or additiveConfig[param] is None:
                missingParams.append(param)

        if missingParams:
            logger.error(f"缺少必要的切片参数: {missingParams}")
            return False, missingParams

        return True, []

    def _getDefaultConfig(self) -> Dict[str, Any]:
        """获取默认配置"""
        return self.applyPreset("standard")

    def getAvailablePresets(self) -> List[str]:
        """获取可用的预设列表"""
        return list(presets.keys())

    def getParameterSchema(self, section: Optional[str] = None) -> Dict:
        """获取参数定义"""
        if section:
            return parameterSchema.get(section, {})
        return parameterSchema


# ===================== 工作流状态 =====================
class WorkflowState(Enum):
    """工作流状态"""
    IDLE = "idle"
    INITIALIZED = "initialized"
    RUNNING = "running"
    PAUSED = "paused"
    COMPLETED = "completed"
    FAILED = "failed"


# ===================== 执行步骤记录 =====================
class ExecutionStep:
    """执行步骤记录"""

    def __init__(self, stepName: str, stepDisplay: str):
        self.stepName = stepName
        self.stepDisplay = stepDisplay
        self.status = "pending"  # pending, running, completed, failed
        self.output = None
        self.error = None
        self.startTime = None
        self.endTime = None

    def toDict(self) -> Dict[str, Any]:
        """转换为字典"""
        return {
            "step": self.stepName,
            "display": self.stepDisplay,
            "status": self.status,
            "output": self.output,
            "error": self.error,
            "startTime": self.startTime,
            "endTime": self.endTime
        }


# ===================== 工作流管理器 =====================
class WorkflowManager:
    """
    工作流管理器
    - 管理三个模块的顺序执行
    - 跟踪执行状态和历史
    - 处理错误和异常
    """

    def __init__(self, configManager: Optional[ConfigManager] = None):
        self.state = WorkflowState.IDLE
        self.projectId = None
        self.config = None
        self.executionHistory = []
        self.currentStepIndex = -1
        self.configManager = configManager or ConfigManager()

        self.steps = [
            ("additive", "增材工艺 - 生成模具"),
            ("casting", "铸造工艺 - 铸造零件"),
            ("subtractive", "减材工艺 - 精加工")
        ]
        logger.info("工作流管理器已初始化")

    def initialize(self, projectId: str, config: Optional[Dict[str, Any]] = None) -> bool:
        """
        初始化工作流

        Args:
            projectId: 项目ID
            config: 工艺配置（如为None则从文件加载）

        Returns:
            初始化成功返回True
        """
        if self.state != WorkflowState.IDLE:
            logger.error(f"工作流状态错误: {self.state.value}")
            return False

        self.projectId = projectId

        # 加载或使用提供的配置
        if config is None:
            self.config = self.configManager.loadConfig(projectId)
        else:
            self.config = config

        self.state = WorkflowState.INITIALIZED
        self.executionHistory = []
        self.currentStepIndex = -1

        logger.info(f"工作流已初始化: {projectId}")
        return True

    def execute(self, moduleRegistry: Dict[str, Callable]) -> bool:
        """
        执行工作流

        Args:
            moduleRegistry: 模块注册表 {模块名: 执行函数}

        执行函数签名:
            def module_func(projectId: str, config: dict) -> dict:
                # 执行工艺
                return {"output_file": "...", "status": "success"}

        Returns:
            执行成功返回True
        """
        if self.state != WorkflowState.INITIALIZED:
            logger.error(
                f"工作流未正确初始化，当前状态: {self.state.value}"
            )
            return False

        self.state = WorkflowState.RUNNING
        logger.info(f"开始执行工作流: {self.projectId}")

        try:
            for index, (stepKey, stepDisplay) in enumerate(self.steps):
                self.currentStepIndex = index

                # 创建执行步骤记录
                stepRecord = ExecutionStep(stepKey, stepDisplay)
                self.executionHistory.append(stepRecord)

                logger.info(f"执行步骤 {index + 1}/3: {stepDisplay}")

                # 检查模块是否存在
                if stepKey not in moduleRegistry:
                    raise KeyError(f"未找到模块: {stepKey}")

                # 获取模块执行函数
                moduleFunc = moduleRegistry[stepKey]
                stepConfig = self.config.get(stepKey, {})

                # 执行模块
                try:
                    stepRecord.status = "running"
                    stepRecord.startTime = datetime.now().isoformat()

                    output = moduleFunc(self.projectId, stepConfig)

                    stepRecord.status = "completed"
                    stepRecord.output = output
                    stepRecord.endTime = datetime.now().isoformat()

                    logger.info(f"步骤成功: {stepDisplay}")

                except Exception as e:
                    stepRecord.status = "failed"
                    stepRecord.error = str(e)
                    stepRecord.endTime = datetime.now().isoformat()

                    logger.error(f"步骤失败: {stepDisplay} - {e}")

                    self.state = WorkflowState.FAILED
                    return False

            self.state = WorkflowState.COMPLETED
            logger.info(f"工作流已完成: {self.projectId}")
            return True

        except Exception as e:
            logger.error(f"工作流执行异常: {e}")
            self.state = WorkflowState.FAILED
            return False

    def getStatus(self) -> Dict[str, Any]:
        """获取工作流状态"""
        return {
            "state": self.state.value,
            "projectId": self.projectId,
            "currentStep": self.currentStepIndex,
            "totalSteps": len(self.steps),
            "progress": f"{self.currentStepIndex + 1}/{len(self.steps)}"
            if self.currentStepIndex >= 0 else "0/3",
            "history": [step.toDict() for step in self.executionHistory]
        }

    def reset(self):
        """重置工作流"""
        self.state = WorkflowState.IDLE
        self.projectId = None
        self.config = None
        self.executionHistory = []
        self.currentStepIndex = -1
        logger.info("工作流已重置")


# ===================== 主程序示例 =====================
if __name__ == "__main__":
    # 配置日志
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )

    print("\n=== 整合后的配置管理示例 ===")

    # 创建配置管理器
    configMgr = ConfigManager("./testConfigs")

    # 应用预设
    config = configMgr.applyPreset("quality")
    print(f"预设 'quality' 已应用")

    # 验证配置
    errors = configMgr.validate(config)
    if not errors:
        print("✓ 配置验证成功")
    else:
        print(f"✗ 配置验证失败: {errors}")

    # 生成Cura配置
    curaConfig = configMgr.generateCuraConfig(config["additive"])
    print(f"✓ 已生成Cura配置，层高: {curaConfig['layer_height']} mm")

    # 检查切片就绪状态
    isReady, missing = configMgr.validateSlicingReadiness(config)
    if isReady:
        print("✓ 切片参数就绪")
    else:
        print(f"✗ 切片参数缺失: {missing}")

    # 应用预设到现有配置
    customConfig = configMgr.applyPreset("standard")
    customConfig["additive"]["layerHeight"] = 0.25
    updatedConfig = configMgr.applyPreset("fast", customConfig)
    print(f"✓ 预设应用到自定义配置，层高: {updatedConfig['additive']['layerHeight']} mm")

    # 保存配置
    success = configMgr.saveConfig("testProject", updatedConfig)
    if success:
        print("✓ 配置已保存")

    print("\n=== 工作流管理示例 ===")


    # 定义模块执行函数
    def mockAdditiveModule(projectId: str, config: Dict) -> Dict:
        """模拟增材模块"""
        curaConfig = configMgr.generateCuraConfig(config)
        print(f"[增材] 生成模具，Cura配置已生成，层高: {config['layerHeight']} mm")
        return {
            "mold_path": f"/output/{projectId}/mold.stl",
            "cura_config": curaConfig,
            "estimated_time": 3600
        }


    def mockCastingModule(projectId: str, config: Dict) -> Dict:
        """模拟铸造模块"""
        print(f"[铸造] 铸造零件，温度: {config['temperature']} °C")
        return {
            "part_path": f"/output/{projectId}/part.stl",
            "casting_time": 1800
        }


    def mockSubtractiveModule(projectId: str, config: Dict) -> Dict:
        """模拟减材模块"""
        print(f"[减材] 精加工，转速: {config['spindleSpeed']} rpm")
        return {
            "gcode_path": f"/output/{projectId}/tool.gcode",
            "machining_time": 900
        }


    # 创建工作流
    workflow = WorkflowManager(configMgr)
    workflow.initialize("testProject", updatedConfig)

    # 注册模块
    modules = {
        "additive": mockAdditiveModule,
        "casting": mockCastingModule,
        "subtractive": mockSubtractiveModule
    }

    # 执行工作流
    success = workflow.execute(modules)

    # 获取状态
    status = workflow.getStatus()
    print(f"\n工作流执行结果: {'成功' if success else '失败'}")
    print(f"最终状态: {status['state']}")

    print("\n=== 整合完成 ===")