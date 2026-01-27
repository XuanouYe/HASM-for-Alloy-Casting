"""
模具生成系统配置管理模块
占位设计：配合上层控制模块集成

此模块设计用于：
1. 管理模具生成系统的所有配置参数
2. 支持配置文件（JSON/YAML）的读取和写入
3. 为上层控制模块提供配置接口
4. 预留扩展接口用于后续功能集成
"""

import json
from typing import Dict, Any, Optional
from pathlib import Path


class MoldGeneratorConfigManager:
    """
    模具生成配置管理器

    职责：
    - 管理所有配置参数的设置和获取
    - 支持配置文件的加载和保存
    - 参数验证和默认值设置
    - 为上层模块提供统一的配置接口
    """

    # 默认配置值
    DEFAULT_CONFIG = {
        # 核心功能参数
        'boundingBoxOffset': 5.0,  # 边界框外扩距离 (mm)

        # 占位设计：浇道功能
        'enableGate': False,
        'gateType': 'cylindrical',  # 'cylindrical', 'conical', 'rectangular'
        'gateRadius': 3.0,  # mm
        'gateHeight': 10.0,  # mm
        'gatePosition': 'center',  # 'center', 'side', 'bottom'

        # 占位设计：支撑计算
        'enableSupport': False,
        'supportType': 'grid',  # 'grid', 'linear', 'custom'
        'supportDensity': 'medium',  # 'light', 'medium', 'heavy'
        'supportAngle': 45.0,  # 悬伸支撑角度 (度)

        # 占位设计：方向优化
        'enableOrientationOptimization': False,
        'optimizationCriteria': 'printability',  # 'printability', 'strength', 'material'
        'orientationConstraints': [],  # 方向约束列表

        # 占位设计：结构调整
        'enableStructureAdjustment': False,
        'adjustmentMethod': 'automatic',  # 'automatic', 'manual'
        'minWallThickness': 2.0,  # 最小壁厚 (mm)
        'maxWallThickness': 20.0,  # 最大壁厚 (mm)

        # 系统参数
        'meshSimplification': False,  # 网格简化
        'simplificationTarget': 10000,  # 目标顶点数
        'meshRepair': True,  # 自动修复网格
        'validateOutput': True,  # 验证输出网格

        # 日志参数
        'logLevel': 'INFO',  # 'DEBUG', 'INFO', 'WARNING', 'ERROR'
        'logFile': None,  # 日志文件路径，None 则仅输出到控制台
    }

    def __init__(self, configPath: Optional[str] = None):
        """
        初始化配置管理器

        Args:
            configPath: 配置文件路径（JSON格式）
                       如果为 None，则使用默认配置
        """
        self.config = self.DEFAULT_CONFIG.copy()

        if configPath:
            self.loadConfig(configPath)

    def loadConfig(self, configPath: str) -> None:
        """
        从文件加载配置

        Args:
            configPath: JSON 配置文件路径

        Raises:
            FileNotFoundError: 配置文件不存在
            json.JSONDecodeError: JSON 格式错误
        """
        configFile = Path(configPath)

        if not configFile.exists():
            raise FileNotFoundError(f"配置文件不存在: {configPath}")

        with open(configFile, 'r', encoding='utf-8') as f:
            loadedConfig = json.load(f)

        # 与默认配置合并（用户配置覆盖默认值）
        self.config.update(loadedConfig)

    def saveConfig(self, configPath: str) -> None:
        """
        将配置保存到文件

        Args:
            configPath: 输出文件路径
        """
        configFile = Path(configPath)
        configFile.parent.mkdir(parents=True, exist_ok=True)

        with open(configFile, 'w', encoding='utf-8') as f:
            json.dump(self.config, f, indent=2, ensure_ascii=False)

    def getConfig(self, key: str, default: Any = None) -> Any:
        """
        获取指定配置值

        Args:
            key: 配置键
            default: 如果键不存在，返回此默认值

        Returns:
            配置值
        """
        return self.config.get(key, default)

    def setConfig(self, key: str, value: Any) -> None:
        """
        设置指定配置值

        Args:
            key: 配置键
            value: 配置值
        """
        self.config[key] = value

    def updateConfig(self, configDict: Dict[str, Any]) -> None:
        """
        批量更新配置

        Args:
            configDict: 包含要更新的配置键值对的字典
        """
        self.config.update(configDict)

    def getAllConfig(self) -> Dict[str, Any]:
        """
        获取所有配置

        Returns:
            完整配置字典
        """
        return self.config.copy()

    def validateConfig(self) -> bool:
        """
        验证配置的合法性

        Returns:
            True 表示配置有效，False 表示有问题
        """
        try:
            # 边界框偏移量检查
            offset = self.config.get('boundingBoxOffset', 5.0)
            if offset <= 0:
                return False

            # 浇道参数检查
            if self.config.get('enableGate', False):
                gateRadius = self.config.get('gateRadius', 3.0)
                if gateRadius <= 0:
                    return False

            # 支撑参数检查
            if self.config.get('enableSupport', False):
                supportAngle = self.config.get('supportAngle', 45.0)
                if supportAngle < 0 or supportAngle > 90:
                    return False

            # 壁厚参数检查
            minWall = self.config.get('minWallThickness', 2.0)
            maxWall = self.config.get('maxWallThickness', 20.0)
            if minWall > maxWall:
                return False

            return True

        except Exception:
            return False

    def resetToDefaults(self) -> None:
        """
        重置配置为默认值
        """
        self.config = self.DEFAULT_CONFIG.copy()

    def printConfig(self) -> None:
        """
        打印当前配置（用于调试）
        """
        print("当前配置清单:")
        print("=" * 60)
        for key, value in sorted(self.config.items()):
            print(f"  {key}: {value}")
        print("=" * 60)


def createExampleConfigFile(filepath: str = "config_example.json") -> None:
    """
    创建示例配置文件

    Args:
        filepath: 输出文件路径
    """
    exampleConfig = {
        "comment": "模具生成系统配置文件示例",
        "version": "1.0.0",

        "# 核心功能": {
            "boundingBoxOffset": 5.0
        },

        "# 浇道功能 (占位设计)": {
            "enableGate": False,
            "gateType": "cylindrical",
            "gateRadius": 3.0,
            "gateHeight": 10.0,
            "gatePosition": "center"
        },

        "# 支撑计算 (占位设计)": {
            "enableSupport": False,
            "supportType": "grid",
            "supportDensity": "medium",
            "supportAngle": 45.0
        },

        "# 方向优化 (占位设计)": {
            "enableOrientationOptimization": False,
            "optimizationCriteria": "printability",
            "orientationConstraints": []
        },

        "# 结构调整 (占位设计)": {
            "enableStructureAdjustment": False,
            "adjustmentMethod": "automatic",
            "minWallThickness": 2.0,
            "maxWallThickness": 20.0
        },

        "# 系统参数": {
            "meshSimplification": False,
            "simplificationTarget": 10000,
            "meshRepair": True,
            "validateOutput": True,
            "logLevel": "INFO",
            "logFile": None
        }
    }

    with open(filepath, 'w', encoding='utf-8') as f:
        json.dump(exampleConfig, f, indent=2, ensure_ascii=False)


# 预留接口：与上层控制模块集成

class MoldGeneratorConfigAdapter:
    """
    配置适配器
    用于与上层控制模块对接

    占位设计：当上层控制模块实现后，通过此适配器进行参数传递
    """

    def __init__(self, configManager: MoldGeneratorConfigManager):
        """
        初始化适配器

        Args:
            configManager: 配置管理器实例
        """
        self.configManager = configManager

    def getConfigForMoldGeneration(self) -> Dict[str, Any]:
        """
        获取用于模具生成的配置参数

        Returns:
            模具生成所需的配置字典
        """
        return {
            'boundingBoxOffset': self.configManager.getConfig('boundingBoxOffset'),
        }

    def getConfigForGateDesign(self) -> Dict[str, Any]:
        """
        获取用于浇道设计的配置参数 (占位)
        """
        if not self.configManager.getConfig('enableGate', False):
            return None

        return {
            'gateType': self.configManager.getConfig('gateType'),
            'gateRadius': self.configManager.getConfig('gateRadius'),
            'gateHeight': self.configManager.getConfig('gateHeight'),
            'gatePosition': self.configManager.getConfig('gatePosition'),
        }

    def getConfigForSupportCalculation(self) -> Dict[str, Any]:
        """
        获取用于支撑计算的配置参数 (占位)
        """
        if not self.configManager.getConfig('enableSupport', False):
            return None

        return {
            'supportType': self.configManager.getConfig('supportType'),
            'supportDensity': self.configManager.getConfig('supportDensity'),
            'supportAngle': self.configManager.getConfig('supportAngle'),
        }

    def getConfigForOrientationOptimization(self) -> Dict[str, Any]:
        """
        获取用于方向优化的配置参数 (占位)
        """
        if not self.configManager.getConfig('enableOrientationOptimization', False):
            return None

        return {
            'optimizationCriteria': self.configManager.getConfig('optimizationCriteria'),
            'orientationConstraints': self.configManager.getConfig('orientationConstraints'),
        }

    def getConfigForStructureAdjustment(self) -> Dict[str, Any]:
        """
        获取用于结构调整的配置参数 (占位)
        """
        if not self.configManager.getConfig('enableStructureAdjustment', False):
            return None

        return {
            'adjustmentMethod': self.configManager.getConfig('adjustmentMethod'),
            'minWallThickness': self.configManager.getConfig('minWallThickness'),
            'maxWallThickness': self.configManager.getConfig('maxWallThickness'),
        }


if __name__ == "__main__":
    """
    配置管理器演示和测试
    """
    # 创建示例配置文件
    createExampleConfigFile("config_example.json")

    # 创建配置管理器
    configManager = MoldGeneratorConfigManager()

    # 打印默认配置
    configManager.printConfig()

    # 验证配置
    if configManager.validateConfig():
        print("\n✓ 配置验证通过")

    # 创建适配器（用于与上层模块集成）
    adapter = MoldGeneratorConfigAdapter(configManager)
    moldGenConfig = adapter.getConfigForMoldGeneration()
    print(f"\n模具生成配置: {moldGenConfig}")