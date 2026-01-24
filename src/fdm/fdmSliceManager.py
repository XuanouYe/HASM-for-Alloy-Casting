"""
切片配置管理器 - 管理CuraEngine的切片参数

功能：
- 提供高温打印预设配置（450°C液态金属铸造）
- 支持参数动态覆盖
- 生成CuraEngine可读的JSON配置
- 参数验证和约束检查
"""

import json
import logging
from typing import Dict, Any, Optional, Tuple
from pathlib import Path


class SliceConfigManager:
    """
    切片配置管理器 - 管理和生成CuraEngine的配置文件

    属性:
        config: 当前配置字典
        presets: 预设配置模板
        logger: 日志记录器
    """

    # 参数约束定义
    PARAMETER_CONSTRAINTS = {
        "layer_height": {"min": 0.08, "max": 0.4, "unit": "mm"},
        "wall_thickness": {"min": 0.4, "max": 2.0, "unit": "mm"},
        "infill_density": {"min": 0, "max": 100, "unit": "%"},
        "print_temperature": {"min": 20, "max": 450, "unit": "°C"},
        "bed_temperature": {"min": 20, "max": 100, "unit": "°C"},
        "print_speed": {"min": 10, "max": 150, "unit": "mm/s"},
        "wall_line_count": {"min": 1, "max": 10, "unit": "count"},
        "top_layers": {"min": 0, "max": 20, "unit": "layers"},
        "bottom_layers": {"min": 0, "max": 20, "unit": "layers"},
    }

    def __init__(self):
        """初始化配置管理器"""
        self.logger = logging.getLogger("SliceConfigManager")
        self.config = self._getDefaultConfig()
        self.presets = self._initializePresets()

    def _initializePresets(self) -> Dict[str, Dict]:
        """
        初始化预设配置

        返回:
            预设配置字典
        """
        presets = {
            "high_temp_450": self._getHighTempPreset(),
            "standard": self._getStandardPreset(),
            "fast": self._getFastPreset(),
            "quality": self._getQualityPreset(),
        }

        self.logger.info(f"Loaded {len(presets)} configuration presets")
        return presets

    def _getDefaultConfig(self) -> Dict:
        """
        获取默认配置（标准参数）

        返回:
            默认配置字典
        """
        return {
            # 基础打印参数
            "layer_height": 0.2,  # 层高(mm)
            "wall_thickness": 0.8,  # 壁厚(mm)
            "wall_line_count": 2,  # 壁线数量
            "infill_density": 50,  # 填充密度(%)
            "infill_pattern": "grid",  # 填充模式

            # 顶底层参数
            "top_layers": 4,  # 顶层数
            "bottom_layers": 4,  # 底层数
            "roofing_layer_count": 0,  # 顶面层数
            "flooring_layer_count": 0,  # 底面层数

            # 温度参数
            "print_temperature": 210,  # 喷头温度(°C)
            "bed_temperature": 60,  # 热床温度(°C)
            "print_temperature_layer_0": 210,  # 第一层喷头温度
            "bed_temperature_layer_0": 60,  # 第一层热床温度

            # 打印速度
            "print_speed": 50,  # 基础打印速度(mm/s)
            "print_speed_layer_0": 20,  # 第一层打印速度
            "travel_speed": 150,  # 移动速度(mm/s)

            # 冷却参数
            "cool_min_layer_time": 10,  # 最小层打印时间(s)
            "fan_enabled": True,  # 冷却风扇开启
            "cool_fan_speed": 100,  # 冷却风扇速度(%)

            # 退丝参数
            "retraction_enabled": True,
            "retraction_distance": 5,  # 退丝距离(mm)
            "retraction_speed": 45,  # 退丝速度(mm/s)

            # 其他参数
            "support_enabled": False,
            "adhesion_type": "none",
        }

    def _getHighTempPreset(self) -> Dict:
        """
        高温预设（450°C液态金属铸造模具）

        特点：
        - 高温喷头材料（陶瓷/钨钢）
        - 厚壁设计（提高刚度）
        - 低冷却（高温下容易翘曲）
        - 缓慢打印（确保层间融合）
        """
        config = self._getDefaultConfig()
        config.update({
            "layer_height": 0.25,  # 较厚层高，提高打印速度
            "wall_thickness": 1.2,  # 加厚壁，提高高温强度
            "wall_line_count": 3,  # 多壁线
            "infill_density": 80,  # 高填充，提高刚度
            "infill_pattern": "honeycomb",  # 蜂窝填充

            "top_layers": 6,  # 加厚顶面
            "bottom_layers": 6,  # 加厚底面

            "print_temperature": 240,  # 高温喷头工作温度
            "print_speed": 30,  # 降速至30mm/s
            "print_speed_layer_0": 15,  # 首层更慢

            "fan_enabled": False,  # 关闭冷却风扇（防止翘曲）
            "retraction_distance": 3,  # 减少退丝（高温易拉丝）
            "retraction_speed": 30,

            "cool_min_layer_time": 20,  # 延长每层打印时间
            "cool_fan_speed": 0,
        })

        return config

    def _getStandardPreset(self) -> Dict:
        """标准预设（通用PLA/PETG）"""
        return self._getDefaultConfig()

    def _getFastPreset(self) -> Dict:
        """快速预设（强度要求低，速度优先）"""
        config = self._getDefaultConfig()
        config.update({
            "layer_height": 0.3,
            "wall_thickness": 0.6,
            "wall_line_count": 1,
            "infill_density": 30,
            "print_speed": 80,
            "print_speed_layer_0": 40,
            "cool_min_layer_time": 5,
        })
        return config

    def _getQualityPreset(self) -> Dict:
        """质量预设（精细表面，强度优先）"""
        config = self._getDefaultConfig()
        config.update({
            "layer_height": 0.1,
            "wall_thickness": 1.2,
            "wall_line_count": 4,
            "infill_density": 100,
            "infill_pattern": "grid",
            "print_speed": 30,
            "print_speed_layer_0": 15,
            "cool_min_layer_time": 15,
        })
        return config

    def loadPreset(self, presetName: str) -> None:
        """
        加载预设配置

        参数:
            presetName: 预设名称

        异常:
            ValueError: 预设不存在
        """
        if presetName not in self.presets:
            raise ValueError(
                f"Preset '{presetName}' not found. Available: {list(self.presets.keys())}"
            )

        self.config = self.presets[presetName].copy()
        self.logger.info(f"Loaded preset: {presetName}")

    def setParameter(self, parameterName: str, value: Any) -> None:
        """
        设置单个参数，带约束检查

        参数:
            parameterName: 参数名
            value: 参数值

        异常:
            ValueError: 参数值超出约束范围
        """
        # 约束检查
        if parameterName in self.PARAMETER_CONSTRAINTS:
            constraint = self.PARAMETER_CONSTRAINTS[parameterName]

            if not (constraint["min"] <= value <= constraint["max"]):
                raise ValueError(
                    f"Parameter '{parameterName}' value {value} out of range "
                    f"[{constraint['min']}, {constraint['max']}] {constraint['unit']}"
                )

        self.config[parameterName] = value
        self.logger.debug(f"Set {parameterName} = {value}")

    def setTemperature(self, temperature: float) -> None:
        """
        设置打印温度

        参数:
            temperature: 温度(°C)
        """
        self.setParameter("print_temperature", temperature)

    def setLayerHeight(self, layerHeight: float) -> None:
        """
        设置层高

        参数:
            layerHeight: 层高(mm)
        """
        self.setParameter("layer_height", layerHeight)

    def setInfillDensity(self, density: float) -> None:
        """
        设置填充密度

        参数:
            density: 密度(%)
        """
        self.setParameter("infill_density", density)

    def setPrintSpeed(self, speed: float) -> None:
        """
        设置打印速度

        参数:
            speed: 速度(mm/s)
        """
        self.setParameter("print_speed", speed)

    def setWallThickness(self, thickness: float) -> None:
        """
        设置壁厚

        参数:
            thickness: 壁厚(mm)
        """
        self.setParameter("wall_thickness", thickness)

    def setTopBottomLayers(self, topLayers: int, bottomLayers: int) -> None:
        """
        设置顶底层数

        参数:
            topLayers: 顶层数
            bottomLayers: 底层数
        """
        self.setParameter("top_layers", topLayers)
        self.setParameter("bottom_layers", bottomLayers)

    def setBatchUpdate(self, parameterDict: Dict) -> None:
        """
        批量更新参数

        参数:
            parameterDict: 参数字典
        """
        for paramName, paramValue in parameterDict.items():
            try:
                self.setParameter(paramName, paramValue)
            except ValueError as e:
                self.logger.warning(f"Skipped invalid parameter: {str(e)}")

    def toDict(self) -> Dict:
        """
        获取配置字典（用于JSON导出）

        返回:
            配置字典
        """
        return self.config.copy()

    def toJson(self, indent: int = 2) -> str:
        """
        获取JSON格式配置

        参数:
            indent: JSON缩进层数

        返回:
            JSON字符串
        """
        return json.dumps(self.config, indent=indent, ensure_ascii=False)

    def saveToFile(self, filePath: str) -> None:
        """
        保存配置到JSON文件

        参数:
            filePath: 输出文件路径
        """
        with open(filePath, 'w', encoding='utf-8') as f:
            json.dump(self.config, f, indent=2, ensure_ascii=False)

        self.logger.info(f"Configuration saved to: {filePath}")

    def loadFromFile(self, filePath: str) -> None:
        """
        从JSON文件加载配置

        参数:
            filePath: 输入文件路径

        异常:
            FileNotFoundError: 文件不存在
            json.JSONDecodeError: JSON格式错误
        """
        with open(filePath, 'r', encoding='utf-8') as f:
            loadedConfig = json.load(f)

        self.config.update(loadedConfig)
        self.logger.info(f"Configuration loaded from: {filePath}")

    def getAvailablePresets(self) -> list:
        """
        获取所有可用预设名称

        返回:
            预设名称列表
        """
        return list(self.presets.keys())

    def validateConfig(self) -> Tuple[bool, str]:
        """
        验证当前配置的完整性和合法性

        返回:
            (是否有效, 验证信息)
        """
        requiredParams = [
            "layer_height", "wall_thickness", "infill_density",
            "print_temperature", "print_speed"
        ]

        for param in requiredParams:
            if param not in self.config:
                return False, f"Missing required parameter: {param}"

        # 逻辑检查
        if self.config["wall_thickness"] < self.config["layer_height"]:
            return False, "wall_thickness should be >= layer_height"

        return True, "Configuration is valid"
