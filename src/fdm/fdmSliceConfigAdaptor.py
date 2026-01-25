"""
改进的FDM切片配置管理 - 工具函数集合

特点:
- 从类改为工具函数，减少复杂性
- 保持功能完整：配置格式转换（camelCase → snake_case）
- 无状态设计，易于测试和复用
"""

import logging
from typing import Dict, Any

logger = logging.getLogger("fdmSliceConfigManager")


def generateCuraConfig(additiveConfig: Dict[str, Any]) -> Dict[str, str]:
    """
    将AdditiveConfig字典（camelCase）转换为CuraEngine所需的格式（snake_case）

    参数:
        additiveConfig: 来自controlConfigManager的配置字典

    返回:
        CuraEngine格式的设置字典

    示例:
        >>> config = {
        ...     "layerHeight": 0.2,
        ...     "wallThickness": 0.8,
        ...     "printTemperature": 210
        ... }
        >>> result = generateCuraConfig(config)
        >>> result["layer_height"]
        0.2
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

    logger.info("Generated Cura configuration from system config")
    return curaConfig


def validateSlicingReadiness(config: Dict[str, Any]) -> tuple[bool, list]:
    """
    快速检查切片参数的完整性

    参数:
        config: Cura格式的配置字典

    返回:
        (是否就绪, 缺失参数列表)
    """
    requiredParams = [
        "layer_height",
        "wall_thickness",
        "print_temperature"
    ]

    missingParams = []
    for param in requiredParams:
        if param not in config or config[param] is None:
            missingParams.append(param)

    if missingParams:
        logger.error(f"Missing essential slicing parameters: {missingParams}")
        return False, missingParams

    return True, []


def validateConfigFormat(additiveConfig: Dict[str, Any]) -> tuple[bool, list]:
    """
    验证AdditiveConfig的数据类型和值范围

    参数:
        additiveConfig: 要验证的配置字典

    返回:
        (是否有效, 错误列表)
    """
    errors = []

    # 数值范围验证
    validationRules = {
        "layerHeight": (0.08, 0.4),
        "wallThickness": (0.4, 2.0),
        "wallLineCount": (1, 10),
        "infillDensity": (0, 100),
        "topLayers": (0, 20),
        "bottomLayers": (0, 20),
        "nozzleTemperature": (20, 450),
        "bedTemperature": (20, 100),
        "printSpeed": (10, 150),
        "printSpeedLayer0": (5, 100),
        "travelSpeed": (50, 300),
        "coolFanSpeed": (0, 100),
        "retractionDistance": (0.0, 20.0),
        "retractionSpeed": (10.0, 100.0),
    }

    for key, (minVal, maxVal) in validationRules.items():
        if key in additiveConfig:
            value = additiveConfig[key]
            if not isinstance(value, (int, float)):
                errors.append(f"{key} must be numeric, got {type(value).__name__}\")")
            elif not (minVal <= value <= maxVal):
                errors.append(f"{key} out of range [{minVal}, {maxVal}], got {value}")

    # 字符串参数验证
    if "infillPattern" in additiveConfig:
        validPatterns = {"grid", "honeycomb", "gyroid", "cubic", "tetrahedral"}
        pattern = additiveConfig["infillPattern"]
        if pattern not in validPatterns:
            errors.append(f"Invalid infillPattern: {pattern}. Must be one of {validPatterns}")

    if "adhesionType" in additiveConfig:
        validAdhesion = {"none", "raft", "brim", "skirt"}
        adhesion = additiveConfig["adhesionType"]
        if adhesion not in validAdhesion:
            errors.append(f"Invalid adhesionType: {adhesion}. Must be one of {validAdhesion}")

    if errors:
        logger.warning(f"Configuration validation errors: {errors}")
        return False, errors

    logger.info("Configuration format validation passed")
    return True, []


def applyPresetToConfig(preset: str, baseConfig: Dict[str, Any]) -> Dict[str, Any]:
    """
    应用预设到配置字典

    参数:
        preset: 预设名称 ("high_temp_450", "standard", "fast", "quality")
        baseConfig: 基础配置字典

    返回:
        应用预设后的配置字典
        
    异常:
        ValueError: 预设名称无效
    """
    presets = {
        "high_temp_450": {
            "nozzleTemperature": 450,
            "bedTemperature": 100,
            "printSpeed": 40,
            "infillDensity": 100,
        },
        "standard": {
            "nozzleTemperature": 210,
            "bedTemperature": 60,
            "printSpeed": 50,
            "infillDensity": 50,
        },
        "fast": {
            "nozzleTemperature": 210,
            "bedTemperature": 60,
            "printSpeed": 80,
            "infillDensity": 20,
            "topLayers": 2,
            "bottomLayers": 2,
        },
        "quality": {
            "nozzleTemperature": 210,
            "bedTemperature": 60,
            "printSpeed": 30,
            "infillDensity": 80,
            "layerHeight": 0.1,
            "topLayers": 6,
            "bottomLayers": 6,
        },
    }

    if preset not in presets:
        raise ValueError(f"Unknown preset: {preset}. Available: {list(presets.keys())}")

    config = dict(baseConfig)
    config.update(presets[preset])
    logger.info(f"Applied preset '{preset}' to configuration")
    return config
