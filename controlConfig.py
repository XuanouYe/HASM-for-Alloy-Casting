import json
from pathlib import Path
from typing import Dict, Any, List, Optional, Tuple
import logging

logger = logging.getLogger(__name__)

parameterSchema = {
    "mold": {
        "boundingBoxOffset":            {"type": float, "min": 0.1,  "max": 100.0,    "default": 2.0,   "unit": "mm",     "required": True,  "description": "边界框偏移"},
        "targetFillTime":               {"type": float, "min": 0.1,  "max": 100.0,    "default": 5.0,   "unit": "s",      "required": False, "description": "目标充型时间"},
        "sprueInletOffset":             {"type": float, "min": 0.0,  "max": 100.0,    "default": 2.0,   "unit": "mm",     "required": False, "description": "浇口偏移"},
        "enableOrientationOptimization":{"type": bool,                                 "default": False,                   "required": False, "description": "启用方向优化"},
        "optimizationCriteria":         {"type": str,  "options": ["printability","strength","material"], "default": "printability", "required": False, "description": "优化目标"},
        "orientationConstraints":       {"type": list,                                 "default": [],                      "required": False, "description": "方向约束"},
        "enableStructureAdjustment":    {"type": bool,                                 "default": False,                   "required": False, "description": "启用结构调整"},
        "adjustmentMethod":             {"type": str,  "options": ["automatic","manual"], "default": "automatic",          "required": False, "description": "调整方式"},
        "minWallThickness":             {"type": float, "min": 0.1,  "max": 100.0,    "default": 2.0,   "unit": "mm",     "required": False, "description": "最小壁厚"},
        "maxWallThickness":             {"type": float, "min": 0.1,  "max": 100.0,    "default": 20.0,  "unit": "mm",     "required": False, "description": "最大壁厚"},
        "meshSimplification":           {"type": bool,                                 "default": False,                   "required": False, "description": "网格简化"},
        "simplificationTarget":         {"type": int,  "min": 100,   "max": 10000000, "default": 10000,                   "required": False, "description": "简化目标面数"},
        "meshRepair":                   {"type": bool,                                 "default": True,                    "required": False, "description": "网格修复"},
        "validateOutput":               {"type": bool,                                 "default": True,                    "required": False, "description": "验证输出"},
        "logLevel":                     {"type": str,  "options": ["DEBUG","INFO","WARNING","ERROR"], "default": "INFO",  "required": False, "description": "日志级别"},
        "logFile":                      {"type": (str, type(None)),                    "default": None,                    "required": False, "description": "日志文件"},
    },
    "additive": {
        # ── 几何参数 ──────────────────────────────────────────────────────────
        "layerHeight":          {"type": float, "min": 0.08, "max": 0.4,    "default": 0.3,   "unit": "mm",     "required": True,  "description": "层高"},
        "wallThickness":        {"type": float, "min": 0.4,  "max": 10.0,   "default": 0.8,   "unit": "mm",     "required": True,  "description": "壁厚"},
        "wallLineCount":        {"type": int,   "min": 1,    "max": 10,     "default": 2,     "unit": "lines",  "required": True,  "description": "壁线数"},
        "infillDensity":        {"type": int,   "min": 0,    "max": 100,    "default": 20,    "unit": "%",      "required": True,  "description": "填充密度"},
        "infillPattern":        {"type": str,   "options": ["grid","honeycomb","gyroid","cubic","tetrahedral"], "default": "grid", "required": True, "description": "填充图案"},
        "topLayers":            {"type": int,   "min": 0,    "max": 50,     "default": 4,                       "required": True,  "description": "顶层数"},
        "bottomLayers":         {"type": int,   "min": 0,    "max": 50,     "default": 4,                       "required": True,  "description": "底层数"},
        "roofingLayerCount":    {"type": int,   "min": 0,    "max": 10,     "default": 0,                       "required": False, "description": "顶面层数"},
        "flooringLayerCount":   {"type": int,   "min": 0,    "max": 10,     "default": 0,                       "required": False, "description": "底面层数"},
        # ── 温度 ──────────────────────────────────────────────────────────────
        "nozzleTemperature":    {"type": int,   "min": 180,  "max": 450,    "default": 210,   "unit": "°C",     "required": True,  "description": "喷嘴温度"},
        "bedTemperature":       {"type": int,   "min": 0,    "max": 150,    "default": 60,    "unit": "°C",     "required": True,  "description": "热床温度"},
        "nozzleTemperatureLayer0":{"type": int, "min": 180,  "max": 450,    "default": 210,   "unit": "°C",     "required": False, "description": "首层喷嘴温度"},
        "bedTemperatureLayer0": {"type": int,   "min": 0,    "max": 150,    "default": 60,    "unit": "°C",     "required": False, "description": "首层热床温度"},
        # ── 速度 ──────────────────────────────────────────────────────────────
        "printSpeed":           {"type": int,   "min": 1,    "max": 150,    "default": 40,    "unit": "mm/s",   "required": True,  "description": "打印速度"},
        "printSpeedLayer0":     {"type": int,   "min": 1,    "max": 100,    "default": 10,    "unit": "mm/s",   "required": False, "description": "首层打印速度"},
        "travelSpeed":          {"type": int,   "min": 10,   "max": 300,    "default": 150,   "unit": "mm/s",   "required": False, "description": "空走速度"},
        # ── 冷却 ──────────────────────────────────────────────────────────────
        "fanEnabled":           {"type": bool,                                "default": True,                   "required": False, "description": "启用风扇"},
        "coolFanSpeed":         {"type": int,   "min": 0,    "max": 100,    "default": 100,   "unit": "%",      "required": False, "description": "风扇速度"},
        "coolMinLayerTime":     {"type": float, "min": 1.0,  "max": 60.0,   "default": 10.0,  "unit": "s",      "required": False, "description": "最小层时间"},
        # ── 挤出机物理参数（颗粒料适配）─────────────────────────────────────
        "nozzleDiameter":       {"type": float, "min": 0.1,  "max": 5.0,    "default": 0.4,   "unit": "mm",     "required": False, "description": "喷嘴直径"},
        "materialDiameter":     {"type": float, "min": 0.5,  "max": 20.0,   "default": 1.75,  "unit": "mm",     "required": False, "description": "等效线径（颗粒料标定值）"},
        "materialFlow":         {"type": int,   "min": 10,   "max": 500,    "default": 200,   "unit": "%",      "required": False, "description": "挤出流量倍率"},
        "extrusionScaleFactor": {"type": float, "min": 0.01, "max": 100.0,  "default": 2.5,                     "required": False, "description": "E→C轴换算系数（颗粒料标定）"},
        # ── 回抽后处理参数（颗粒料专用，与 CuraEngine 内置回抽完全分离）────
        # 语义说明：
        #   retractionEnabled      → 控制 applyRetractionCompensation 后处理是否执行
        #   CuraEngine retraction  → 在 generateCuraConfig 中始终强制为 false
        #                           （颗粒料螺杆不支持 CuraEngine 内置回抽模型）
        "retractionEnabled":    {"type": bool,                              "default": True,                    "required": False, "description": "启用回抽后处理"},
        "retractionDistance":   {"type": float, "min": 0.0,  "max": 50.0,   "default": 10.0,  "unit": "mm",     "required": False, "description": "回抽距离（螺杆当量）"},
        "retractionSpeed":      {"type": float, "min": 1.0,  "max": 100.0,  "default": 35.0,  "unit": "mm/s",   "required": False, "description": "回抽速度"},
        "retractionBufferLength":{"type": float,"min": 0.0,  "max": 50.0,   "default": 3.0,   "unit": "mm",     "required": False, "description": "回抽触发缓冲长度"},
        "retractionReloadSpeed":{"type": float, "min": 10.0, "max": 2000.0, "default": 500.0, "unit": "mm/min", "required": False, "description": "重载速度"},
        # ── 支撑 / 粘附 ───────────────────────────────────────────────────────
        "supportEnabled":       {"type": bool,                                "default": False,                  "required": False, "description": "启用支撑"},
        "adhesionType":         {"type": str,  "options": ["none","raft","brim","skirt"], "default": "none",    "required": False, "description": "粘附类型"},
        # ── 轴范围 ────────────────────────────────────────────────────────────
        "axisLimits":           {"type": dict,                                "default": {"X": [-100.0, 100.0], "Y": [-100.0, 100.0], "Z": [0.0, 100.0]}, "required": False, "description": "轴限位"},
    },
    "casting": {
        "temperature":          {"type": float, "min": 50,   "max": 500,    "default": 200.0, "unit": "°C",     "required": True,  "description": "浇注温度"},
        "volume":               {"type": float, "min": 1,    "max": 10000,  "default": 100.0, "unit": "cm³",    "required": True,  "description": "浇注体积"},
        "solidificationTime":   {"type": int,   "min": 10,   "max": 7200,   "default": 1800,  "unit": "s",      "required": True,  "description": "凝固时间"},
        "pressure":             {"type": int,   "min": 0,    "max": 100,    "default": 5,     "unit": "bar",    "required": True,  "description": "浇注压力"},
    },
    "subtractive": {
        "machineModel":         {"type": str,                                 "default": "Default_5Axis_CNC",    "required": True,  "description": "机床型号"},
        "spindleSpeed":         {"type": int,   "min": 100,  "max": 60000,  "default": 5000,  "unit": "rpm",    "required": True,  "description": "主轴转速"},
        "feedRate":             {"type": int,   "min": 1,    "max": 5000,   "default": 500,   "unit": "mm/min", "required": True,  "description": "进给速度"},
        "toolDiameter":         {"type": float, "min": 0.1,  "max": 20.0,   "default": 6.0,   "unit": "mm",     "required": True,  "description": "刀具直径"},
        "toolSafetyMargin":     {"type": float, "min": 0.01, "max": 5.0,    "default": 0.5,   "unit": "mm",     "required": True,  "description": "刀具安全余量"},
        "stepOver":             {"type": float, "min": 0.01, "max": 10.0,   "default": 1.5,   "unit": "mm",     "required": True,  "description": "行距"},
        "layerStepDown":        {"type": float, "min": 0.01, "max": 10.0,   "default": 1.0,   "unit": "mm",     "required": True,  "description": "层切深"},
        "waterlineStepDown":    {"type": float, "min": 0.01, "max": 5.0,    "default": 0.5,   "unit": "mm",     "required": True,  "description": "等高线切深"},
        "safeHeight":           {"type": float, "min": 1.0,  "max": 100.0,  "default": 5.0,   "unit": "mm",     "required": True,  "description": "安全高度"},
        "axisMode":             {"type": str,   "options": ["hemisphere","custom"], "default": "hemisphere",    "required": True,  "description": "轴模式"},
        "axisCount":            {"type": int,   "min": 1,    "max": 100,    "default": 9,                       "required": True,  "description": "候选轴数"},
        "angleThreshold":       {"type": float, "min": 0.1,  "max": 3.14,   "default": 1.047, "unit": "rad",    "required": True,  "description": "角度阈值"},
        "candidateAxes":        {"type": list,                                "default": [[0.0, 0.0, 1.0]],      "required": False, "description": "候选轴列表"},
        "unit":                 {"type": str,   "options": ["mm","inch"],    "default": "mm",                   "required": False, "description": "单位"},
        "absoluteMode":         {"type": bool,                                "default": True,                   "required": False, "description": "绝对坐标"},
        "wcsCode":              {"type": str,                                 "default": "G54",                  "required": False, "description": "工件坐标系"},
        "spindleDirection":     {"type": str,   "options": ["CW","CCW"],     "default": "CW",                   "required": False, "description": "主轴方向"},
        "coolantEnabled":       {"type": bool,                                "default": True,                   "required": False, "description": "启用冷却"},
        "coolantCode":          {"type": str,                                 "default": "M8",                   "required": False, "description": "冷却指令"},
        "feedrateMergeStrategy":{"type": bool,                                "default": True,                   "required": False, "description": "进给合并策略"},
        "kinematics": {
            "type": dict,
            "default": {
                "topology": "tiltRotate", "rotationOrder": "XZ",
                "rotationCenter": [0.0, 0.0, 0.0],
                "aAxisLimit": [-120.0, 120.0], "bAxisLimit": [-360.0, 360.0],
                "aAxisName": "A", "bAxisName": "B",
                "aSign": 1.0, "bSign": 1.0
            },
            "required": False, "description": "运动学配置"
        },
        "outputFormat": {
            "type": dict,
            "default": {
                "coordDecimals": 3, "angleDecimals": 3, "feedDecimals": 1,
                "lineNumbers": False, "lineNumberIncrement": 10
            },
            "required": False, "description": "输出格式"
        }
    },
    "fdm": {
        # FIX: wslEnginePath 填 Windows 格式路径，fdmExecutor 负责转换为 WSL 路径
        "wslEnginePath": {
            "type": str,
            "default": "C:\\Users\\XuanouYe\\Desktop\\Thesis\\04-Implementation\\PC\\external\\CuraEngine\\build\\Release\\CuraEngine",
            "required": False,
            "description": "CuraEngine 路径（Windows 格式，自动转 WSL）"
        },
        "definitionFiles":      {"type": list, "default": [], "required": False, "description": "定义文件列表"},
        "autoDropToBuildPlate": {"type": bool, "default": True, "required": False, "description": "自动落到打印床"},
        "autoCenterXY":         {"type": bool, "default": True, "required": False, "description": "XY 自动居中"},
    }
}


class ConfigManager:
    def __init__(self, configDir: str = "./configs"):
        self.configDir = Path(configDir)
        self.configDir.mkdir(exist_ok=True, parents=True)

    def loadConfig(self, projectId: str) -> Dict[str, Any]:
        configFile = self.configDir / f"{projectId}.json"
        if configFile.exists():
            with open(configFile, 'r', encoding='utf-8') as f:
                config = json.load(f)
            return config
        return self.getDefaultConfig()

    def saveConfig(self, projectId: str, config: Dict[str, Any]) -> bool:
        errors = self.validate(config)
        if errors:
            return False
        configFile = self.configDir / f"{projectId}.json"
        with open(configFile, 'w', encoding='utf-8') as f:
            json.dump(config, f, indent=2, ensure_ascii=False)
        return True

    def validate(self, config: Dict[str, Any]) -> List[str]:
        errors = []
        for section in ["additive", "casting", "subtractive", "mold"]:
            if section not in config:
                continue
            if not isinstance(config[section], dict):
                continue
            errors.extend(self._validateSection(section, config[section]))
        return errors

    def _validateSection(self, section: str, params: Dict[str, Any]) -> List[str]:
        errors = []
        schema = parameterSchema.get(section, {})
        for paramName, paramSchema in schema.items():
            value = params.get(paramName)
            if value is None:
                if paramSchema.get('default') is not None:
                    params[paramName] = paramSchema['default']
                elif paramSchema.get('required', False):
                    errors.append(f"{section}.{paramName}")
                continue
            expectedType = paramSchema['type']
            if not isinstance(value, expectedType):
                if expectedType == float and isinstance(value, int):
                    params[paramName] = float(value)
                elif expectedType == bool and isinstance(value, str):
                    params[paramName] = value.lower() in ['true', '1', 'yes', 'on']
                elif expectedType in (list, dict) and isinstance(value, expectedType):
                    pass
                else:
                    errors.append(f"{section}.{paramName}")
                continue
            if isinstance(value, (int, float)):
                minVal = paramSchema.get('min')
                maxVal = paramSchema.get('max')
                if minVal is not None and value < minVal:
                    errors.append(f"{section}.{paramName}")
                if maxVal is not None and value > maxVal:
                    errors.append(f"{section}.{paramName}")
            if 'options' in paramSchema and value not in paramSchema['options']:
                errors.append(f"{section}.{paramName}")
        return errors

    def getDefaultConfig(self) -> Dict[str, Any]:
        config = {}
        for section, params in parameterSchema.items():
            config[section] = {}
            for paramName, paramSchema in params.items():
                if 'default' in paramSchema:
                    config[section][paramName] = paramSchema['default']
        return config

    def getParameterSchema(self, section: Optional[str] = None) -> Dict:
        if section:
            return parameterSchema.get(section, {})
        return parameterSchema

    def generateCuraConfig(self, additiveConfig: Dict[str, Any]) -> Dict[str, str]:
        """
        将 GUI additiveConfig 参数字典转换为 CuraEngine -s 参数字典。

        回抽策略分离（FIX 核心）：
          - CuraEngine 内置 retraction_enabled 始终强制为 "false"
            理由：颗粒料螺杆的挤出/回抽物理模型与 CuraEngine 线材模型不兼容，
                  若开启内置回抽，CuraEngine 在 G 代码中插入 E- 指令，
                  经 replaceExtruderAxis 后变为 C- 指令，
                  再叠加 applyRetractionCompensation 的后处理回抽，
                  导致实际回抽量翻倍。
          - 后处理回抽由 additiveConfig["retractionEnabled"] 单独控制，
            在 fdmExecutor.generateGcode 中判断执行。
        """
        nozzleDia = additiveConfig.get("nozzleDiameter", 0.4)

        curaConfig = {
            # 几何
            "layer_height":              str(additiveConfig.get("layerHeight", 0.3)),
            "wall_thickness":            str(additiveConfig.get("wallThickness", 0.8)),
            "wall_line_count":           str(additiveConfig.get("wallLineCount", 2)),
            "infill_density":            str(additiveConfig.get("infillDensity", 20)),
            "infill_pattern":            additiveConfig.get("infillPattern", "grid"),
            "top_layers":                str(additiveConfig.get("topLayers", 4)),
            "bottom_layers":             str(additiveConfig.get("bottomLayers", 4)),
            "roofing_layer_count":       str(additiveConfig.get("roofingLayerCount", 0)),
            "flooring_layer_count":      str(additiveConfig.get("flooringLayerCount", 0)),
            # 温度
            "print_temperature":         str(additiveConfig.get("nozzleTemperature", 210)),
            "bed_temperature":           str(additiveConfig.get("bedTemperature", 60)),
            "print_temperature_layer_0": str(additiveConfig.get("nozzleTemperatureLayer0", 210)),
            "bed_temperature_layer_0":   str(additiveConfig.get("bedTemperatureLayer0", 60)),
            # 速度
            "print_speed":               str(additiveConfig.get("printSpeed", 40)),
            "speed_layer_0":             str(additiveConfig.get("printSpeedLayer0", 20)),
            "speed_travel":              str(additiveConfig.get("travelSpeed", 150)),
            # 冷却
            "fan_enabled":               str(additiveConfig.get("fanEnabled", True)).lower(),
            "cool_fan_speed":            str(additiveConfig.get("coolFanSpeed", 100)),
            "cool_min_layer_time":       str(additiveConfig.get("coolMinLayerTime", 10.0)),
            # 挤出机物理参数
            "machine_nozzle_size":       str(nozzleDia),
            "line_width":                str(additiveConfig.get("lineWidth", nozzleDia)),
            "material_diameter":         str(additiveConfig.get("materialDiameter", 1.75)),
            "material_flow":             str(additiveConfig.get("materialFlow", 200)),
            # FIX: CuraEngine 内置回抽始终关闭，后处理回抽由 applyRetractionCompensation 负责
            # 若此处为 true，CuraEngine 会在 G 代码插入 E- 回抽指令，
            # 与后处理回抽叠加导致双重回抽，颗粒料螺杆无法承受。
            "retraction_enabled":        "false",
            # 支撑 / 粘附
            "support_enable":            str(additiveConfig.get("supportEnabled", False)).lower(),
            "adhesion_type":             additiveConfig.get("adhesionType", "none"),
        }
        return curaConfig

    def getRetractionConfig(self, additiveConfig: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """
        提取后处理回抽参数，供 fdmExecutor.applyRetractionCompensation 使用。
        若 retractionEnabled=False，返回 None 表示跳过后处理。

        FIX NEW: 将回抽参数提取逻辑集中到 ConfigManager，
                 避免 fdmExecutor 直接散落读取 additiveConfig 键名，
                 同时统一 fallback 到 schema 默认值。
        """
        if not additiveConfig.get("retractionEnabled", False):
            return None

        schema = parameterSchema.get("additive", {})

        def _get(key: str):
            return additiveConfig.get(key, schema.get(key, {}).get("default"))

        return {
            "enabled":      True,
            "bufferLength": float(_get("retractionBufferLength")),
            "distance":     float(_get("retractionDistance")),
            "speed":        float(_get("retractionSpeed")),
            "reloadSpeed":  float(_get("retractionReloadSpeed")),
        }

    def validateSlicingReadiness(self, config: Dict[str, Any]) -> Tuple[bool, List[str]]:
        requiredParams = ["layerHeight", "wallThickness", "nozzleTemperature"]
        missingParams = []
        additiveConfig = config.get("additive", {})
        for param in requiredParams:
            if param not in additiveConfig or additiveConfig[param] is None:
                missingParams.append(param)
        if missingParams:
            return False, missingParams
        return True, []

    def getMoldConfig(self, config: Dict[str, Any]) -> Dict[str, Any]:
        moldConfig = config.get("mold", {})
        return {
            "boundingBoxOffset":    moldConfig.get("boundingBoxOffset"),
            "meshSimplification":   moldConfig.get("meshSimplification"),
            "simplificationTarget": moldConfig.get("simplificationTarget"),
            "meshRepair":           moldConfig.get("meshRepair"),
            "validateOutput":       moldConfig.get("validateOutput"),
        }

    def getGateConfig(self, config: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        moldConfig = config.get("mold", {})
        if not moldConfig.get("enableGate", False):
            return None
        return {
            "gateType":     moldConfig.get("gateType"),
            "gateRadius":   moldConfig.get("gateRadius"),
            "gateHeight":   moldConfig.get("gateHeight"),
            "gatePosition": moldConfig.get("gatePosition"),
        }

    def getSupportConfig(self, config: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        moldConfig = config.get("mold", {})
        if not moldConfig.get("enableSupport", False):
            return None
        return {
            "supportType":    moldConfig.get("supportType"),
            "supportDensity": moldConfig.get("supportDensity"),
            "supportAngle":   moldConfig.get("supportAngle"),
        }
