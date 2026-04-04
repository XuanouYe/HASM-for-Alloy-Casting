import json
from pathlib import Path
from typing import Dict, Any, List, Optional, Tuple

parameterSchema = {
    "mold": {
        "boundingBoxOffset":             {"type": float, "min": 0.1,  "max": 100.0,    "default": 2.0,   "unit": "mm",     "required": True,  "description": "边界框偏移"},
        "targetFillTime":                {"type": float, "min": 0.1,  "max": 100.0,    "default": 5.0,   "unit": "s",      "required": False, "description": "目标充型时间"},
        "sprueInletOffset":              {"type": float, "min": 0.0,  "max": 100.0,    "default": 2.0,   "unit": "mm",     "required": False, "description": "浇口偏移"},
        "enableOrientationOptimization": {"type": bool,                                 "default": False,                   "required": False, "description": "启用方向优化"},
        "optimizationCriteria":          {"type": str,  "options": ["printability","strength","material"], "default": "printability", "required": False, "description": "优化目标"},
        "orientationConstraints":        {"type": list,                                 "default": [],                      "required": False, "description": "方向约束"},
        "enableStructureAdjustment":     {"type": bool,                                 "default": False,                   "required": False, "description": "启用结构调整"},
        "adjustmentMethod":              {"type": str,  "options": ["automatic","manual"], "default": "automatic",          "required": False, "description": "调整方式"},
        "minWallThickness":              {"type": float, "min": 0.1,  "max": 100.0,    "default": 1.0,   "unit": "mm",     "required": False, "description": "最小壁厚"},
        "maxWallThickness":              {"type": float, "min": 0.1,  "max": 100.0,    "default": 20.0,  "unit": "mm",     "required": False, "description": "最大壁厚"},
        "meshSimplification":            {"type": bool,                                 "default": False,                   "required": False, "description": "网格简化"},
        "simplificationTarget":          {"type": int,   "min": 100,   "max": 10000000, "default": 10000,                   "required": False, "description": "简化目标面数"},
        "meshRepair":                    {"type": bool,                                 "default": True,                    "required": False, "description": "网格修复"},
        "validateOutput":                {"type": bool,                                 "default": True,                    "required": False, "description": "验证输出"},
    },
    "additive": {
        "layerHeight":                  {"type": float, "min": 0.08, "max": 0.4,    "default": 0.2,   "unit": "mm",     "required": True,  "description": "层高"},
        "wallThickness":                {"type": float, "min": 0.4,  "max": 10.0,   "default": 0.4,   "unit": "mm",     "required": True,  "description": "壁厚"},
        "wallLineCount":                {"type": int,   "min": 1,    "max": 10,     "default": 2,     "unit": "lines",  "required": True,  "description": "壁线数"},
        "infillDensity":                {"type": int,   "min": 0,    "max": 100,    "default": 5,     "unit": "%",      "required": True,  "description": "填充密度"},
        "infillPattern":                {"type": str,   "options": ["grid","honeycomb","gyroid","cubic","tetrahedral"], "default": "grid", "required": True, "description": "填充图案"},
        "topLayers":                    {"type": int,   "min": 0,    "max": 50,     "default": 1,                       "required": True,  "description": "顶层数"},
        "bottomLayers":                 {"type": int,   "min": 0,    "max": 50,     "default": 4,                       "required": True,  "description": "底层数"},
        "roofingLayerCount":            {"type": int,   "min": 0,    "max": 10,     "default": 0,                       "required": False, "description": "顶面层数"},
        "flooringLayerCount":           {"type": int,   "min": 0,    "max": 10,     "default": 0,                       "required": False, "description": "底面层数"},
        "nozzleTemperature":            {"type": int,   "min": 180,  "max": 450,    "default": 210,   "unit": "°C",     "required": True,  "description": "喷嘴温度"},
        "bedTemperature":               {"type": int,   "min": 0,    "max": 150,    "default": 60,    "unit": "°C",     "required": True,  "description": "热床温度"},
        "nozzleTemperatureLayer0":      {"type": int,   "min": 180,  "max": 450,    "default": 210,   "unit": "°C",     "required": False, "description": "首层喷嘴温度"},
        "bedTemperatureLayer0":         {"type": int,   "min": 0,    "max": 150,    "default": 60,    "unit": "°C",     "required": False, "description": "首层热床温度"},
        "printSpeed":                   {"type": int,   "min": 1,    "max": 150,    "default": 40,    "unit": "mm/s",   "required": True,  "description": "打印速度"},
        "printSpeedLayer0":             {"type": int,   "min": 1,    "max": 100,    "default": 1,     "unit": "mm/s",   "required": False, "description": "首层打印速度"},
        "travelSpeed":                  {"type": int,   "min": 10,   "max": 300,    "default": 150,   "unit": "mm/s",   "required": False, "description": "空走速度"},
        "fanEnabled":                   {"type": bool,                              "default": True,                    "required": False, "description": "启用风扇"},
        "coolFanSpeed":                 {"type": int,   "min": 0,    "max": 100,    "default": 100,   "unit": "%",      "required": False, "description": "风扇速度"},
        "coolMinLayerTime":             {"type": float, "min": 1.0,  "max": 60.0,   "default": 10.0,  "unit": "s",      "required": False, "description": "最小层时间"},
        "nozzleDiameter":               {"type": float, "min": 0.1,  "max": 5.0,    "default": 0.4,   "unit": "mm",     "required": False, "description": "喷嘴直径"},
        "materialDiameter":             {"type": float, "min": 0.5,  "max": 20.0,   "default": 1.75,  "unit": "mm",     "required": False, "description": "等效线径"},
        "materialFlow":                 {"type": int,   "min": 10,   "max": 500,    "default": 100,   "unit": "%",      "required": False, "description": "挤出流量倍率"},
        "extrusionScaleFactor":         {"type": float, "min": 0.01, "max": 100.0,  "default": 2.5,                     "required": False, "description": "E→C轴换算系数"},
        "retractionEnabled":            {"type": bool,                              "default": True,                    "required": False, "description": "启用回抽后处理"},
        "retractionDistance":           {"type": float, "min": 0.0,  "max": 50.0,   "default": 25.0,  "unit": "mm",     "required": False, "description": "回抽距离"},
        "retractionSpeed":              {"type": float, "min": 1.0,  "max": 100.0,  "default": 35.0,  "unit": "mm/s",   "required": False, "description": "回抽速度"},
        "retractionBufferLength":       {"type": float, "min": 0.0,  "max": 50.0,   "default": 3.0,   "unit": "mm",     "required": False, "description": "回抽缓冲长度"},
        "retractionReloadSpeed":        {"type": float, "min": 10.0, "max": 2000.0, "default": 500.0, "unit": "mm/min", "required": False, "description": "重载速度"},
        "retractionReloadExtraRatio":   {"type": float, "min": 0.0,  "max": 0.5,    "default": 0.0,                     "required": False, "description": "重载超量补偿"},
        "retractionMinTravelDistance":  {"type": float, "min": 0.0,  "max": 50.0,   "default": 4.0,   "unit": "mm",     "required": False, "description": "回抽最小空行程"},
        "firstLayerLineWidthFactor":    {"type": int,   "min": 100,  "max": 200,    "default": 120,   "unit": "%",      "required": False, "description": "首层线宽倍率"},
        "firstLayerZOverlap":           {"type": float, "min": 0.0,  "max": 0.3,    "default": 0.2,   "unit": "mm",     "required": False, "description": "首层Z轴压入量"},
        "supportEnabled":               {"type": bool,                               "default": False,                  "required": False, "description": "启用支撑"},
        "adhesionType":                 {"type": str,  "options": ["none","raft","brim","skirt"], "default": "brim",    "required": False, "description": "粘附类型"},
        "axisLimits":                   {"type": dict,                                "default": {"X": [-100.0, 100.0], "Y": [-100.0, 100.0], "Z": [0.0, 100.0]}, "required": False, "description": "轴限位"},
    },
    "casting": {
        "temperature":        {"type": float, "min": 50,  "max": 500,   "default": 200.0, "unit": "°C",   "required": True, "description": "浇注温度"},
        "volume":             {"type": float, "min": 1,   "max": 10000, "default": 100.0, "unit": "cm³",  "required": True, "description": "浇注体积"},
        "solidificationTime": {"type": int,   "min": 10,  "max": 7200,  "default": 1800,  "unit": "s",    "required": True, "description": "凝固时间"},
        "pressure":           {"type": int,   "min": 0,   "max": 100,   "default": 5,     "unit": "bar",  "required": True, "description": "浇注压力"},
    },
    "subtractive": {
        "machineModel":             {"type": str,                                  "default": "Default_5Axis_CNC",    "required": True,  "description": "机床型号"},
        "spindleSpeed":             {"type": int,   "min": 100,  "max": 60000,  "default": 5000,  "unit": "rpm",    "required": True,  "description": "主轴转速"},
        "feedRate":                 {"type": int,   "min": 1,    "max": 5000,   "default": 500,   "unit": "mm/min", "required": True,  "description": "进给速度"},
        "toolDiameter":             {"type": float, "min": 0.1,  "max": 20.0,   "default": 6.0,   "unit": "mm",     "required": True,  "description": "刀具直径"},
        "toolSafetyMargin":         {"type": float, "min": 0.01, "max": 5.0,    "default": 0.5,   "unit": "mm",     "required": True,  "description": "刀具安全余量"},
        "stepOver":                 {"type": float, "min": 0.01, "max": 10.0,   "default": 1.5,   "unit": "mm",     "required": True,  "description": "行距"},
        "layerStepDown":            {"type": float, "min": 0.01, "max": 10.0,   "default": 1.0,   "unit": "mm",     "required": True,  "description": "层切深"},
        "waterlineStepDown":        {"type": float, "min": 0.01, "max": 5.0,    "default": 0.5,   "unit": "mm",     "required": True,  "description": "等高线切深"},
        "safeHeight":               {"type": float, "min": 1.0,  "max": 100.0,  "default": 5.0,   "unit": "mm",     "required": True,  "description": "安全高度"},
        "axisMode":                 {"type": str,   "options": ["hemisphere","custom"], "default": "hemisphere",    "required": True,  "description": "轴模式"},
        "axisCount":                {"type": int,   "min": 1,    "max": 100,    "default": 48,                      "required": True,  "description": "候选轴数"},
        "minAxisZ":                 {"type": float, "min": -1.0, "max": 1.0,    "default": 0.02,                    "required": False, "description": "最小轴Z分量"},
        "angleThreshold":           {"type": float, "min": 0.1,  "max": 3.14,   "default": 1.047, "unit": "rad",    "required": True,  "description": "角度阈值"},
        "shellStepOver":            {"type": float, "min": 0.01, "max": 10.0,   "default": 1.2,   "unit": "mm",     "required": False, "description": "外壳去除行距"},
        "shellLayerStepDown":       {"type": float, "min": 0.01, "max": 10.0,   "default": 1.0,   "unit": "mm",     "required": False, "description": "外壳去除层切深"},
        "shellFeedRate":            {"type": float, "min": 1.0,  "max": 5000.0, "default": 500.0, "unit": "mm/min", "required": False, "description": "外壳去除进给"},
        "shellRoughStock":          {"type": float, "min": 0.0,  "max": 5.0,    "default": 0.0,   "unit": "mm",     "required": False, "description": "外壳粗加工余量"},
        "riserMode":                {"type": str,   "options": ["dropRaster","contour","spiral"], "default": "dropRaster", "required": False, "description": "冒口去除策略"},
        "riserStepOver":            {"type": float, "min": 0.01, "max": 10.0,   "default": 1.0,   "unit": "mm",     "required": False, "description": "冒口去除行距"},
        "riserFeedRate":            {"type": float, "min": 1.0,  "max": 5000.0, "default": 480.0, "unit": "mm/min", "required": False, "description": "冒口去除进给"},
        "finishStepOver":           {"type": float, "min": 0.01, "max": 5.0,    "default": 0.45,  "unit": "mm",     "required": False, "description": "精加工行距"},
        "finishProjectionStep":     {"type": float, "min": 0.01, "max": 5.0,    "default": 0.25,  "unit": "mm",     "required": False, "description": "精加工投影步长"},
        "finishFeedRate":           {"type": float, "min": 1.0,  "max": 5000.0, "default": 425.0, "unit": "mm/min", "required": False, "description": "精加工进给"},
        "finishStock":              {"type": float, "min": 0.0,  "max": 1.0,    "default": 0.03,  "unit": "mm",     "required": False, "description": "精加工余量"},
        "gateStepOver":             {"type": float, "min": 0.01, "max": 10.0,   "default": 1.2,   "unit": "mm",     "required": False, "description": "浇口去除行距"},
        "gateFeedRate":             {"type": float, "min": 1.0,  "max": 5000.0, "default": 500.0, "unit": "mm/min", "required": False, "description": "浇口去除进给"},
        "step3AxisCount":           {"type": int,   "min": 1,    "max": 200,    "default": 16,                      "required": False, "description": "精加工候选轴数"},
        "step3AxisSampleCount":     {"type": int,   "min": 100,  "max": 100000, "default": 16000,                   "required": False, "description": "精加工轴采样数"},
        "step3TargetCoverage":      {"type": float, "min": 0.0,  "max": 1.0,    "default": 0.995,                   "required": False, "description": "精加工目标覆盖率"},
        "step3AxisDiversityDot":    {"type": float, "min": 0.0,  "max": 1.0,    "default": 0.985,                   "required": False, "description": "精加工轴多样性阈值"},
        "directLinkThreshold":      {"type": float, "min": 0.0,  "max": 1000.0, "default": 2.0,                     "required": False, "description": "直连阈值"},
        "maxRetractOffset":         {"type": float, "min": 0.0,  "max": 5000.0, "default": 100.0,                   "required": False, "description": "抬刀偏置"},
        "rotationChangeThreshold":  {"type": float, "min": 0.0,  "max": 180.0,  "default": 5.0,                     "required": False, "description": "姿态变化阈值"},
        "rotationRetractAngle":     {"type": float, "min": 0.0,  "max": 180.0,  "default": 30.0,                    "required": False, "description": "旋转抬刀角度"},
        "rotationSafeZ":            {"type": float, "min": -1000.0, "max": 5000.0, "default": 30.0,                 "required": False, "description": "旋转安全高度"},
        "linkFeedRate":             {"type": float, "min": 0.0,  "max": 50000.0, "default": 2000.0,                 "required": False, "description": "联接进给"},
        "candidateAxes":            {"type": list,                                "default": [[0.0, 0.0, 1.0]],      "required": False, "description": "候选轴列表"},
        "unit":                     {"type": str,   "options": ["mm","inch"],    "default": "mm",                   "required": False, "description": "单位"},
        "absoluteMode":             {"type": bool,                                "default": True,                   "required": False, "description": "绝对坐标"},
        "wcsCode":                  {"type": str,                                 "default": "G55",                  "required": False, "description": "工件坐标系"},
        "spindleDirection":         {"type": str,   "options": ["CW","CCW"],     "default": "CW",                   "required": False, "description": "主轴方向"},
        "coolantEnabled":           {"type": bool,                                "default": True,                   "required": False, "description": "启用冷却"},
        "coolantCode":              {"type": str,                                 "default": "M8",                   "required": False, "description": "冷却指令"},
        "feedrateMergeStrategy":    {"type": bool,                                "default": True,                   "required": False, "description": "进给合并策略"},
        "enableStep1ShellRemoval":  {"type": bool,                                "default": True,                   "required": False, "description": "Step1 模壳去除"},
        "enableStep2RiserRemoval":  {"type": bool,                                "default": True,                   "required": False, "description": "Step2 冒口去除"},
        "enableStep3PartFinishing": {"type": bool,                                "default": True,                   "required": False, "description": "Step3 零件精加工"},
        "enableStep4GateRemoval":   {"type": bool,                                "default": True,                   "required": False, "description": "Step4 浇口去除"},
        "kinematics": {
            "type": dict,
            "default": {
                "machineType": "xyzac-trt",
                "aAxisName": "A",
                "cAxisName": "C",
                "aSign": 1.0,
                "cSign": 1.0,
                "aAxisLimit": [-120.0, 120.0],
                "cAxisLimit": [-360.0, 360.0],
                "workOffsetX": 0.0,
                "workOffsetY": 0.0,
                "workOffsetZ": 0.0,
                "aAxisOffsetY": 0.0,
                "aAxisOffsetZ": 90.0383,
                "singularityEps": 0.01
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
        for section in parameterSchema:
            if section not in config or not isinstance(config[section], dict):
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
        nozzleDia = additiveConfig.get("nozzleDiameter", 0.4)
        return {
            "layer_height": str(additiveConfig.get("layerHeight", 0.3)),
            "wall_thickness": str(additiveConfig.get("wallThickness", 0.8)),
            "wall_line_count": str(additiveConfig.get("wallLineCount", 2)),
            "infill_density": str(additiveConfig.get("infillDensity", 20)),
            "infill_pattern": additiveConfig.get("infillPattern", "grid"),
            "top_layers": str(additiveConfig.get("topLayers", 4)),
            "bottom_layers": str(additiveConfig.get("bottomLayers", 4)),
            "roofing_layer_count": str(additiveConfig.get("roofingLayerCount", 0)),
            "flooring_layer_count": str(additiveConfig.get("flooringLayerCount", 0)),
            "print_temperature": str(additiveConfig.get("nozzleTemperature", 210)),
            "bed_temperature": str(additiveConfig.get("bedTemperature", 60)),
            "material_print_temperature_layer_0": str(additiveConfig.get("nozzleTemperatureLayer0", 210)),
            "material_bed_temperature_layer_0": str(additiveConfig.get("bedTemperatureLayer0", 65)),
            "print_speed": str(additiveConfig.get("printSpeed", 40)),
            "speed_print_layer_0": str(additiveConfig.get("printSpeedLayer0", 5)),
            "speed_travel_layer_0": str(additiveConfig.get("printSpeedLayer0", 5)),
            "speed_travel": str(additiveConfig.get("travelSpeed", 150)),
            "fan_enabled": str(additiveConfig.get("fanEnabled", True)).lower(),
            "cool_fan_speed": str(additiveConfig.get("coolFanSpeed", 100)),
            "cool_min_layer_time": str(additiveConfig.get("coolMinLayerTime", 10.0)),
            "machine_nozzle_size": str(nozzleDia),
            "line_width": str(additiveConfig.get("lineWidth", nozzleDia)),
            "material_diameter": str(additiveConfig.get("materialDiameter", 1.75)),
            "material_flow": str(additiveConfig.get("materialFlow", 200)),
            "retraction_enabled": "false",
            "support_enable": str(additiveConfig.get("supportEnabled", False)).lower(),
            "adhesion_type": additiveConfig.get("adhesionType", "none"),
            "initial_layer_line_width_factor": str(additiveConfig.get("firstLayerLineWidthFactor", 120)),
            "layer_0_z_overlap": str(additiveConfig.get("firstLayerZOverlap", 0.1)),
        }

    def getRetractionConfig(self, additiveConfig: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        if not additiveConfig.get("retractionEnabled", False):
            return None
        schema = parameterSchema["additive"]

        def _get(key: str):
            return additiveConfig.get(key, schema[key]["default"])

        return {
            "enabled":          True,
            "bufferLength":     float(_get("retractionBufferLength")),
            "distance":         float(_get("retractionDistance")),
            "speed":            float(_get("retractionSpeed")),
            "reloadSpeed":      float(_get("retractionReloadSpeed")),
            "reloadExtraRatio": float(_get("retractionReloadExtraRatio")),
            "minTravelDist":    float(_get("retractionMinTravelDistance")),
        }

    def validateSlicingReadiness(self, config: Dict[str, Any]) -> Tuple[bool, List[str]]:
        requiredParams = ["layerHeight", "wallThickness", "nozzleTemperature"]
        additiveConfig = config.get("additive", {})
        missingParams = [p for p in requiredParams if not additiveConfig.get(p)]
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