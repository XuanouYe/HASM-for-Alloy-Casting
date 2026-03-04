import json
from pathlib import Path
from typing import Dict, Any, List, Optional
import logging

logger = logging.getLogger(__name__)

parameterSchema = {
    "mold": {
        "boundingBoxOffset": {"type": float, "min": 0.1, "max": 100.0, "default": 2.0, "unit": "mm", "required": True},
        "targetFillTime": {"type": float, "min": 0.1, "max": 100.0, "default": 5.0, "unit": "s", "required": False},
        "sprueInletOffset": {"type": float, "min": 0.0, "max": 100.0, "default": 5.0, "unit": "mm", "required": False},
        "enableOrientationOptimization": {"type": bool, "default": False, "required": False},
        "optimizationCriteria": {"type": str, "default": "printability", "options": ["printability", "strength", "material"], "required": False},
        "orientationConstraints": {"type": list, "default": [], "required": False},
        "enableStructureAdjustment": {"type": bool, "default": False, "required": False},
        "adjustmentMethod": {"type": str, "default": "automatic", "options": ["automatic", "manual"], "required": False},
        "minWallThickness": {"type": float, "min": 0.1, "max": 100.0, "default": 2.0, "unit": "mm", "required": False},
        "maxWallThickness": {"type": float, "min": 0.1, "max": 100.0, "default": 20.0, "unit": "mm", "required": False},
        "meshSimplification": {"type": bool, "default": False, "required": False},
        "simplificationTarget": {"type": int, "min": 100, "max": 10000000, "default": 10000, "required": False},
        "meshRepair": {"type": bool, "default": True, "required": False},
        "validateOutput": {"type": bool, "default": True, "required": False},
        "logLevel": {"type": str, "default": "INFO", "options": ["DEBUG", "INFO", "WARNING", "ERROR"], "required": False},
        "logFile": {"type": (str, type(None)), "default": None, "required": False},
    },
    "additive": {
        "layerHeight": {"type": float, "min": 0.08, "max": 0.4, "default": 0.3, "unit": "mm", "required": True},
        "wallThickness": {"type": float, "min": 0.4, "max": 2.0, "default": 0.8, "unit": "mm", "required": True},
        "wallLineCount": {"type": int, "min": 1, "max": 10, "default": 2, "unit": "lines", "required": True},
        "infillDensity": {"type": int, "min": 0, "max": 100, "default": 20, "unit": "%", "required": True},
        "infillPattern": {"type": str, "default": "grid", "options": ["grid", "honeycomb", "gyroid", "cubic", "tetrahedral"], "required": True},
        "topLayers": {"type": int, "min": 0, "max": 50, "default": 4, "required": True},
        "bottomLayers": {"type": int, "min": 0, "max": 50, "default": 4, "required": True},
        "roofingLayerCount": {"type": int, "min": 0, "max": 10, "default": 0, "required": False},
        "flooringLayerCount": {"type": int, "min": 0, "max": 10, "default": 0, "required": False},
        "nozzleTemperature": {"type": int, "min": 180, "max": 450, "default": 210, "unit": "°C", "required": True},
        "bedTemperature": {"type": int, "min": 0, "max": 150, "default": 60, "unit": "°C", "required": True},
        "nozzleTemperatureLayer0": {"type": int, "min": 180, "max": 450, "default": 210, "unit": "°C", "required": False},
        "bedTemperatureLayer0": {"type": int, "min": 0, "max": 150, "default": 60, "unit": "°C", "required": False},
        "printSpeed": {"type": int, "min": 10, "max": 150, "default": 40, "unit": "mm/s", "required": True},
        "printSpeedLayer0": {"type": int, "min": 5, "max": 100, "default": 20, "unit": "mm/s", "required": False},
        "travelSpeed": {"type": int, "min": 50, "max": 300, "default": 150, "unit": "mm/s", "required": False},
        "fanEnabled": {"type": bool, "default": True, "required": False},
        "coolFanSpeed": {"type": int, "min": 0, "max": 100, "default": 100, "unit": "%", "required": False},
        "coolMinLayerTime": {"type": float, "min": 1.0, "max": 60.0, "default": 10.0, "unit": "s", "required": False},
        "retractionEnabled": {"type": bool, "default": True, "required": False},
        "retractionDistance": {"type": float, "min": 0.0, "max": 50.0, "default": 25.0, "unit": "mm", "required": False},
        "retractionSpeed": {"type": float, "min": 10.0, "max": 100.0, "default": 35.0, "unit": "mm/s", "required": False},
        "retractionBufferLength": {"type": float, "min": 0.0, "max": 20.0, "default": 3.0, "unit": "mm", "required": False},
        "retractionReloadSpeed": {"type": float, "min": 10.0, "max": 2000.0, "default": 500.0, "unit": "mm/min", "required": False},
        "supportEnabled": {"type": bool, "default": False, "required": False},
        "adhesionType": {"type": str, "default": "none", "options": ["none", "raft", "brim", "skirt"], "required": False},
        "axisLimits": {"type": dict, "default": {"X": [-100.0, 100.0], "Y": [-100.0, 100.0], "Z": [0.0, 100.0]}, "required": False},
    },
    "casting": {
        "temperature": {"type": float, "min": 50, "max": 500, "default": 200.0, "unit": "°C", "required": True},
        "volume": {"type": float, "min": 1, "max": 10000, "default": 100.0, "unit": "cm³", "required": True},
        "solidificationTime": {"type": int, "min": 10, "max": 7200, "default": 1800, "unit": "s", "required": True},
        "pressure": {"type": int, "min": 0, "max": 100, "default": 5, "unit": "bar", "required": True},
    },
    "subtractive": {
        "machineModel": {"type": str, "default": "Default_CNC", "required": True},
        "spindleSpeed": {"type": int, "min": 100, "max": 60000, "default": 5000, "unit": "rpm", "required": True},
        "feedRate": {"type": int, "min": 1, "max": 5000, "default": 500, "unit": "mm/min", "required": True},
        "toolDiameter": {"type": float, "min": 0.1, "max": 20.0, "default": 6.0, "unit": "mm", "required": True},
        "toolSafetyMargin": {"type": float, "min": 0.01, "max": 5.0, "default": 0.5, "unit": "mm", "required": True},
        "stepOver": {"type": float, "min": 0.01, "max": 10.0, "default": 1.5, "unit": "mm", "required": True},
        "layerStepDown": {"type": float, "min": 0.01, "max": 10.0, "default": 1.0, "unit": "mm", "required": True},
        "waterlineStepDown": {"type": float, "min": 0.01, "max": 5.0, "default": 0.5, "unit": "mm", "required": True},
        "safeHeight": {"type": float, "min": 1.0, "max": 100.0, "default": 5.0, "unit": "mm", "required": True},
        "axisMode": {"type": str, "default": "hemisphere", "options": ["hemisphere", "custom"], "required": True},
        "axisCount": {"type": int, "min": 1, "max": 100, "default": 9, "required": True},
        "angleThreshold": {"type": float, "min": 0.1, "max": 3.14, "default": 1.047, "unit": "rad", "required": True},
        "candidateAxes": {"type": list, "default": [[0.0, 0.0, 1.0]], "required": False},
    },
    "fdm": {
        "wslEnginePath": {"type": str, "default": "", "required": False},
        "definitionFiles": {"type": list, "default": [], "required": False},
        "autoDropToBuildPlate": {"type": bool, "default": True, "required": False},
        "autoCenterXY": {"type": bool, "default": True, "required": False},
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
        else:
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
            sectionErrors = self._validateSection(section, config[section])
            errors.extend(sectionErrors)
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
                elif expectedType == list and isinstance(value, list):
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
            if 'options' in paramSchema:
                if value not in paramSchema['options']:
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
        curaConfig = {
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
            "print_temperature_layer_0": str(additiveConfig.get("nozzleTemperatureLayer0", 210)),
            "bed_temperature_layer_0": str(additiveConfig.get("bedTemperatureLayer0", 60)),
            "print_speed": str(additiveConfig.get("printSpeed", 40)),
            "print_speed_layer_0": str(additiveConfig.get("printSpeedLayer0", 20)),
            "travel_speed": str(additiveConfig.get("travelSpeed", 150)),
            "fan_enabled": str(additiveConfig.get("fanEnabled", True)).lower(),
            "cool_fan_speed": str(additiveConfig.get("coolFanSpeed", 100)),
            "cool_min_layer_time": str(additiveConfig.get("coolMinLayerTime", 10.0)),
            "retraction_enabled": "false",
            "support_enabled": str(additiveConfig.get("supportEnabled", False)).lower(),
            "adhesion_type": additiveConfig.get("adhesionType", "none"),
        }
        return curaConfig

    def validateSlicingReadiness(self, config: Dict[str, Any]) -> tuple[bool, list]:
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
            "boundingBoxOffset": moldConfig.get("boundingBoxOffset"),
            "meshSimplification": moldConfig.get("meshSimplification"),
            "simplificationTarget": moldConfig.get("simplificationTarget"),
            "meshRepair": moldConfig.get("meshRepair"),
            "validateOutput": moldConfig.get("validateOutput"),
        }

    def getGateConfig(self, config: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        moldConfig = config.get("mold", {})
        if not moldConfig.get("enableGate", False):
            return None
        return {
            "gateType": moldConfig.get("gateType"),
            "gateRadius": moldConfig.get("gateRadius"),
            "gateHeight": moldConfig.get("gateHeight"),
            "gatePosition": moldConfig.get("gatePosition"),
        }

    def getSupportConfig(self, config: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        moldConfig = config.get("mold", {})
        if not moldConfig.get("enableSupport", False):
            return None
        return {
            "supportType": moldConfig.get("supportType"),
            "supportDensity": moldConfig.get("supportDensity"),
            "supportAngle": moldConfig.get("supportAngle"),
        }
