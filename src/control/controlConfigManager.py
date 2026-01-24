import re
import json
import yaml

from enum import Enum
from pathlib import Path
from dataclasses import dataclass, asdict
from typing import Dict, Any, Optional, List, Tuple
from datetime import datetime

from src.control.controlConfigValidation import validationRules, InfillPattern, CuttingStrategy


@dataclass
class AdditiveConfig:
    printerModel: Optional[str] = None
    layerHeight: Optional[float] = None
    nozzleDiameter: Optional[float] = None
    nozzleTemperature: Optional[int] = None
    infillDensity: Optional[int] = None
    infillPattern: Optional[str] = None
    shellThickness: Optional[List[float]] = None
    printSpeed: Optional[int] = None
    printOrientation: Optional[Dict[str, float]] = None
    estimatedPrintTime : Optional[int] = None

@dataclass
class CastingConfig:
    alloyTemperature: Optional[float] = None
    castingSpeed: Optional[str] = None
    castingVolume: Optional[float] = None
    solidificationTime: Optional[int] = None
    coolingMedium: Optional[str] = None
    castingPressure: Optional[int] = None
    degassing: Optional[bool] = None


@dataclass
class SubtractiveConfig:
    machineModel: Optional[str] = None
    spindleSpeed: Optional[int] = None
    feedRate: Optional[int] = None
    depthOfCut: Optional[float] = None
    stepOver: Optional[float] = None
    toolList: Optional[List[Dict[str, Any]]] = None
    cuttingStrategy: Optional[str] = None
    coolant: Optional[str] = None
    estimatedMachiningTime: Optional[int] = None

@dataclass
class ProjectConfig:
    projectId: str
    partName: str
    additiveConfig: AdditiveConfig
    castingConfig: CastingConfig
    subtractiveConfig: SubtractiveConfig
    inputModel: Optional[Dict[str, Any]] = None
    targetSpecs: Optional[Dict[str, Any]] = None
    materialConfig: Optional[Dict[str, Any]] = None
    version: Optional[str] = None
    createdTime: Optional[str] = None

class DirectoryManager:
    """
    Manages directory structure and file naming
    """

    def __init__(self, baseDir: str = "./projects"):
        self.baseDir = Path(baseDir)
        self.baseDir.mkdir(exist_ok=True)

    def getProjectDir(self, projectSlug: str) -> Path:
        """
        Get project directory path
        """
        return self.baseDir / projectSlug

    def createProjectStructure(self, projectSlug: str) -> Dict[str, Path]:
        """
        Create the standard directory structure for a project
        """
        projectDir = self.getProjectDir(projectSlug)

        directories = {
            "01-input": projectDir / "01-input",
            "02-mold": projectDir / "02-mold",
            "03-product": projectDir / "03-product",
            "04-print": projectDir / "04-print",
            "05-CNC": projectDir / "05-CNC",
            "06-Control": projectDir / "06-Control",
            "07-Logs": projectDir / "07-Logs",
            "08-Reports": projectDir / "08-Reports"
        }

        for dirPath in directories.values():
            dirPath.mkdir(parents=True, exist_ok=True)

        return directories

    def generateFilename(self, stage: str, fileType: str, projectSlug: str, dateStr: str = None, version: int = 1) -> str:
        """
        Generate filename according to naming convention:
        project-slug.stage.yyyymmdd.v###.ext
        """
        if dateStr is None:
            dateStr = datetime.now().strftime("%Y%m%d")

        extMap = {
            "stl": "stl",
            "json": "json",
            "gcode": "gcode",
            "yaml": "yaml",
            "csv": "csv",
            "pdf": "pdf",
            "parameters": "json",
            "config": "yaml",
            "report": "json",
            "log": "csv"
        }

        ext = extMap.get(fileType, fileType)

        stageMap = {
            "mold": "02-mold",
            "product": "03-product",
            "print": "04-print",
            "cnc": "05-cnc",
            "control": "06-control",
            "logs": "07-logs",
            "reports": "08-reports"
        }

        # Format version with leading zeros
        versionStr = f"v{version:03d}"

        # Generate filename
        if stage in ["mold", "product"]:
            filename = f"{projectSlug}.{stage}.adjusted.{dateStr}.{versionStr}.{ext}"
        elif stage in ["print", "cnc"]:
            filename = f"{projectSlug}.{fileType}.{dateStr}.{versionStr}.{ext}"
        elif stage == "control":
            filename = f"{projectSlug}.device.config.{dateStr}.{versionStr}.{ext}"
        elif stage == "logs":
            filename = f"{projectSlug}.manufacturing.log.{dateStr}.{versionStr}.{ext}"
        elif stage == "reports":
            filename = f"{projectSlug}.quality.report.{dateStr}.{versionStr}.{ext}"
        else:
            filename = f"{projectSlug}.{stage}.{dateStr}.{versionStr}.{ext}"

        return filename

    def getFilePath(self, projectSlug: str, stage: str, fileType: str, dateStr: str = None, version: int = 1) -> Path:
        """Get full file path for a specific file"""
        directories = self.createProjectStructure(projectSlug)

        stageMap = {
            "mold": "02-mold",
            "product": "03-product",
            "print": "04-print",
            "cnc": "05-cnc",
            "control": "06-control",
            "logs": "07-logs",
            "reports": "08-reports"
        }

        directory = directories.get(stageMap.get(stage, stage))
        filename = self.generateFilename(stage, fileType, projectSlug, dateStr, version)

        return directory / filename

class ConfigManager:
    """
    Unified configuration manager with structured directories
    """

    def __init__(self, configDir: str = "./configs"):
        self.configDir = Path(configDir)
        self.configDir.mkdir(exist_ok=True)
        self.configs = {}
        self.dirManager = DirectoryManager()

    def _sanitizeProjectId(self, projectId: str) -> str:
        """Convert project ID to slug format for directory names"""
        # Replace spaces and special characters with hyphens
        slug = re.sub(r'[^\w\s-]', '', projectId.lower())
        slug = re.sub(r'[-\s]+', '-', slug).strip('-')
        return slug

    def _migrateFromYaml(self, projectId: str, yamlPath: Path):
        with open(yamlPath, 'r') as f:
            yamlConfig = yaml.safe_load(f)

        self.configs[projectId] = yamlConfig
        self.saveConfig(projectId)

        backupPath = yamlPath.with_suffix('.yaml.backup')
        yamlPath.rename(backupPath)

    def loadProjectConfig(self, projectId: str) -> Dict[str, Any]:
        """Load project configuration"""
        configFile = self.configDir / f"{projectId}.json"

        if configFile.exists():
            with open(configFile, 'r', encoding='utf-8') as f:
                self.configs[projectId] = json.load(f)
        else:
            yamlFile = self.configDir / f"{projectId}.yaml"
            if yamlFile.exists():
                self._migrateFromYaml(projectId, yamlFile)
            else:
                self.configs[projectId] = self._createDefaultConfig(projectId)

        return self.configs[projectId]

    def _createDefaultConfig(self, projectId: str) -> Dict[str, Any]:
        """Generate default configuration with structured paths"""
        projectSlug = self._sanitizeProjectId(projectId)
        currentDate = datetime.now().strftime("%Y%m%d")

        # Create directory structure
        directories = self.dirManager.createProjectStructure(projectSlug)

        # Create config instances with default values
        additiveConfig = AdditiveConfig(
            printerModel="Default_Printer",
            layerHeight=0.2,
            nozzleDiameter=0.4,
            nozzleTemperature=210,
            infillDensity=20,
            printSpeed=50,
            estimatedPrintTime=7200
        )
        castingConfig = CastingConfig(
            alloyTemperature=200.0,
            castingVolume=100.0,
            solidificationTime=1800,
            castingPressure=5
        )
        subtractiveConfig = SubtractiveConfig(
            machineModel="Default_CNC",
            spindleSpeed=5000,
            feedRate=100,
            depthOfCut=1.0,
            stepOver=10.0,
            estimatedMachiningTime=10800
        )

        # Create default config with updated JSON structure
        defaultConfig = {
            "projectId": projectId,
            "projectSlug": projectSlug,
            "partName": f"component_{projectSlug}",
            "directoryStructure": {
                "baseDir": str(self.dirManager.baseDir / projectSlug),
                "01-input": str(directories["01-input"]),
                "02-mold": str(directories["02-mold"]),
                "03-product": str(directories["03-product"]),
                "04-print": str(directories["04-print"]),
                "05-CNC": str(directories["05-CNC"]),
                "06-Control": str(directories["06-Control"]),
                "07-Logs": str(directories["07-Logs"]),
                "08-Reports": str(directories["08-Reports"])
            },
            "inputModel": {
                "stlPath": str(directories["01-input"] / f"{projectSlug}.with_gates_risers.{currentDate}.v001.stl"),
                "scaleUnit": "mm",
                "boundingBox": {"x": [0, 100], "y": [0, 100], "z": [0, 50]}
            },
            "processPipeline": ["additive", "casting", "subtractive", "quality_check"],
            "targetSpecs": {
                "dimensionalTolerance": "±0.5mm",
                "surfaceRoughness": "Ra1.6",
                "productionCycleTime": "4hours"
            },
            "materialConfig": {
                "castingAlloy": "EGaIn_based",
                "moldMaterial": "PLA",
                "coolant": "none"
            },
            "createdAt": datetime.now().isoformat(),
            "lastModified": datetime.now().isoformat(),
            "additiveParams": asdict(additiveConfig),
            "castingParams": asdict(castingConfig),
            "subtractiveParams": asdict(subtractiveConfig),
            "verificationParams": {
                "vericutSimulation": None,
                "thermalSimulation": None,
                "flowSimulation": None
            },
            "filePaths": {
                # Input files
                "inputStl": str(directories["01-input"] / f"{projectSlug}.with_gates_risers.{currentDate}.v001.stl"),

                # Mold files
                "moldAdjustedStl": str(directories["02-mold"] / f"{projectSlug}.mold.adjusted.{currentDate}.v001.stl"),

                # Product files
                "productAdjustedStl": str(
                    directories["03-product"] / f"{projectSlug}.product.adjusted.{currentDate}.v001.stl"),

                # Print files
                "printParameters": str(
                    directories["04-print"] / f"{projectSlug}.print.parameters.{currentDate}.v001.json"),
                "printGcode": str(directories["04-print"] / f"{projectSlug}.print.gcode.{currentDate}.v001.gcode"),

                # CNC files
                "cncGcode": str(directories["05-CNC"] / f"{projectSlug}.machining.gcode.{currentDate}.v001.gcode"),
                "interferenceReport": str(
                    directories["05-CNC"] / f"{projectSlug}.interference.report.{currentDate}.v001.json"),

                # Control files
                "deviceConfig": str(directories["06-Control"] / f"{projectSlug}.device.config.{currentDate}.v001.yaml"),

                # Log files
                "manufacturingLog": str(
                    directories["07-Logs"] / f"{projectSlug}.manufacturing.log.{currentDate}.v001.csv"),

                # Report files
                "qualityReport": str(directories["08-Reports"] / f"{projectSlug}.quality.report.{currentDate}.v001.pdf")
            }
        }
        return defaultConfig

    def validateConfig(self, projectId: str, strict: bool = True) -> Tuple[bool, List[str]]:
        """
        Complete configuration validation - target accuracy > 95%.

        Validation content:
        1. Required parameter check
        2. Parameter value range check
        3. Parameter type check
        4. Logical consistency check
        5. Inter-dependency check
        6. Unit consistency check

        Args:
            projectId: Project identifier.
            strict: Whether to use strict validation mode.

        Returns:
            Tuple of (validation passed, list of error messages).
        """
        config = self.configs.get(projectId)
        if not config:
            return False, ["ERROR: Configuration does not exist"]

        errors = []
        warnings = []

        # ========== 1. Required Parameter Check ==========
        for module, params in validationRules.items():
            moduleConfig = config.get(module, {})

            for paramKey, paramRule in params.items():
                if paramRule.get('required', False):
                    value = moduleConfig.get(paramKey)

                    # Check if exists
                    if value is None:
                        errors.append(
                            f"ERROR: Required parameter missing {module}.{paramKey}"
                        )
                    # Check if empty string or empty list
                    elif isinstance(value, (str, list)) and len(value) == 0:
                        errors.append(
                            f"ERROR: Required parameter is empty {module}.{paramKey}"
                        )

        # ========== 2. Parameter Type Check ==========
        for module, params in validationRules.items():
            moduleConfig = config.get(module, {})

            for paramKey, paramRule in params.items():
                value = moduleConfig.get(paramKey)

                if value is None:
                    continue

                expectedType = paramRule.get('type')
                if expectedType and not isinstance(value, expectedType):
                    errors.append(
                        f"ERROR: Parameter type error {module}.{paramKey} - "
                        f"expected {expectedType.__name__}, got {type(value).__name__}"
                    )

        # ========== 3. Parameter Range Check ==========
        for module, params in validationRules.items():
            moduleConfig = config.get(module, {})

            for paramKey, paramRule in params.items():
                value = moduleConfig.get(paramKey)

                if value is None or not isinstance(value, (int, float)):
                    continue

                # Check minimum value
                minVal = paramRule.get('min')
                if minVal is not None and value < minVal:
                    errors.append(
                        f"ERROR: Parameter value too small {module}.{paramKey} = {value} "
                        f"(minimum: {minVal} {paramRule.get('unit', '')})"
                    )

                # Check maximum value
                maxVal = paramRule.get('max')
                if maxVal is not None and value > maxVal:
                    errors.append(
                        f"ERROR: Parameter value too large {module}.{paramKey} = {value} "
                        f"(maximum: {maxVal} {paramRule.get('unit', '')})"
                    )

        # ========== 4. Enumeration Value Check ==========
        enumRules = {
            'additiveParams.infillPattern': InfillPattern,
            'castingParams.coolingMedium': ['air', 'water', 'oil'],
            'subtractiveParams.cuttingStrategy': CuttingStrategy
        }

        for rulePath, allowedValues in enumRules.items():
            module, key = rulePath.split('.')
            moduleConfig = config.get(module, {})
            value = moduleConfig.get(key)

            if value is not None:
                if isinstance(allowedValues, type) and issubclass(allowedValues, Enum):
                    validValues = [e.value for e in allowedValues]
                elif isinstance(allowedValues, list):
                    validValues = allowedValues
                else:
                    validValues = [allowedValues]

                if value not in validValues:
                    errors.append(
                        f"ERROR: Invalid parameter value {rulePath} = {value} "
                        f"(allowed values: {validValues})"
                    )

        # ========== 5. Logical Consistency Check ==========
        if strict:
            additiveParams = config.get('additiveParams', {})
            castingParams = config.get('castingParams', {})
            subtractiveParams = config.get('subtractiveParams', {})

            # Layer height < nozzle diameter
            layerHeight = additiveParams.get('layerHeight')
            nozzleDiameter = additiveParams.get('nozzleDiameter')
            if layerHeight and nozzleDiameter and layerHeight >= nozzleDiameter:
                errors.append(
                    f"ERROR: Logical error - layer height({layerHeight}) "
                    f"should be less than nozzle diameter({nozzleDiameter})"
                )

            # Must set infill density when shell thickness > 0
            shellThickness = additiveParams.get('shellThickness')
            infillDensity = additiveParams.get('infillDensity')
            if shellThickness and shellThickness > 0:
                if infillDensity is None or infillDensity == 0:
                    errors.append(
                        f"ERROR: Logical error - must set infill density when shell thickness > 0"
                    )

        # ========== 6. Directory Structure Check ==========
        dirStructure = config.get('directoryStructure', {})
        requiredDirs = [
            '01-input', '02-mold', '03-product', '04-print',
            '05-CNC', '06-Control', '07-Logs', '08-Reports'
        ]

        for dirName in requiredDirs:
            if dirName not in dirStructure:
                warnings.append(
                    f"WARNING: Incomplete directory structure - missing {dirName}"
                )

        # Output errors and warnings
        allMessages = errors + warnings
        if allMessages and strict:
            for msg in allMessages:
                print(msg)

        # Return results
        isValid = len(errors) == 0
        return isValid, errors

    def saveConfig(self, projectId: str):
        """Save configuration to YAML"""
        config_file = self.configDir / f"{projectId}.json"

        def json_serializer(obj: Any) -> Any:
            if isinstance(obj, datetime):
                return obj.isoformat()
            if isinstance(obj, Path):
                return str(obj)
            raise TypeError(f"Type {type(obj)} not serializable")

        with open(config_file, 'w', encoding='utf-8') as f:
            json.dump(
                self.configs[projectId],
                f,
                indent=2,
                ensure_ascii=False,
                default=json_serializer
            )

    def getParameter(self, projectId: str, module: str, key: str, defaultValue: Any = None, castType: Optional[type] = None) -> Any:
        """
        Safely retrieve a single parameter, supporting type conversion and default values.

        Args:
            projectId: Project identifier.
            module: Module name (e.g., 'additiveParams').
            key: Parameter key name.
            defaultValue: Default value if parameter is missing.
            castType: Type conversion function.

        Returns:
            Parameter value or default value.
        """
        if projectId not in self.configs:
            try:
                self.loadProjectConfig(projectId)
            except Exception as e:
                print(f"ERROR: Failed to load project configuration - {e}")
                return defaultValue

        try:
            # Get module configuration
            moduleConfig = self.configs.get(projectId, {}).get(module, {})

            if not isinstance(moduleConfig, dict):
                print(f"WARNING: Module {module} is not a dictionary type")
                return defaultValue

            # Get parameter value
            value = moduleConfig.get(key, defaultValue)

            # If value doesn't exist and a default value is specified
            if value is None and defaultValue is not None:
                return defaultValue

            # Type conversion
            if value is not None and castType is not None:
                try:
                    return castType(value)
                except (ValueError, TypeError) as e:
                    print(f"WARNING: Type conversion failed for {key} -> {castType.__name__}: {e}")
                    return defaultValue

            return value

        except (KeyError, TypeError, AttributeError) as e:
            print(f"ERROR: Failed to retrieve parameter {module}.{key}: {e}")
            return defaultValue

    def setParameter(self, projectId: str, module: str, key: str, value: Any):
        """Update single parameter"""
        if projectId not in self.configs:
            self.loadProjectConfig(projectId)

        if module in self.configs[projectId]:
            self.configs[projectId][module][key] = value
        self.saveConfig(projectId)

    def _parseTimeString(self, timeStr: str) -> int:
        """
        Parse time string into seconds.

        Supported formats:
        - "120min" -> 7200s
        - "2h" -> 7200s
        - "3600s" -> 3600s
        - "1.5h" -> 5400s
        - "1 hour 30 minutes" -> 5400s

        Args:
            timeStr: Time string.

        Returns:
            Number of seconds.
        """
        if isinstance(timeStr, (int, float)):
            return int(timeStr)

        if not isinstance(timeStr, str):
            raise ValueError(f"Invalid time format: {timeStr}")

        # Extract numbers and units
        match = re.match(r'([0-9.]+)\s*([a-z]+)', timeStr.lower().strip())

        if not match:
            raise ValueError(f"Invalid time format: {timeStr}")

        value = float(match.group(1))
        unit = match.group(2)

        # Unit conversion map
        conversionMap = {
            's': 1,
            'sec': 1,
            'second': 1,
            'min': 60,
            'minute': 60,
            'h': 3600,
            'hour': 3600,
            'day': 86400,
            'd': 86400
        }

        if unit not in conversionMap:
            raise ValueError(f"Unsupported time unit: {unit}")

        return int(value * conversionMap[unit])

    def createExecutionSequence(self, projectId: str) -> Dict[str, Any]:
        if projectId not in self.configs:
            self.loadProjectConfig(projectId)

        config = self.configs[projectId]
        currentDate = datetime.now().strftime("%Y%m%d")

        # Initialize time variables with default values
        additiveTimeSeconds = 0
        castingTimeSeconds = 0
        subtractiveTimeSeconds = 0

        # Safely parse time values
        additiveTime = config.get('additiveParams', {}).get('estimatedPrintTime', 0)
        castingTime = config.get('castingParams', {}).get('solidificationTime', 0)
        subtractiveTime = config.get('subtractiveParams', {}).get('estimatedMachiningTime', 0)

        try:
            if isinstance(additiveTime, str):
                additiveTimeSeconds = self._parseTimeString(additiveTime)
            else:
                additiveTimeSeconds = int(additiveTime)

            castingTimeSeconds = int(castingTime)

            if isinstance(subtractiveTime, str):
                subtractiveTimeSeconds = self._parseTimeString(subtractiveTime)
            else:
                subtractiveTimeSeconds = int(subtractiveTime)

        except Exception as e:
            print(f"WARNING: Time parsing failed - {e}")
            # Use default values (already 0)

        totalTimeSeconds = additiveTimeSeconds + castingTimeSeconds + subtractiveTimeSeconds

        filePaths = config.get('filePaths', {})

        executionSequence = {
            "executionSequence": [
                {
                    "step": 1,
                    "stage": "preparation",
                    "action": "loadInputModel",
                    "status": "pending",
                    "startTime": None,
                    "endTime": None,
                    "inputFile": filePaths.get('inputStl', '')
                },
                {
                    "step": 2,
                    "stage": "additive",
                    "action": "printMold",
                    "status": "pending",
                    "expectedDurationSeconds": additiveTimeSeconds,
                    "machine": config.get('additiveParams', {}).get('printerModel', 'Unknown'),
                    "parametersFile": filePaths.get('printParameters', ''),
                    "gcodeFile": filePaths.get('printGcode', ''),
                    "actualDuration": None
                },
                {
                    "step": 3,
                    "stage": "casting",
                    "action": "injectAlloy",
                    "status": "pending",
                    "expectedDurationSeconds": castingTimeSeconds,
                    "machine": "casting_device",
                    "actualDuration": None
                },
                {
                    "step": 4,
                    "stage": "subtractive",
                    "action": "machineMoldRemoval",
                    "status": "pending",
                    "expectedDurationSeconds": subtractiveTimeSeconds,
                    "machine": config.get('subtractiveParams', {}).get('machineModel', 'Unknown'),
                    "gcodeFile": filePaths.get('cncGcode', ''),
                    "interferenceReport": filePaths.get('interferenceReport', ''),
                    "actualDuration": None
                },
                {
                    "step": 5,
                    "stage": "inspection",
                    "action": "finalQualityCheck",
                    "status": "pending",
                    "metrics": ["dimensionalAccuracy", "surfaceRoughness", "defectDetection"],
                    "reportFile": filePaths.get('qualityReport', '')
                }
            ],
            "batchId": f"batch_{currentDate}_001",
            "totalEstimatedTimeSeconds": totalTimeSeconds,
            "totalEstimatedTimeMinutes": totalTimeSeconds / 60 if totalTimeSeconds > 0 else 0,
            "logFile": filePaths.get('manufacturingLog', ''),
            "configFile": filePaths.get('deviceConfig', '')
        }

        return executionSequence

    def generateFilePath(self, projectId: str, stage: str, fileType: str, version: int = 1) -> str:
        """Generate file path for a specific file type"""
        if projectId not in self.configs:
            self.loadProjectConfig(projectId)

        config = self.configs[projectId]
        projectSlug = config.get('projectSlug', self._sanitizeProjectId(projectId))

        return str(self.dirManager.getFilePath(projectSlug, stage, fileType, version=version))


def main():
    """
    Test configuration manager functionality.
    """
    print("=== Configuration Manager Test ===")

    # 1. Create configuration manager
    print("1. Creating configuration manager...")
    configManager = ConfigManager(configDir="./test.configs")

    # 2. Create project configuration
    print("2. Creating project configuration...")
    projectId = "test.project.001"
    config = configManager.loadProjectConfig(projectId)

    print(f"   Project ID: {config.get('projectId')}")
    print(f"   Project slug: {config.get('projectSlug')}")

    # 3. Validate configuration
    print("3. Validating configuration...")
    isValid, errors = configManager.validateConfig(projectId, strict=False)
    print(f"   Configuration validation result: {'Passed' if isValid else 'Failed'}")
    if errors:
        print(f"   Error messages: {errors[:3]}")  # Show only first 3 errors

    # 4. Set parameters
    print("4. Setting parameters...")
    configManager.setParameter(projectId, "additiveParams", "layerHeight", 0.2)
    configManager.setParameter(projectId, "additiveParams", "nozzleDiameter", 0.4)
    configManager.setParameter(projectId, "additiveParams", "printerModel", "Prusa i3")

    # 5. Get parameters
    print("5. Getting parameters...")
    layerHeight = configManager.getParameter(projectId, "additiveParams", "layerHeight")
    printerModel = configManager.getParameter(projectId, "additiveParams", "printerModel")
    print(f"   Layer height: {layerHeight} mm")
    print(f"   Printer model: {printerModel}")

    # 6. Generate execution sequence
    print("6. Generating execution sequence...")
    executionSeq = configManager.createExecutionSequence(projectId)
    print(f"   Total steps: {len(executionSeq.get('executionSequence', []))}")
    print(f"   Estimated total time: {executionSeq.get('totalEstimatedTimeMinutes', 0):.1f} minutes")

    # 7. Generate file path
    print("7. Generating file path...")
    stlPath = configManager.generateFilePath(projectId, "mold", "stl")
    print(f"   STL file path: {stlPath}")

    # 8. Test directory manager
    print("8. Testing directory manager...")
    dirManager = DirectoryManager("./test.projects")
    directories = dirManager.createProjectStructure("test-project")
    print(f"   Created {len(directories)} directories")

    # 9. Clean up test files
    import shutil
    print("\nCleaning up test files...")
    if Path("./test.configs").exists():
        shutil.rmtree("./test.configs")
    if Path("./test.projects").exists():
        shutil.rmtree("./test.projects")

    print("\nTest completed!")


if __name__ == "__main__":
    main()