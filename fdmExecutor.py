import os
import posixpath
import struct
import subprocess
import time
from pathlib import Path, PureWindowsPath
from typing import Any, Dict, List, Optional, Tuple

from controlConfig import ConfigManager
from dataModel import ManifestManager

defaultWslEnginePath = ""
defaultDefinitionFiles = []
defaultAutoDropToBuildPlate = False
defaultAutoCenterXy = False


class SliceException(Exception):
    pass


class CuraEngineController:
    def __init__(self, wslEnginePath: str):
        self.wslEnginePath = wslEnginePath
        self.lastExecutionTime = 0.0
        self._validateWslEnvironment()

    def _validateWslEnvironment(self) -> None:
        status = subprocess.run(["wsl", "--status"], capture_output=True, text=True, encoding="utf-8")
        if status.returncode != 0:
            raise SliceException("WslNotAvailable")
        exists = subprocess.run(["wsl", "test", "-f", self.wslEnginePath], capture_output=True)
        if exists.returncode != 0:
            raise SliceException("CuraEngineNotFound")

    def _getStlBoundingBox(self, stlPath: str) -> Tuple[float, float, float, float, float, float]:
        with open(stlPath, 'rb') as f:
            f.read(80)
            numTriangles = struct.unpack('<I', f.read(4))[0]
            minX = minY = minZ = float('inf')
            maxX = maxY = maxZ = float('-inf')
            for _ in range(numTriangles):
                f.read(12)
                for _ in range(3):
                    x, y, z = struct.unpack('<fff', f.read(12))
                    minX = min(minX, x)
                    maxX = max(maxX, x)
                    minY = min(minY, y)
                    maxY = max(maxY, y)
                    minZ = min(minZ, z)
                    maxZ = max(maxZ, z)
                f.read(2)
            return (minX, maxX, minY, maxY, minZ, maxZ)

    def _windowsPathToWsl(self, windowsPath: str) -> str:
        normalized = str(Path(windowsPath).resolve())
        winPath = PureWindowsPath(normalized)
        drive = winPath.drive.replace(":", "").lower()
        pathParts = winPath.parts[1:]
        return posixpath.join("/mnt", drive, *pathParts)

    def _wslPathToWindows(self, wslPath: str) -> str:
        if not wslPath.startswith("/mnt/"):
            return wslPath
        parts = wslPath.split("/")
        drive = parts[2].upper() + ":"
        pathParts = parts[3:]
        return str(PureWindowsPath(drive, *pathParts))

    def _validateInputFile(self, filePath: str, expectedExt: str) -> str:
        if not filePath.lower().endswith(expectedExt):
            raise SliceException("InvalidFileFormat")
        wslPath = self._windowsPathToWsl(filePath)
        result = subprocess.run(["wsl", "test", "-f", wslPath], capture_output=True)
        if result.returncode != 0:
            raise SliceException("FileNotFound")
        return wslPath

    def _ensureOutputDirectory(self, outputPath: str) -> str:
        wslPath = self._windowsPathToWsl(outputPath)
        outputDir = posixpath.dirname(wslPath)
        if outputDir:
            subprocess.run(["wsl", "mkdir", "-p", outputDir], capture_output=True, text=True)
        return wslPath

    def _applyMachineLimits(self, settings: Dict[str, str], axisLimits: Dict[str, Tuple[float, float]]) -> None:
        if "X" in axisLimits:
            minX, maxX = axisLimits["X"]
            width = maxX - minX
            settings["machine_width"] = str(width)
            if minX < 0:
                settings["machine_center_is_zero"] = "true"
            else:
                settings["machine_center_is_zero"] = "false"

        if "Y" in axisLimits:
            minY, maxY = axisLimits["Y"]
            depth = maxY - minY
            settings["machine_depth"] = str(depth)

        if "Z" in axisLimits:
            minZ, maxZ = axisLimits["Z"]
            height = maxZ - minZ
            settings["machine_height"] = str(height)

    def _buildCommandArgs(self, stlPath: str, outputPath: str, definitionFiles: Optional[List[str]] = None,
                          settings: Optional[Dict[str, str]] = None) -> list:
        cmdArgs = ["wsl", self.wslEnginePath, "slice", "-v"]
        if definitionFiles:
            for dFile in definitionFiles:
                cmdArgs.extend(["-j", self._windowsPathToWsl(dFile)])
        if settings:
            for k, v in settings.items():
                cmdArgs.extend(["-s", f"{k}={v}"])
        cmdArgs.extend(["-o", outputPath, "-l", stlPath])
        return cmdArgs

    def _executeSlice(self, cmdArgs: list) -> None:
        startTime = time.time()
        result = subprocess.run(cmdArgs, capture_output=True, text=True, encoding="utf-8", errors="replace")
        self.lastExecutionTime = time.time() - startTime
        if result.returncode != 0:
            raise SliceException("CuraEngineExecutionFailed")

    def _validateOutputFile(self, wslPath: str) -> None:
        result = subprocess.run(["wsl", "test", "-s", wslPath], capture_output=True)
        if result.returncode != 0:
            raise SliceException("OutputGenerationFailed")

    def _replaceExtruderAxis(self, filePath: str) -> None:
        with open(filePath, 'r') as f:
            lines = f.readlines()

        processedLines = []
        for line in lines:
            if line.startswith(('G0', 'G1', 'G2', 'G3', 'G92')):
                parts = line.split(';')
                commandTokens = parts[0].split()
                newTokens = []
                for token in commandTokens:
                    if token.startswith('E'):
                        newTokens.append(f"C{token[1:]}")
                    else:
                        newTokens.append(token)

                newLine = " ".join(newTokens)
                if len(parts) > 1:
                    newLine += " ;" + ";".join(parts[1:])
                newLine += "\n"
                processedLines.append(newLine)
            else:
                processedLines.append(line)

        with open(filePath, 'w') as f:
            f.writelines(processedLines)

    def generateGcode(
            self,
            stlPath: str,
            outputPath: str,
            settings: Optional[Dict[str, str]] = None,
            definitionFiles: Optional[List[str]] = None,
            autoDropToBuildPlate: bool = False,
            autoCenterXY: bool = False,
            axisLimits: Optional[Dict[str, Tuple[float, float]]] = None
    ) -> str:
        wslStlPath = self._validateInputFile(stlPath, ".stl")
        wslOutputPath = self._ensureOutputDirectory(outputPath)

        if settings is None:
            settings = {}

        if axisLimits:
            self._applyMachineLimits(settings, axisLimits)
            isCenterZero = settings.get("machine_center_is_zero", "false") == "true"
        else:
            isCenterZero = False

        if autoDropToBuildPlate or autoCenterXY:
            minX, maxX, minY, maxY, minZ, maxZ = self._getStlBoundingBox(stlPath)

            if autoDropToBuildPlate and "mesh_position_z" not in settings:
                settings["mesh_position_z"] = str(-minZ)

            if autoCenterXY:
                if isCenterZero:
                    targetX = 0.0
                    targetY = 0.0
                else:
                    width = float(settings.get("machine_width", "200.0"))
                    depth = float(settings.get("machine_depth", "200.0"))
                    targetX = width / 2.0
                    targetY = depth / 2.0

                currentCenterX = (minX + maxX) / 2.0
                currentCenterY = (minY + maxY) / 2.0

                if "mesh_position_x" not in settings:
                    settings["mesh_position_x"] = str(targetX - currentCenterX)

                if "mesh_position_y" not in settings:
                    settings["mesh_position_y"] = str(targetY - currentCenterY)
        else:
            if "mesh_position_x" not in settings:
                settings["mesh_position_x"] = "0.0"
            if "mesh_position_y" not in settings:
                settings["mesh_position_y"] = "0.0"
            if "mesh_position_z" not in settings:
                settings["mesh_position_z"] = "0.0"

        cmdArgs = self._buildCommandArgs(
            stlPath=wslStlPath,
            outputPath=wslOutputPath,
            definitionFiles=definitionFiles,
            settings=settings
        )
        self._executeSlice(cmdArgs)
        self._validateOutputFile(wslOutputPath)

        windowsOutputPath = self._wslPathToWindows(wslOutputPath)
        self._replaceExtruderAxis(windowsOutputPath)

        return windowsOutputPath


def generateGcodeInterface(
        stlPath: str,
        outputPath: str,
        processConfig: Dict[str, Any],
        axisLimits: Optional[Dict[str, Tuple[float, float]]] = None
) -> str:
    cm = ConfigManager()
    defaultConfig = cm.getDefaultConfig()
    additiveConfig = processConfig.get("additive") or defaultConfig.get("additive") or {}
    settings = cm.generateCuraConfig(additiveConfig)

    enginePath = processConfig.get("wslEnginePath", defaultWslEnginePath)
    if not enginePath:
        raise SliceException("WslEnginePathNotConfigured")

    defs = processConfig.get("definitionFiles", defaultDefinitionFiles)
    autoDrop = processConfig.get("autoDropToBuildPlate", defaultAutoDropToBuildPlate)
    autoCenter = processConfig.get("autoCenterXY", defaultAutoCenterXy)

    controller = CuraEngineController(enginePath)
    return controller.generateGcode(
        stlPath=stlPath,
        outputPath=outputPath,
        settings=settings,
        definitionFiles=defs,
        autoDropToBuildPlate=autoDrop,
        autoCenterXY=autoCenter,
        axisLimits=axisLimits
    )


def main():
    workspace = "workspaceTest"
    manifestPath = os.path.join(workspace, "manifest.json")

    if os.path.exists(manifestPath):
        manifestMgr = ManifestManager.load(manifestPath)
        files = manifestMgr["files"]
        stlPath = files.get("moldShell")
        outPath = files.get("fdmGcode")
    else:
        return

    testConfig = {
        "wslEnginePath": "/mnt/c/users/xuanouye/desktop/thesis/04-implementation/pc/external/curaengine/build/release/CuraEngine",
        "definitionFiles": [
            "C:\\Users\\XuanouYe\\Desktop\\Thesis\\04-Implementation\\PC\\external\\Cura\\resources\\definitions\\fdmprinter.def.json",
            "C:\\Users\\XuanouYe\\Desktop\\Thesis\\04-Implementation\\PC\\external\\Cura\\resources\\definitions\\fdmextruder.def.json",
        ],
        "settings": {
            "layer_height": "0.2",
            "wall_thickness": "0.8",
            "top_layers": "4",
            "bottom_layers": "4",
        },
    }

    axisLimits = {
        "X": (-100.0, 100.0),
        "Y": (-100.0, 100.0),
        "Z": (0.0, 100.0)
    }

    controller = CuraEngineController(testConfig["wslEnginePath"])
    controller.generateGcode(
        stlPath=stlPath,
        outputPath=outPath,
        settings=testConfig["settings"],
        definitionFiles=testConfig["definitionFiles"],
        autoDropToBuildPlate=False,
        autoCenterXY=False,
        axisLimits=axisLimits
    )


if __name__ == "__main__":
    main()
