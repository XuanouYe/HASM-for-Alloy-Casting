import time
import posixpath
import subprocess
import struct
from pathlib import Path, PureWindowsPath
from typing import Dict, Optional, List, Tuple, Any
from controlConfig import ConfigManager


class CuraEngineController:
    def __init__(self, wslEnginePath: str):
        self.wslEnginePath = wslEnginePath
        self.lastExecutionTime = 0.0

    def getStlBoundingBox(self, stlPath: str) -> Tuple[float, float, float, float, float, float]:
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

    def windowsPathToWsl(self, windowsPath: str) -> str:
        normalized = str(Path(windowsPath).resolve())
        winPath = PureWindowsPath(normalized)
        drive = winPath.drive.replace(":", "").lower()
        pathParts = winPath.parts[1:]
        return posixpath.join("/mnt", drive, *pathParts)

    def wslPathToWindows(self, wslPath: str) -> str:
        if not wslPath.startswith("/mnt/"):
            return wslPath
        parts = wslPath.split("/")
        drive = parts[2].upper() + ":\\"
        pathParts = parts[3:]
        return str(PureWindowsPath(drive, *pathParts))

    def ensureOutputDirectory(self, outputPath: str) -> str:
        wslPath = self.windowsPathToWsl(outputPath)
        outputDir = posixpath.dirname(wslPath)
        if outputDir:
            subprocess.run(["wsl", "mkdir", "-p", outputDir], capture_output=True, text=True)
        return wslPath

    def applyMachineLimits(self, settings: Dict[str, str], axisLimits: Dict[str, Tuple[float, float]]) -> None:
        if "X" in axisLimits:
            minX, maxX = axisLimits["X"]
            settings["machine_width"] = str(maxX - minX)
            settings["machine_center_is_zero"] = "true" if minX < 0 else "false"
        if "Y" in axisLimits:
            minY, maxY = axisLimits["Y"]
            settings["machine_depth"] = str(maxY - minY)
        if "Z" in axisLimits:
            minZ, maxZ = axisLimits["Z"]
            settings["machine_height"] = str(maxZ - minZ)

    def buildCommandArgs(self, stlPath: str, outputPath: str, definitionFiles: Optional[List[str]] = None,
                         settings: Optional[Dict[str, str]] = None) -> list:
        cmdArgs = ["wsl", self.wslEnginePath, "slice", "-v"]

        if definitionFiles and len(definitionFiles) > 0:
            cmdArgs.extend(["-j", self.windowsPathToWsl(definitionFiles[0])])

        objectSettings = {}
        globalSettings = {}
        if settings:
            for k, v in settings.items():
                if k.startswith("mesh_position_"):
                    objectSettings[k] = v
                else:
                    globalSettings[k] = v

        for k, v in globalSettings.items():
            cmdArgs.extend(["-s", f"{k}={v}"])

        cmdArgs.append("-e0")
        if definitionFiles and len(definitionFiles) > 1:
            cmdArgs.extend(["-j", self.windowsPathToWsl(definitionFiles[1])])

        cmdArgs.extend(["-o", outputPath])
        cmdArgs.extend(["-l", stlPath])

        for k, v in objectSettings.items():
            cmdArgs.extend(["-s", f"{k}={v}"])

        return cmdArgs

    def executeSlice(self, cmdArgs: list) -> None:
        startTime = time.time()
        subprocess.run(cmdArgs, capture_output=True, text=True, encoding="utf-8", errors="replace")
        self.lastExecutionTime = time.time() - startTime

    def replaceExtruderAxis(self, filePath: str) -> None:
        with open(filePath, 'r') as f:
            lines = f.readlines()
        processedLines = []
        for line in lines:
            if line.startswith(('G0', 'G1', 'G2', 'G3', 'G92')):
                lineStripped = line.rstrip('\n')
                parts = lineStripped.split(';')
                commandTokens = parts[0].split()
                newTokens = []
                for token in commandTokens:
                    if token.startswith('E'):
                        newTokens.append(f"C{token[1:]}")
                    else:
                        newTokens.append(token)
                newLine = ' '.join(newTokens)
                if len(parts) > 1:
                    newLine += ' ;' + ';'.join(parts[1:])
                newLine += '\n'
                processedLines.append(newLine)
            else:
                processedLines.append(line)
        with open(filePath, 'w') as f:
            f.writelines(processedLines)

    def generateGcode(self, stlPath: str, outputPath: str, settings: Optional[Dict[str, str]] = None,
                      definitionFiles: Optional[List[str]] = None, autoDropToBuildPlate: bool = True,
                      autoCenterXY: bool = True, axisLimits: Optional[Dict[str, Tuple[float, float]]] = None) -> str:
        wslStlPath = self.windowsPathToWsl(stlPath)
        wslOutputPath = self.ensureOutputDirectory(outputPath)

        if settings is None:
            settings = {}

        if axisLimits:
            self.applyMachineLimits(settings, axisLimits)

        isCenterZero = settings.get("machine_center_is_zero", "false").lower() == "true"

        if autoDropToBuildPlate or autoCenterXY:
            minX, maxX, minY, maxY, minZ, maxZ = self.getStlBoundingBox(stlPath)

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

        cmdArgs = self.buildCommandArgs(stlPath=wslStlPath, outputPath=wslOutputPath, definitionFiles=definitionFiles,
                                        settings=settings)
        self.executeSlice(cmdArgs)
        windowsOutputPath = self.wslPathToWindows(wslOutputPath)
        self.replaceExtruderAxis(windowsOutputPath)
        return windowsOutputPath


def generateGcodeInterface(stlPath: str, outputPath: str, processConfig: Dict[str, Any],
                           axisLimits: Optional[Dict[str, Tuple[float, float]]] = None) -> Dict[str, str]:
    cm = ConfigManager()
    defaultConfig = cm.getDefaultConfig()

    additiveConfig = processConfig.get("additive") or defaultConfig.get("additive") or {}
    settings = cm.generateCuraConfig(additiveConfig)

    enginePath = processConfig.get("wslEnginePath",
                                   "/mnt/c/users/xuanouye/desktop/thesis/04-implementation/pc/external/curaengine/build/release/CuraEngine")

    defs = processConfig.get("definitionFiles", [
        "C:\\Users\\XuanouYe\\Desktop\\Thesis\\04-Implementation\\HASM-for-Alloy-Casting\\external\\Cura\\resources\\definitions\\fdmprinter.def.json",
        "C:\\Users\\XuanouYe\\Desktop\\Thesis\\04-Implementation\\HASM-for-Alloy-Casting\\external\\Cura\\resources\\definitions\\fdmextruder.def.json"
    ])

    autoDrop = processConfig.get("autoDropToBuildPlate", True)
    autoCenter = processConfig.get("autoCenterXY", True)

    if not axisLimits:
        axisLimits = {
            "X": (-100.0, 100.0),
            "Y": (-100.0, 100.0),
            "Z": (0.0, 100.0)
        }

    controller = CuraEngineController(enginePath)

    resultPath = controller.generateGcode(
        stlPath=stlPath,
        outputPath=outputPath,
        settings=settings,
        definitionFiles=defs,
        autoDropToBuildPlate=autoDrop,
        autoCenterXY=autoCenter,
        axisLimits=axisLimits
    )

    return {"gcodePath": resultPath, "status": "success"}


def main(stlPath: str):
    wslEnginePath = "/mnt/c/users/xuanouye/desktop/thesis/04-implementation/pc/external/curaengine/build/release/CuraEngine"
    outputPath = stlPath.replace(".stl", ".gcode")
    definitionFiles = [
        "C:\\Users\\XuanouYe\\Desktop\\Thesis\\04-Implementation\\HASM-for-Alloy-Casting\\external\\Cura\\resources\\definitions\\fdmprinter.def.json",
        "C:\\Users\\XuanouYe\\Desktop\\Thesis\\04-Implementation\\HASM-for-Alloy-Casting\\external\\Cura\\resources\\definitions\\fdmextruder.def.json",
    ]
    settings = {
        "layer_height": "0.2",
        "wall_thickness": "0.8",
        "roofing_layer_count": "0",
        "flooring_layer_count": "0",
        "top_layers": "4",
        "bottom_layers": "4",
    }
    axisLimits = {
        "X": (-100.0, 100.0),
        "Y": (-100.0, 100.0),
        "Z": (0.0, 100.0)
    }

    controller = CuraEngineController(wslEnginePath)
    out = controller.generateGcode(
        stlPath=stlPath,
        outputPath=outputPath,
        settings=settings,
        definitionFiles=definitionFiles,
        autoDropToBuildPlate=True,
        autoCenterXY=True,
        axisLimits=axisLimits
    )
    print(out)


if __name__ == "__main__":
    targetStlPath = "C:\\Users\\XuanouYe\\Desktop\\Thesis\\04-Implementation\\HASM-for-Alloy-Casting\\testModels\\cylinder.mold.stl"
    main(targetStlPath)
