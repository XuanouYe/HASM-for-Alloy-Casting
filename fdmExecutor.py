import time
import posixpath
import subprocess
import struct
from pathlib import Path, PureWindowsPath
from typing import Dict, Optional, List, Tuple


class CuraEngineController:
    def __init__(self, wslEnginePath: str):
        self.wslEnginePath = wslEnginePath
        self.lastExecutionTime = 0.0
        self._validateWslEnvironment()

    def _validateWslEnvironment(self) -> None:
        subprocess.run(["wsl", "--status"], capture_output=True, text=True, encoding="utf-8", errors="ignore",
                       timeout=5)
        subprocess.run(["wsl", "test", "-f", self.wslEnginePath], capture_output=True, timeout=5)

    def _getStlBoundingBox(self, stlPath: str) -> Tuple[float, float, float, float, float, float]:
        with open(stlPath, 'rb') as f:
            header = f.read(80)
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
        wslPath = posixpath.join("/mnt", drive, *pathParts)
        return wslPath

    def _wslPathToWindows(self, wslPath: str) -> str:
        if not wslPath.startswith("/mnt/"):
            return wslPath
        parts = wslPath.split("/")
        drive = parts[2].upper() + ":"
        pathParts = parts[3:]
        winPath = str(PureWindowsPath(drive, *pathParts))
        return winPath

    def _validateInputFile(self, filePath: str, expectedExt: str) -> str:
        wslPath = self._windowsPathToWsl(filePath)
        subprocess.run(["wsl", "test", "-f", wslPath], capture_output=True, timeout=5)
        return wslPath

    def _ensureOutputDirectory(self, outputPath: str) -> str:
        wslPath = self._windowsPathToWsl(outputPath)
        outputDir = posixpath.dirname(wslPath)
        if outputDir:
            subprocess.run(["wsl", "mkdir", "-p", outputDir], capture_output=True, text=True, timeout=5)
        return wslPath

    def _buildCommandArgs(self, stlPath: str, outputPath: str, definitionFiles: Optional[List[str]] = None,
                          settings: Optional[Dict[str, str]] = None) -> list:
        cmdArgs = ["wsl", self.wslEnginePath, "slice", "-v"]
        if definitionFiles:
            for f in definitionFiles:
                cmdArgs.extend(["-j", self._windowsPathToWsl(f)])
        if settings:
            for k, v in settings.items():
                cmdArgs.extend(["-s", f"{k}={v}"])
        cmdArgs.extend(["-o", outputPath, "-l", stlPath])
        return cmdArgs

    def _executeSlice(self, cmdArgs: list) -> None:
        startTime = time.time()
        subprocess.run(cmdArgs, capture_output=True, text=True, encoding="utf-8", errors="replace", timeout=600)
        self.lastExecutionTime = time.time() - startTime

    def _validateOutputFile(self, wslPath: str) -> None:
        subprocess.run(["wsl", "test", "-s", wslPath], capture_output=True, timeout=5)

    def generateGcode(self, stlPath: str, outputPath: str, settings: Optional[Dict[str, str]] = None,
                      definitionFiles: Optional[List[str]] = None, autoDropToBuildPlate: bool = True) -> str:
        wslStlPath = self._validateInputFile(stlPath, ".stl")
        wslOutputPath = self._ensureOutputDirectory(outputPath)

        if autoDropToBuildPlate:
            minX, maxX, minY, maxY, minZ, maxZ = self._getStlBoundingBox(stlPath)
            if settings is None:
                settings = {}
            if "mesh_position_z" not in settings:
                settings["mesh_position_z"] = str(-minZ)

        cmdArgs = self._buildCommandArgs(stlPath=wslStlPath, outputPath=wslOutputPath, definitionFiles=definitionFiles,
                                         settings=settings)
        self._executeSlice(cmdArgs)
        self._validateOutputFile(wslOutputPath)
        return self._wslPathToWindows(wslOutputPath)


def main(stlPath: str):
    wslEnginePath = "/mnt/c/users/xuanouye/desktop/thesis/04-implementation/pc/external/curaengine/build/release/CuraEngine"
    outputPath = stlPath.replace(".stl", ".gcode")
    definitionFiles = [
        "C:\\Users\\XuanouYe\\Desktop\\Thesis\\04-Implementation\\PC\\external\\Cura\\resources\\definitions\\fdmprinter.def.json",
        "C:\\Users\\XuanouYe\\Desktop\\Thesis\\04-Implementation\\PC\\external\\Cura\\resources\\definitions\\fdmextruder.def.json",
    ]
    settings = {
        "layer_height": "0.2",
        "wall_thickness": "0.8",
        "roofing_layer_count": "0",
        "flooring_layer_count": "0",
        "top_layers": "4",
        "bottom_layers": "4",
    }
    controller = CuraEngineController(wslEnginePath)
    out = controller.generateGcode(
        stlPath=stlPath,
        outputPath=outputPath,
        settings=settings,
        definitionFiles=definitionFiles,
        autoDropToBuildPlate=True,
    )
    print(out)


if __name__ == "__main__":
    targetStlPath = "C:\\Users\\XuanouYe\\Desktop\\Thesis\\04-Implementation\\PC\\src\\cube.mold.stl"
    main(targetStlPath)
