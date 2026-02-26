import time
import posixpath
import subprocess
import struct
import os
from pathlib import Path, PureWindowsPath
from typing import Dict, Optional, List, Tuple, Any

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
        try:
            status = subprocess.run(
                ["wsl", "--status"],
                capture_output=True,
                text=True,
                encoding="utf-8",
                errors="ignore",
                timeout=5,
            )
            if status.returncode != 0:
                raise SliceException("WSL is not available or not properly configured")

            exists = subprocess.run(
                ["wsl", "test", "-f", self.wslEnginePath],
                capture_output=True,
                timeout=5,
            )
            if exists.returncode != 0:
                raise SliceException(f"CuraEngine not found in WSL at: {self.wslEnginePath}")
        except subprocess.TimeoutExpired:
            raise SliceException("WSL validation timeout")
        except FileNotFoundError:
            raise SliceException("wsl command not found. Please ensure WSL is installed.")
        except Exception as e:
            raise SliceException(f"WSL validation failed: {str(e)}")

    def _getStlBoundingBox(self, stlPath: str) -> Tuple[float, float, float, float, float, float]:
        try:
            with open(stlPath, 'rb') as f:
                header = f.read(80)
                if header.startswith(b'solid'):
                    raise SliceException("ASCII STL format not supported. Please use binary STL.")

                num_triangles = struct.unpack('<I', f.read(4))[0]

                min_x = min_y = min_z = float('inf')
                max_x = max_y = max_z = float('-inf')

                for _ in range(num_triangles):
                    f.read(12)
                    for _ in range(3):
                        x, y, z = struct.unpack('<fff', f.read(12))
                        min_x = min(min_x, x)
                        max_x = max(max_x, x)
                        min_y = min(min_y, y)
                        max_y = max(max_y, y)
                        min_z = min(min_z, z)
                        max_z = max(max_z, z)
                    f.read(2)

                return (min_x, max_x, min_y, max_y, min_z, max_z)
        except struct.error as e:
            raise SliceException(f"Invalid STL file format: {str(e)}")
        except FileNotFoundError:
            raise SliceException(f"STL file not found: {stlPath}")
        except Exception as e:
            raise SliceException(f"Failed to read STL bounding box: {str(e)}")

    def _windowsPathToWsl(self, windowsPath: str) -> str:
        try:
            normalized = str(Path(windowsPath).resolve())
            winPath = PureWindowsPath(normalized)
            drive = winPath.drive.replace(":", "").lower()
            pathParts = winPath.parts[1:]
            wslPath = posixpath.join("/mnt", drive, *pathParts)
            return wslPath
        except Exception:
            return windowsPath

    def _wslPathToWindows(self, wslPath: str) -> str:
        try:
            if not wslPath.startswith("/mnt/"):
                return wslPath
            parts = wslPath.split("/")
            drive = parts[2].upper() + ":"
            pathParts = parts[3:]
            winPath = str(PureWindowsPath(drive, *pathParts))
            return winPath
        except Exception:
            return wslPath

    def _validateInputFile(self, filePath: str, expectedExt: str) -> str:
        if not filePath.lower().endswith(expectedExt):
            raise SliceException(f"Invalid file format. Expected {expectedExt}")
        wslPath = self._windowsPathToWsl(filePath)
        try:
            result = subprocess.run(["wsl", "test", "-f", wslPath], capture_output=True, timeout=5)
            if result.returncode != 0:
                raise SliceException(f"Input file not found: {filePath}")
        except subprocess.TimeoutExpired:
            raise SliceException("File validation timeout")
        except SliceException:
            raise
        except Exception as e:
            raise SliceException(f"File validation failed: {str(e)}")
        return wslPath

    def _ensureOutputDirectory(self, outputPath: str) -> str:
        wslPath = self._windowsPathToWsl(outputPath)
        outputDir = posixpath.dirname(wslPath)
        if outputDir:
            try:
                subprocess.run(["wsl", "mkdir", "-p", outputDir], capture_output=True, text=True, timeout=5)
            except Exception:
                pass
        return wslPath

    def _buildCommandArgs(self, stlPath: str, outputPath: str, definitionFiles: Optional[List[str]] = None,
                          settings: Optional[Dict[str, str]] = None) -> list:
        cmd = ["wsl", self.wslEnginePath, "slice", "-v"]
        if definitionFiles:
            for f in definitionFiles:
                cmd.extend(["-j", self._windowsPathToWsl(f)])
        if settings:
            for k, v in settings.items():
                cmd.extend(["-s", f"{k}={v}"])
        cmd.extend(["-o", outputPath, "-l", stlPath])
        return cmd

    def _executeSlice(self, cmdArgs: list) -> None:
        startTime = time.time()
        try:
            result = subprocess.run(
                cmdArgs, capture_output=True, text=True, encoding="utf-8", errors="replace", timeout=600
            )
            self.lastExecutionTime = time.time() - startTime
            if result.returncode != 0:
                errorMsg = result.stderr if result.stderr else "Unknown error"
                raise SliceException(f"CuraEngine execution failed: {errorMsg}")
        except subprocess.TimeoutExpired:
            raise SliceException("Slicing timeout (exceeded 600 seconds)")
        except SliceException:
            raise
        except Exception as e:
            raise SliceException(f"Execution error: {str(e)}")

    def _validateOutputFile(self, wslPath: str) -> None:
        try:
            result = subprocess.run(["wsl", "test", "-s", wslPath], capture_output=True, timeout=5)
            if result.returncode != 0:
                raise SliceException(f"Output G-code not generated or is empty")
        except subprocess.TimeoutExpired:
            raise SliceException("Output file validation timeout")
        except Exception as e:
            raise SliceException(f"Output validation failed: {str(e)}")

    def generateGcode(
            self,
            stlPath: str,
            outputPath: str,
            settings: Optional[Dict[str, str]] = None,
            definitionFiles: Optional[List[str]] = None,
            autoDropToBuildPlate: bool = False,
            autoCenterXY: bool = False,
    ) -> str:
        wslStlPath = self._validateInputFile(stlPath, ".stl")
        wslOutputPath = self._ensureOutputDirectory(outputPath)
        
        if settings is None:
            settings = {}
            
        if autoDropToBuildPlate or autoCenterXY:
            min_x, max_x, min_y, max_y, min_z, max_z = self._getStlBoundingBox(stlPath)

            if autoDropToBuildPlate and "mesh_position_z" not in settings:
                settings["mesh_position_z"] = str(-min_z)

            if autoCenterXY:
                center_x = (min_x + max_x) / 2.0
                center_y = (min_y + max_y) / 2.0

                if "mesh_position_x" not in settings:
                    settings["mesh_position_x"] = str(-center_x)

                if "mesh_position_y" not in settings:
                    settings["mesh_position_y"] = str(-center_y)
        else:
            if "mesh_position_x" not in settings:
                settings["mesh_position_x"] = "0.0"
            if "mesh_position_y" not in settings:
                settings["mesh_position_y"] = "0.0"
            if "mesh_position_z" not in settings:
                settings["mesh_position_z"] = "0.0"

        finalWslOutputPath = wslOutputPath
        cmdArgs = self._buildCommandArgs(
            stlPath=wslStlPath,
            outputPath=finalWslOutputPath,
            definitionFiles=definitionFiles,
            settings=settings
        )
        self._executeSlice(cmdArgs)
        self._validateOutputFile(finalWslOutputPath)
        return self._wslPathToWindows(finalWslOutputPath)


def generateGcodeInterface(
    stlPath: str,
    outputPath: str,
    processConfig: Dict[str, Any],
) -> str:
    cm = ConfigManager()
    defaultConfig = cm.getDefaultConfig()
    additiveConfig = processConfig.get("additive") or defaultConfig.get("additive") or {}
    settings = cm.generateCuraConfig(additiveConfig)
    
    enginePath = processConfig.get("wslEnginePath", defaultWslEnginePath)
    if not enginePath:
        raise SliceException("wslEnginePath not configured in processConfig or defaults")
        
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
    )


def main():
    workspace = "workspace_test"
    manifestPath = os.path.join(workspace, "manifest.json")
    
    if os.path.exists(manifestPath):
        manifestMgr = ManifestManager.load(manifestPath)
        files = manifestMgr["files"]
        stlPath = files.get("moldShell")
        outPath = files.get("fdmGcode")
    else:
        print("Run moldGenerator.py first to create manifest")
        return
        
    testConfig = {
        "wsl_engine_path": "/mnt/c/users/xuanouye/desktop/thesis/04-implementation/pc/external/curaengine/build/release/CuraEngine",
        "definition_files": [
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
    
    try:
        controller = CuraEngineController(testConfig["wsl_engine_path"])
        out = controller.generateGcode(
            stlPath=stlPath,
            outputPath=outPath,
            settings=testConfig["settings"],
            definitionFiles=testConfig["definition_files"],
            autoDropToBuildPlate=False,
            autoCenterXY=False
        )
        print(f"Output generated at: {out}")
    except SliceException as e:
        print(f"Slicing failed: {e}")
    except Exception as e:
        print(f"Error: {e}")


if __name__ == "__main__":
    main()