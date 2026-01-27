"""
CuraEngine Controller Module
This module provides an interface to control CuraEngine slicing software 
via Windows Subsystem for Linux (WSL).
"""

import time
import posixpath
import subprocess
import struct
from pathlib import Path, PureWindowsPath
from typing import Dict, Optional, List, Tuple


class SliceException(Exception):
    """
    Custom exception for slicing-related errors.
    """
    pass


class CuraEngineController:
    """
    Controller class for interfacing with CuraEngine running in WSL.

    This class handles:
    - WSL environment validation
    - File path conversion between Windows and WSL
    - STL file validation and analysis
    - CuraEngine command construction and execution
    - Slicing result validation
    """

    def __init__(self, wslEnginePath: str):
        """
        Initialize the CuraEngine controller.

        Args:
            wslEnginePath (str): Path to CuraEngine executable within WSL

        Raises:
            SliceException: If WSL environment or CuraEngine path is invalid
        """
        self.wslEnginePath = wslEnginePath
        self.lastExecutionTime = 0.0  # Time taken for last slicing operation
        self._validateWslEnvironment()

    def _validateWslEnvironment(self) -> None:
        """
        Validate that WSL is available and CuraEngine exists.

        Raises:
            SliceException: If WSL is not available or CuraEngine not found
        """
        try:
            # Check if WSL is running
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

            # Check if CuraEngine exists in WSL
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
        """
        Extract bounding box dimensions from binary STL file.

        Args:
            stlPath (str): Path to STL file

        Returns:
            Tuple[float, float, float, float, float, float]: 
                (min_x, max_x, min_y, max_y, min_z, max_z)

        Raises:
            SliceException: If STL file is invalid or in ASCII format
        """
        try:
            with open(stlPath, 'rb') as f:
                header = f.read(80)
                # Check for ASCII STL (not supported)
                if header.startswith(b'solid'):
                    raise SliceException("ASCII STL format not supported. Please use binary STL.")

                # Read number of triangles
                num_triangles = struct.unpack('<I', f.read(4))[0]

                # Initialize bounds
                min_x = min_y = min_z = float('inf')
                max_x = max_y = max_z = float('-inf')

                # Read all triangles and calculate bounds
                for _ in range(num_triangles):
                    # Skip normal vector
                    f.read(12)

                    # Read three vertices
                    for _ in range(3):
                        x, y, z = struct.unpack('<fff', f.read(12))
                        min_x = min(min_x, x)
                        max_x = max(max_x, x)
                        min_y = min(min_y, y)
                        max_y = max(max_y, y)
                        min_z = min(min_z, z)
                        max_z = max(max_z, z)

                    # Skip attribute byte count
                    f.read(2)

                return (min_x, max_x, min_y, max_y, min_z, max_z)

        except struct.error as e:
            raise SliceException(f"Invalid STL file format: {str(e)}")
        except FileNotFoundError:
            raise SliceException(f"STL file not found: {stlPath}")
        except Exception as e:
            raise SliceException(f"Failed to read STL bounding box: {str(e)}")

    def _windowsPathToWsl(self, windowsPath: str) -> str:
        """
        Convert Windows file path to WSL (Linux) path format.

        Args:
            windowsPath (str): Windows file path (e.g., "C:\Users\...")

        Returns:
            str: WSL path (e.g., "/mnt/c/users/...")
        """
        try:
            normalized = str(Path(windowsPath).resolve())
            winPath = PureWindowsPath(normalized)
            drive = winPath.drive.replace(":", "").lower()  # "C:" -> "c"
            pathParts = winPath.parts[1:]  # Remove drive letter
            wslPath = posixpath.join("/mnt", drive, *pathParts)
            return wslPath
        except Exception as e:
            # Fallback to original path if conversion fails
            return windowsPath

    def _wslPathToWindows(self, wslPath: str) -> str:
        """
        Convert WSL (Linux) file path to Windows path format.

        Args:
            wslPath (str): WSL file path (e.g., "/mnt/c/users/...")

        Returns:
            str: Windows path (e.g., "C:\Users\...")
        """
        try:
            if not wslPath.startswith("/mnt/"):
                return wslPath

            parts = wslPath.split("/")
            drive = parts[2].upper() + ":"  # "c" -> "C:"
            pathParts = parts[3:]

            winPath = str(PureWindowsPath(drive, *pathParts))
            return winPath
        except Exception as e:
            # Fallback to original path if conversion fails
            return wslPath

    def _validateInputFile(self, filePath: str, expectedExt: str) -> str:
        """
        Validate that input file exists and has correct extension.

        Args:
            filePath (str): Path to input file
            expectedExt (str): Expected file extension (e.g., ".stl")

        Returns:
            str: Validated WSL path to input file

        Raises:
            SliceException: If file validation fails
        """
        # Check file extension
        if not filePath.lower().endswith(expectedExt):
            raise SliceException(
                f"Invalid file format. Expected {expectedExt}, got {posixpath.splitext(filePath)[1]}"
            )

        # Convert to WSL path
        wslPath = self._windowsPathToWsl(filePath)

        # Check if file exists in WSL
        try:
            result = subprocess.run(
                ["wsl", "test", "-f", wslPath],
                capture_output=True,
                timeout=5
            )
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
        """
        Ensure output directory exists in WSL.

        Args:
            outputPath (str): Desired output file path

        Returns:
            str: WSL path to output file
        """
        wslPath = self._windowsPathToWsl(outputPath)
        outputDir = posixpath.dirname(wslPath)

        # Create directory if it doesn't exist
        if not outputDir:
            return wslPath

        try:
            subprocess.run(
                ["wsl", "mkdir", "-p", outputDir],
                capture_output=True,
                text=True,
                timeout=5
            )
        except Exception:
            # Directory creation may fail, but slicing might still work
            pass

        return wslPath

    def _buildCommandArgs(
            self,
            stlPath: str,
            outputPath: str,
            definitionFiles: Optional[List[str]] = None,
            settings: Optional[Dict[str, str]] = None
    ) -> list:
        """
        Build command arguments for CuraEngine.

        Args:
            stlPath (str): WSL path to STL file
            outputPath (str): WSL path for output G-code
            definitionFiles (List[str], optional): Printer definition files
            settings (Dict[str, str], optional): Slicing settings

        Returns:
            list: Complete command argument list for subprocess
        """
        cmd = ["wsl", self.wslEnginePath, "slice", "-v"]  # Verbose output

        # Add printer definition files
        if definitionFiles:
            for f in definitionFiles:
                cmd.extend(["-j", self._windowsPathToWsl(f)])

        # Add slicing settings
        if settings:
            for k, v in settings.items():
                cmd.extend(["-s", f"{k}={v}"])

        # Add output and input files
        cmd.extend(["-o", outputPath, "-l", stlPath])

        return cmd

    def _executeSlice(self, cmdArgs: list) -> None:
        """
        Execute CuraEngine slicing command.

        Args:
            cmdArgs (list): Command arguments for CuraEngine

        Raises:
            SliceException: If slicing fails or times out
        """
        startTime = time.time()

        try:
            # Execute CuraEngine
            result = subprocess.run(
                cmdArgs,
                capture_output=True,
                text=True,
                encoding="utf-8",
                errors="replace",
                timeout=600  # 10 minute timeout
            )

            self.lastExecutionTime = time.time() - startTime

            # Check for errors
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
        """
        Validate that output G-code file was generated and is not empty.

        Args:
            wslPath (str): WSL path to output file

        Raises:
            SliceException: If output file validation fails
        """
        try:
            result = subprocess.run(
                ["wsl", "test", "-s", wslPath],
                capture_output=True,
                timeout=5
            )
            if result.returncode != 0:
                raise SliceException(f"Output G-code not generated or is empty")
        except subprocess.TimeoutExpired:
            raise SliceException("Output file validation timeout")
        except SliceException:
            raise
        except Exception as e:
            raise SliceException(f"Output validation failed: {str(e)}")

    def generateGcode(
            self,
            stlPath: str,
            outputPath: str,
            settings: Optional[Dict[str, str]] = None,
            definitionFiles: Optional[List[str]] = None,
            autoDropToBuildPlate: bool = True,
    ) -> str:
        """
        Generate G-code from STL file using CuraEngine.

        Args:
            stlPath (str): Path to input STL file
            outputPath (str): Path for output G-code file
            settings (Dict[str, str], optional): Slicing parameters
            definitionFiles (List[str], optional): Printer definition files
            autoDropToBuildPlate (bool): Automatically position model on build plate

        Returns:
            str: Windows path to generated G-code file

        Raises:
            SliceException: If any step in the slicing process fails
        """
        # Validate input file
        wslStlPath = self._validateInputFile(stlPath, ".stl")

        # Ensure output directory exists
        wslOutputPath = self._ensureOutputDirectory(outputPath)

        # Auto-position model on build plate if enabled
        if autoDropToBuildPlate:
            min_x, max_x, min_y, max_y, min_z, max_z = self._getStlBoundingBox(stlPath)
            if settings is None:
                settings = {}
            if "mesh_position_z" not in settings:
                # Move model so lowest point is at Z=0
                settings["mesh_position_z"] = str(-min_z)

        finalWslOutputPath = wslOutputPath

        # Build and execute slicing command
        cmdArgs = self._buildCommandArgs(
            stlPath=wslStlPath,
            outputPath=finalWslOutputPath,
            definitionFiles=definitionFiles,
            settings=settings
        )

        self._executeSlice(cmdArgs)

        # Validate output
        self._validateOutputFile(finalWslOutputPath)

        # Return Windows path for convenience
        return self._wslPathToWindows(finalWslOutputPath)


def main():
    """
    Example usage of CuraEngineController.

    Demonstrates how to configure and use the controller for slicing.
    """
    # Test configuration
    testConfig = {
        "wsl_engine_path": "/mnt/c/users/xuanouye/desktop/thesis/04-implementation/pc/external/curaengine/build/release/CuraEngine",
        "stl_path": "C:\\Users\\XuanouYe\\Desktop\\Thesis\\04-Implementation\\PC\\src\\monk.stl",
        "output_path": "C:\\Users\\XuanouYe\\Desktop\\Thesis\\04-Implementation\\PC\\src\\output.gcode",
        "definition_files": [
            "C:\\Users\\XuanouYe\\Desktop\\Thesis\\04-Implementation\\PC\\external\\Cura\\resources\\definitions\\fdmprinter.def.json",
            "C:\\Users\\XuanouYe\\Desktop\\Thesis\\04-Implementation\\PC\\external\\Cura\\resources\\definitions\\fdmextruder.def.json",
        ],
        "settings": {
            "layer_height": "0.2",
            "wall_thickness": "0.8",
            "roofing_layer_count": "0",
            "flooring_layer_count": "0",
            "top_layers": "4",
            "bottom_layers": "4",
        },
    }

    try:
        # Initialize controller
        controller = CuraEngineController(testConfig["wsl_engine_path"])

        # Generate G-code
        out = controller.generateGcode(
            stlPath=testConfig["stl_path"],
            outputPath=testConfig["output_path"],
            settings=testConfig["settings"],
            definitionFiles=testConfig["definition_files"],
            autoDropToBuildPlate=True,
        )

        # Print results
        print(f"Slicing completed successfully!")
        print(f"Output: {out}")
        print(f"Execution time: {controller.lastExecutionTime:.2f}s")

    except SliceException as e:
        print(f"Slicing failed: {e}")
    except Exception as e:
        print(f"Error: {e}")


if __name__ == "__main__":
    main()
