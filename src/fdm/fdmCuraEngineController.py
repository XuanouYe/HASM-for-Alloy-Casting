"""
CuraEngine control layer - WSL wrapper for CuraEngine command-line interface
Used for STL model slicing to generate G-code.
"""

import sys
import logging
import subprocess
import posixpath
from datetime import datetime
from typing import Dict, Optional, List


class SliceException(Exception):
    """Slicing operation exception."""
    pass


class CuraEngineController:
    """Controller for calling CuraEngine via WSL (WSL mode only)."""

    def __init__(self, wslEnginePath: str, logLevel: str = "INFO"):
        self.wslEnginePath = wslEnginePath
        self.lastExecutionTime = 0.0
        self.logger = self._initializeLogger(logLevel)
        self._validateWslEngine()
        self.logger.info(f"CuraEngineController initialized with WSL engine: {self.wslEnginePath}")

    def _initializeLogger(self, logLevel: str) -> logging.Logger:
        logger = logging.getLogger("CuraEngineController")
        if logger.handlers:
            return logger

        logger.setLevel(getattr(logging, logLevel))

        consoleHandler = logging.StreamHandler(sys.stdout)
        consoleHandler.setLevel(getattr(logging, logLevel))

        formatter = logging.Formatter(
            '[%(asctime)s] [%(name)s] [%(levelname)s] %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )
        consoleHandler.setFormatter(formatter)
        logger.addHandler(consoleHandler)
        return logger

    def _validateWslEngine(self) -> None:
        """Validate WSL availability and existence of CuraEngine executable within WSL."""
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

            self.logger.info("WSL and CuraEngine validated successfully")
        except subprocess.TimeoutExpired:
            raise SliceException("WSL validation timeout")
        except FileNotFoundError:
            raise SliceException("wsl command not found. Please ensure WSL is installed.")
        except Exception as e:
            raise SliceException(f"WSL validation failed: {str(e)}")

    def generateGcode(
        self,
        stlPath: str,
        outputPath: str,
        settings: Optional[Dict[str, str]] = None,
        definitionFiles: Optional[List[str]] = None,
        previewOnly: bool = False,
        addTimestamp: bool = False,  # Timestamp disabled by default to match fdmTest
    ) -> str:
        """
        Execute STL slicing to generate G-code (parameters passed via -s).

        Parameters:
            stlPath: STL file path (WSL format, e.g., /mnt/c/...).
            outputPath: Output G-code path (WSL format).
            settings: Slicing settings dictionary, converted to -s key=value.
            definitionFiles: Cura definition file list (WSL paths).
            previewOnly: Preview command only, no execution.
            addTimestamp: Whether to add timestamp to output filename.
        """

        stlPath = self._validateInputFile(stlPath, ".stl")
        outputPath = self._ensureOutputDirectory(outputPath)

        finalOutputPath = outputPath
        if addTimestamp:
            # Use posixpath for WSL path handling
            outDir = posixpath.dirname(outputPath)
            outName = posixpath.basename(outputPath)
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            finalOutputPath = posixpath.join(outDir, f"{timestamp}_{outName}")

        self.logger.info("Starting slice process:")
        self.logger.info(f"  Input STL: {stlPath}")
        self.logger.info(f"  Output G-code: {finalOutputPath}")

        cmdArgs = self._buildCommandArgs(
            stlPath=stlPath,
            outputPath=finalOutputPath,
            definitionFiles=definitionFiles,
            settings=settings,
        )

        if previewOnly:
            self.logger.info("  Mode: PREVIEW ONLY (not executing)")
            self.logger.info(f"  Command: {' '.join(cmdArgs)}")
            return finalOutputPath

        self._executeSlice(cmdArgs)
        self._validateOutputFile(finalOutputPath)
        self.logger.info(f"Slice completed successfully in {self.lastExecutionTime:.2f}s")
        return finalOutputPath

    def _validateInputFile(self, filePath: str, expectedExt: str) -> str:
        """Validate input file exists and has correct format."""
        if not filePath.lower().endswith(expectedExt):
            raise SliceException(
                f"Invalid file format. Expected {expectedExt}, got {posixpath.splitext(filePath)[1]}"
            )

        try:
            result = subprocess.run(["wsl", "test", "-f", filePath], capture_output=True, timeout=5)
            if result.returncode != 0:
                raise SliceException(f"Input file not found in WSL: {filePath}")

            size = subprocess.run(
                ["wsl", "stat", "-c", "%s", filePath],
                capture_output=True,
                text=True,
                timeout=5,
            )
            if size.returncode == 0:
                fileSize = int(size.stdout.strip())
                self.logger.info(f"Input file verified: {filePath} ({fileSize / 1024 / 1024:.2f} MB)")
        except subprocess.TimeoutExpired:
            raise SliceException("File validation timeout")
        except SliceException:
            raise
        except Exception as e:
            raise SliceException(f"File validation failed: {str(e)}")

        return filePath

    def _ensureOutputDirectory(self, outputPath: str) -> str:
        """Ensure output directory exists (use posixpath for WSL paths)."""
        outputDir = posixpath.dirname(outputPath)

        if not outputDir:
            # If no directory part, use current directory
            self.logger.warning(f"No directory in output path, using current directory: {outputPath}")
            return outputPath

        try:
            # Use WSL mkdir -p to create directory
            result = subprocess.run(
                ["wsl", "mkdir", "-p", outputDir],
                capture_output=True,
                text=True,
                timeout=5
            )
            if result.returncode == 0:
                self.logger.debug(f"Output directory ensured: {outputDir}")
            else:
                self.logger.warning(f"Failed to create directory {outputDir}: {result.stderr}")
        except Exception as e:
            self.logger.warning(f"Failed to create output directory: {e}")

        return outputPath

    def _buildCommandArgs(
        self,
        stlPath: str,
        outputPath: str,
        definitionFiles: Optional[List[str]] = None,
        settings: Optional[Dict[str, str]] = None,
    ) -> list:
        """Build command-line arguments (consistent with fdmTest.py style)."""
        cmd = ["wsl", self.wslEnginePath, "slice", "-v"]

        # Cura definition files (-j)
        if definitionFiles:
            for f in definitionFiles:
                cmd.extend(["-j", f])

        # Settings (-s key=value)
        if settings:
            for k, v in settings.items():
                cmd.extend(["-s", f"{k}={v}"])

        # Output / input (consistent with fdmTest.py order)
        cmd.extend(["-o", outputPath, "-l", stlPath])

        self.logger.debug("Command args: " + " ".join(cmd))
        return cmd

    def _executeSlice(self, cmdArgs: list) -> None:
        """Execute slicing command."""
        import time

        startTime = time.time()
        try:
            self.logger.info("Executing CuraEngine via WSL...")

            # Consistent with fdmTest.py: use encoding/errors to avoid garbled characters
            result = subprocess.run(
                cmdArgs,
                capture_output=True,
                text=True,
                encoding="utf-8",
                errors="replace",
                timeout=600,
            )

            self.lastExecutionTime = time.time() - startTime

            if result.returncode != 0:
                errorMsg = result.stderr or result.stdout or "Unknown error"
                raise SliceException(
                    f"CuraEngine execution failed (code {result.returncode}): {errorMsg}"
                )

            if result.stdout:
                self.logger.debug(f"CuraEngine output: {result.stdout[:500]}")

        except subprocess.TimeoutExpired:
            raise SliceException("CuraEngine execution timeout (>600s)")
        except SliceException:
            raise
        except Exception as e:
            raise SliceException(f"CuraEngine execution error: {str(e)}")

    def _validateOutputFile(self, outputPath: str) -> None:
        """Validate successful output file generation."""
        try:
            exists = subprocess.run(["wsl", "test", "-f", outputPath], capture_output=True, timeout=5)
            if exists.returncode != 0:
                raise SliceException(f"Output G-code file not generated: {outputPath}")

            size = subprocess.run(
                ["wsl", "stat", "-c", "%s", outputPath],
                capture_output=True,
                text=True,
                timeout=5,
            )
            if size.returncode == 0:
                fileSize = int(size.stdout.strip())
                if fileSize == 0:
                    raise SliceException("Output G-code file is empty")
                self.logger.info(f"Output file verified: {outputPath} ({fileSize / 1024:.2f} KB)")

        except subprocess.TimeoutExpired:
            raise SliceException("Output file validation timeout")
        except SliceException:
            raise
        except Exception as e:
            raise SliceException(f"Output file validation failed: {str(e)}")

    def getLastExecutionTime(self) -> float:
        """Get last slicing execution time (seconds)."""
        return self.lastExecutionTime


def main():
    """Test program."""

    testConfig = {
        "wsl_engine_path": "/mnt/c/users/xuanouye/desktop/thesis/04-implementation/pc/external/curaengine/build/release/CuraEngine",
        "stl_path": "/mnt/c/users/xuanouye/desktop/thesis/04-implementation/pc/tests/model/monk.stl",
        "output_path": "/mnt/c/users/xuanouye/desktop/output.gcode",  # Fully consistent with fdmTest.py
        "definition_files": [
            "/mnt/c/users/xuanouye/desktop/thesis/04-implementation/pc/external/Cura/resources/definitions/fdmprinter.def.json",
            "/mnt/c/users/xuanouye/desktop/thesis/04-implementation/pc/external/Cura/resources/definitions/fdmextruder.def.json",
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
        print("\n1. Initializing CuraEngineController...")
        controller = CuraEngineController(testConfig["wsl_engine_path"], logLevel="DEBUG")
        print("✓ Controller initialized successfully")

        print("\n2. Preview mode test...")
        controller.generateGcode(
            stlPath=testConfig["stl_path"],
            outputPath=testConfig["output_path"],
            settings=testConfig["settings"],
            definitionFiles=testConfig["definition_files"],
            previewOnly=True,
            addTimestamp=False,  # No timestamp
        )

        print("\n3. Executing actual slicing...")
        out = controller.generateGcode(
            stlPath=testConfig["stl_path"],
            outputPath=testConfig["output_path"],
            settings=testConfig["settings"],
            definitionFiles=testConfig["definition_files"],
            previewOnly=False,
            addTimestamp=False,
        )
        print(f"\n✓ Slicing completed successfully!")
        print(f"  Output file: {out}")
        print(f"  Time taken: {controller.getLastExecutionTime():.2f} seconds")

        print("\n4. Testing output with timestamp...")
        outWithTimestamp = controller.generateGcode(
            stlPath=testConfig["stl_path"],
            outputPath=testConfig["output_path"],
            settings=testConfig["settings"],
            definitionFiles=testConfig["definition_files"],
            previewOnly=False,
            addTimestamp=True,
        )
        print(f"  Output with timestamp: {outWithTimestamp}")

    except SliceException as e:
        print(f"\n✗ Slicing failed: {e}")
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
