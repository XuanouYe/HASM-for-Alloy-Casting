"""
改进的CuraEngine控制层 - 集中处理WSL路径转换和时间戳管理

职责:
- WSL路径转换（Windows ↔ WSL）- 内部处理，外部透明
- 时间戳管理 - 在此统一处理
- 文件验证与处理
- CuraEngine命令执行
"""

import sys
import time
import logging
import posixpath
import subprocess
from datetime import datetime
from pathlib import Path, PureWindowsPath
from typing import Dict, Optional, List


class SliceException(Exception):
    """切片操作异常"""
    pass


class CuraEngineController:
    """
    改进版CuraEngineController - 集中路径处理与时间戳管理

    特点:
    - 对外API接收/返回Windows路径，内部自动转换WSL格式
    - 时间戳处理统一在此，外部无需关注
    - 简化异常处理，异常自然向上传播
    """

    def __init__(self, wslEnginePath: str, logLevel: str = "INFO"):
        """
        初始化CuraEngine控制器

        参数:
            wslEnginePath: WSL环境中CuraEngine的路径 (e.g., "/mnt/c/CuraEngine/CuraEngine")
            logLevel: 日志级别
        """
        self.wslEnginePath = wslEnginePath
        self.lastExecutionTime = 0.0
        self.logger = self._initializeLogger(logLevel)
        self._validateWslEnvironment()
        self.logger.info(f"CuraEngineController initialized with WSL engine: {self.wslEnginePath}")

    def _initializeLogger(self, logLevel: str) -> logging.Logger:
        """初始化日志记录器"""
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

    def _validateWslEnvironment(self) -> None:
        """
        验证WSL环境和CuraEngine可执行文件

        异常:
            SliceException: 如果WSL不可用或CuraEngine不存在
        """
        try:
            status = subprocess.run(
                ["wsl", "--status"],
                capture_output=True,
                text=True,
                timeout=5
            )
            if status.returncode != 0:
                raise SliceException("WSL is not available or not properly configured")

            exists = subprocess.run(
                ["wsl", "test", "-f", self.wslEnginePath],
                capture_output=True,
                timeout=5
            )
            if exists.returncode != 0:
                raise SliceException(f"CuraEngine not found in WSL at: {self.wslEnginePath}")

            self.logger.info("WSL and CuraEngine validated successfully")
        except subprocess.TimeoutExpired:
            raise SliceException("WSL validation timeout")
        except FileNotFoundError:
            raise SliceException("wsl command not found. Please ensure WSL is installed.")
        except SliceException:
            raise
        except Exception as e:
            raise SliceException(f"WSL validation failed: {str(e)}")

    # ====== 路径转换方法（内部使用）======

    def _windowsPathToWsl(self, windowsPath: str) -> str:
        """
        将Windows路径转换为WSL格式

        参数:
            windowsPath: Windows路径 (e.g., "C:\\Users\\file.stl" or "C:/Users/file.stl")

        返回:
            WSL路径 (e.g., "/mnt/c/Users/file.stl")
        """
        try:
            # 标准化路径
            normalized = str(Path(windowsPath).resolve())
            winPath = PureWindowsPath(normalized)

            # 提取驱动器和路径部分
            drive = winPath.drive.replace(":", "").lower()  # "C:" -> "c"
            pathParts = winPath.parts[1:]  # 去除驱动器

            # 构建WSL路径
            wslPath = posixpath.join("/mnt", drive, *pathParts)
            self.logger.debug(f"Path (W→WSL): {windowsPath} -> {wslPath}")
            return wslPath
        except Exception as e:
            self.logger.warning(f"Windows to WSL conversion failed: {e}, using original path")
            return windowsPath

    def _wslPathToWindows(self, wslPath: str) -> str:
        """
        将WSL路径转换为Windows格式（用于日志和报告）

        参数:
            wslPath: WSL路径 (e.g., "/mnt/c/Users/file.gcode")

        返回:
            Windows路径 (e.g., "C:\\Users\\file.gcode")
        """
        try:
            if not wslPath.startswith("/mnt/"):
                return wslPath

            parts = wslPath.split("/")
            drive = parts[2].upper() + ":"  # "c" -> "C:"
            pathParts = parts[3:]

            winPath = str(PureWindowsPath(drive, *pathParts))
            self.logger.debug(f"Path (WSL→W): {wslPath} -> {winPath}")
            return winPath
        except Exception as e:
            self.logger.warning(f"WSL to Windows conversion failed: {e}, using original path")
            return wslPath

    # ====== 文件处理方法 ======

    def _validateInputFile(self, filePath: str, expectedExt: str) -> str:
        """
        验证输入文件存在且格式正确

        参数:
            filePath: 文件路径（Windows格式）
            expectedExt: 期望的文件扩展名

        返回:
            WSL格式的文件路径

        异常:
            SliceException: 文件不存在或格式错误
        """
        if not filePath.lower().endswith(expectedExt):
            raise SliceException(
                f"Invalid file format. Expected {expectedExt}, got {posixpath.splitext(filePath)[1]}"
            )

        # 转换为WSL路径用于验证
        wslPath = self._windowsPathToWsl(filePath)

        try:
            result = subprocess.run(
                ["wsl", "test", "-f", wslPath],
                capture_output=True,
                timeout=5
            )
            if result.returncode != 0:
                raise SliceException(f"Input file not found: {filePath}")

            # 获取文件大小
            size = subprocess.run(
                ["wsl", "stat", "-c", "%s", wslPath],
                capture_output=True,
                text=True,
                timeout=5
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

        return wslPath

    def _ensureOutputDirectory(self, outputPath: str) -> str:
        """
        确保输出目录存在

        参数:
            outputPath: 输出文件路径（Windows格式）

        返回:
            WSL格式的输出路径
        """
        wslPath = self._windowsPathToWsl(outputPath)
        outputDir = posixpath.dirname(wslPath)

        if not outputDir:
            self.logger.warning(f"No directory in output path, using current directory: {outputPath}")
            return wslPath

        try:
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

        return wslPath

    def _buildCommandArgs(
        self,
        stlPath: str,
        outputPath: str,
        definitionFiles: Optional[List[str]] = None,
        settings: Optional[Dict[str, str]] = None
    ) -> list:
        """
        构建CuraEngine命令行参数

        参数:
            stlPath: STL文件的WSL路径
            outputPath: 输出文件的WSL路径
            definitionFiles: Cura定义文件列表
            settings: 切片设置字典

        返回:
            命令行参数列表
        """
        cmd = ["wsl", self.wslEnginePath, "slice", "-v"]

        if definitionFiles:
            for f in definitionFiles:
                cmd.extend(["-j", self._windowsPathToWsl(f)])

        if settings:
            for k, v in settings.items():
                cmd.extend(["-s", f"{k}={v}"])

        cmd.extend(["-o", outputPath, "-l", stlPath])
        self.logger.debug("Command: " + " ".join(cmd))
        return cmd

    def _executeSlice(self, cmdArgs: list) -> None:
        """
        执行切片命令

        参数:
            cmdArgs: 命令行参数列表

        异常:
            SliceException: 切片执行失败
        """
        startTime = time.time()
        try:
            self.logger.info("Executing CuraEngine via WSL...")
            result = subprocess.run(
                cmdArgs,
                capture_output=True,
                text=True,
                timeout=600  # 10分钟超时
            )

            self.lastExecutionTime = time.time() - startTime

            if result.returncode != 0:
                errorMsg = result.stderr if result.stderr else "Unknown error"
                raise SliceException(f"CuraEngine execution failed: {errorMsg}")

            self.logger.info(result.stdout)
        except subprocess.TimeoutExpired:
            raise SliceException("Slicing timeout (exceeded 600 seconds)")
        except SliceException:
            raise
        except Exception as e:
            raise SliceException(f"Execution error: {str(e)}")

    def _validateOutputFile(self, wslPath: str) -> None:
        """
        验证输出G代码文件是否生成

        参数:
            wslPath: WSL格式的输出文件路径

        异常:
            SliceException: 输出文件不存在或为空
        """
        try:
            result = subprocess.run(
                ["wsl", "test", "-s", wslPath],
                capture_output=True,
                timeout=5
            )
            if result.returncode != 0:
                raise SliceException(f"Output G-code not generated or is empty")

            size = subprocess.run(
                ["wsl", "stat", "-c", "%s", wslPath],
                capture_output=True,
                text=True,
                timeout=5
            )
            if size.returncode == 0:
                fileSize = int(size.stdout.strip())
                self.logger.info(f"G-code generated successfully ({fileSize / 1024:.2f} KB)")
        except subprocess.TimeoutExpired:
            raise SliceException("Output file validation timeout")
        except SliceException:
            raise
        except Exception as e:
            raise SliceException(f"Output validation failed: {str(e)}")

    # ====== 公开API ======

    def generateGcode(
        self,
        stlPath: str,
        outputPath: str,
        settings: Optional[Dict[str, str]] = None,
        definitionFiles: Optional[List[str]] = None,
        previewOnly: bool = False,
        addTimestamp: bool = False
    ) -> str:
        """
        执行STL切片生成G代码

        这是对外的主要接口，自动处理所有Windows ↔ WSL路径转换

        参数:
            stlPath: STL文件路径（Windows格式）
            outputPath: G代码输出文件路径（Windows格式）
            settings: 切片设置字典 (snake_case格式)
            definitionFiles: Cura定义文件列表（Windows路径）
            previewOnly: 仅预览命令，不执行
            addTimestamp: 是否在输出文件名添加时间戳

        返回:
            G代码文件路径（Windows格式）

        异常:
            SliceException: 切片执行失败
        """
        # 验证并转换输入文件
        wslStlPath = self._validateInputFile(stlPath, ".stl")

        # 确保输出目录存在
        wslOutputPath = self._ensureOutputDirectory(outputPath)

        # 处理时间戳
        finalWslOutputPath = wslOutputPath
        if addTimestamp:
            outDir = posixpath.dirname(wslOutputPath)
            outName = posixpath.basename(wslOutputPath)
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            finalWslOutputPath = posixpath.join(outDir, f"{timestamp}_{outName}")

        self.logger.info("Starting slice process:")
        self.logger.info(f"  Input STL: {stlPath}")
        self.logger.info(f"  Output G-code: {self._wslPathToWindows(finalWslOutputPath)}")

        # 构建并执行命令
        cmdArgs = self._buildCommandArgs(
            stlPath=wslStlPath,
            outputPath=finalWslOutputPath,
            definitionFiles=definitionFiles,
            settings=settings
        )

        if previewOnly:
            self.logger.info("  Mode: PREVIEW ONLY (not executing)")
            self.logger.info(f"  Command: {' '.join(cmdArgs)}")
            return self._wslPathToWindows(finalWslOutputPath)

        # 执行切片
        self._executeSlice(cmdArgs)

        # 验证输出
        self._validateOutputFile(finalWslOutputPath)
        self.logger.info(f"Slice completed successfully in {self.lastExecutionTime:.2f}s")

        # 返回Windows格式的路径
        return self._wslPathToWindows(finalWslOutputPath)
