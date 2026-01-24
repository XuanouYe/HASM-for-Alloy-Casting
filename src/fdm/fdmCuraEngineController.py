"""
CuraEngine控制层 - 封装CuraEngine命令行接口
用于STL模型切片生成G代码

模块功能：
- 自动检测CuraEngine可执行文件
- 构建并执行切片命令
- 监控执行过程，捕获错误
- 记录切片日志和性能指标
"""

import os
import sys
import json
import logging
import subprocess
import tempfile
from pathlib import Path
from datetime import datetime
from typing import Dict, Optional, Tuple
import shutil


class SliceException(Exception):
    """切片操作异常"""
    pass


class CuraEngineController:
    """
    CuraEngine控制器 - 管理CuraEngine的调用和参数构建

    属性:
        enginePath: CuraEngine可执行文件路径
        logger: 日志记录器
        lastExecutionTime: 最后一次执行耗时(秒)
        tempFileDir: 临时文件存储目录
    """

    def __init__(self, enginePath: str = "./CuraEngine", logLevel: str = "INFO"):
        """
        初始化CuraEngine控制器

        参数:
            enginePath: CuraEngine可执行文件路径，支持绝对/相对路径
            logLevel: 日志级别 (DEBUG/INFO/WARNING/ERROR)
        """
        self.enginePath = enginePath
        self.lastExecutionTime = 0.0
        self.tempFileDir = tempfile.gettempdir()

        # 初始化日志系统
        self.logger = self._initializeLogger(logLevel)

        # 验证CuraEngine是否存在
        self._validateEnginePath()

        self.logger.info(f"CuraEngineController initialized with engine: {self.enginePath}")

    def _initializeLogger(self, logLevel: str) -> logging.Logger:
        """初始化日志记录器"""
        logger = logging.getLogger("CuraEngineController")

        # 避免重复添加处理器
        if logger.handlers:
            return logger

        logger.setLevel(getattr(logging, logLevel))

        # 控制台处理器
        consoleHandler = logging.StreamHandler(sys.stdout)
        consoleHandler.setLevel(getattr(logging, logLevel))

        # 日志格式
        formatter = logging.Formatter(
            '[%(asctime)s] [%(name)s] [%(levelname)s] %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )
        consoleHandler.setFormatter(formatter)

        logger.addHandler(consoleHandler)

        return logger

    def _validateEnginePath(self) -> None:
        """
        验证CuraEngine可执行文件是否存在

        异常:
            SliceException: 引擎文件不存在
        """
        if not os.path.exists(self.enginePath):
            errorMsg = f"CuraEngine not found at: {self.enginePath}"
            self.logger.error(errorMsg)
            raise SliceException(errorMsg)

        if not os.access(self.enginePath, os.X_OK):
            self.logger.warning(f"CuraEngine may not be executable: {self.enginePath}")

    def generateGcode(
            self,
            stlPath: str,
            configDict: Dict,
            outputPath: str,
            previewOnly: bool = False,
            wslMode: bool = True,
            wslEnginePath: str = "/mnt/c/users/xuanouye/desktop/thesis/04-implementation/pc/external/curaengine/build/release/CuraEngine",
            definitionFiles: list = None,
            extraSettings: Dict = None
    ) -> str:
        """
        执行STL切片，生成G代码

        参数:
            stlPath: 输入STL文件路径
            configDict: 切片配置字典
            outputPath: 输出G代码文件路径
            previewOnly: 仅预览模式（不执行实际切片）

        返回:
            输出G代码文件的绝对路径

        异常:
            SliceException: STL文件不存在或切片失败
            ValueError: 配置参数非法
        """
        # 验证输入STL文件
        stlPath = self._validateInputFile(stlPath, ".stl")

        # 验证和处理输出路径
        outputPath = self._processOutputPath(outputPath)

        # 增强输出文件名（添加时间戳）
        outputDir = os.path.dirname(outputPath)
        outputBaseName = os.path.basename(outputPath)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        enhancedOutputPath = os.path.join(
            outputDir,
            f"{timestamp}_{outputBaseName}"
        )

        self.logger.info(f"Starting slice process:")
        self.logger.info(f"  Input STL: {stlPath}")
        self.logger.info(f"  Output G-code: {enhancedOutputPath}")

        if previewOnly:
            self.logger.info("  Mode: PREVIEW ONLY (not executing)")
            return enhancedOutputPath

        # 生成临时配置文件
        tempConfigPath = self._generateTempConfig(configDict)

        try:
            # 构建命令行参数
            cmdArgs = self._buildCommandArgs(
                stlPath,
                tempConfigPath,
                enhancedOutputPath,
                wslMode=wslMode,
                wslEnginePath=wslEnginePath,
                definitionFiles=definitionFiles,
                extraSettings=extraSettings
            )

            # 执行切片
            self._executeSlice(cmdArgs)

            # 验证输出文件
            self._validateOutputFile(enhancedOutputPath)

            self.logger.info(
                f"Slice completed successfully in {self.lastExecutionTime:.2f}s"
            )

            return enhancedOutputPath

        except SliceException as e:
            self.logger.error(f"Slice failed: {str(e)}")
            raise
        finally:
            # 清理临时文件
            if os.path.exists(tempConfigPath):
                os.remove(tempConfigPath)
                self.logger.debug(f"Cleaned up temp config: {tempConfigPath}")

    def _validateInputFile(self, filePath: str, expectedExt: str) -> str:
        """
        验证输入文件

        参数:
            filePath: 文件路径
            expectedExt: 期望的文件扩展名

        返回:
            绝对路径

        异常:
            SliceException: 文件不存在或格式错误
        """
        filePath = os.path.abspath(filePath)

        if not os.path.exists(filePath):
            raise SliceException(f"Input file not found: {filePath}")

        if not filePath.lower().endswith(expectedExt):
            raise SliceException(
                f"Invalid file format. Expected {expectedExt}, got {os.path.splitext(filePath)[1]}"
            )

        fileSize = os.path.getsize(filePath)
        self.logger.info(f"Input file verified: {filePath} ({fileSize / 1024 / 1024:.2f} MB)")

        return filePath

    def _processOutputPath(self, outputPath: str) -> str:
        """处理和验证输出路径"""
        outputPath = os.path.abspath(outputPath)
        outputDir = os.path.dirname(outputPath)

        # 创建输出目录（如果不存在）
        os.makedirs(outputDir, exist_ok=True)

        return outputPath

    def _generateTempConfig(self, configDict: Dict) -> str:
        """
        生成临时JSON配置文件

        参数:
            configDict: 配置字典

        返回:
            临时配置文件路径

        异常:
            SliceException: 配置生成失败
        """
        try:
            # 创建临时文件
            tempFile = tempfile.NamedTemporaryFile(
                mode='w',
                suffix='.json',
                delete=False,
                dir=self.tempFileDir
            )

            # 写入配置
            json.dump(configDict, tempFile, indent=2)
            tempFile.close()

            self.logger.debug(f"Generated temp config: {tempFile.name}")

            return tempFile.name

        except Exception as e:
            raise SliceException(f"Failed to generate config file: {str(e)}")

    def _buildCommandArgs(
            self,
            stlPath: str,
            configPath: str,
            outputPath: str,
            wslMode: bool = True,
            wslEnginePath: str = "/mnt/c/users/xuanouye/desktop/thesis/04-implementation/pc/external/curaengine/build/release/CuraEngine",
            definitionFiles: list = None,
            extraSettings: Dict = None
    ) -> list:
        """
        构建CuraEngine命令行参数

        参数:
            stlPath: STL输入文件
            configPath: 配置文件
            outputPath: G代码输出路径
            wslMode: 是否在WSL模式下运行
            wslEnginePath: WSL中CuraEngine的路径
            definitionFiles: 额外的定义文件列表
            extraSettings: 额外的设置参数

        返回:
            命令行参数列表
        """
        if wslMode:
            # WSL模式：通过wsl命令调用Linux中的CuraEngine
            cmdArgs = ["wsl", wslEnginePath]
        else:
            # 原生Windows模式
            cmdArgs = [self.enginePath]

        # 基础切片参数
        cmdArgs.extend([
            "slice",
            "-v",
            "-j", configPath,
            "-o", outputPath,
            "-l", stlPath
        ])

        # 添加定义文件（如fdmprinter.def.json等）
        if definitionFiles:
            for defFile in definitionFiles:
                cmdArgs.extend(["-j", defFile])

        # 添加额外的设置参数
        if extraSettings:
            for key, value in extraSettings.items():
                cmdArgs.extend(["-s", f"{key}={value}"])

        self.logger.debug(f"Command args: {' '.join(cmdArgs)}")

        return cmdArgs

    def _executeSlice(self, cmdArgs: list) -> None:
        """
        执行切片命令

        参数:
            cmdArgs: 命令行参数列表

        异常:
            SliceException: 切片执行失败
        """
        import time

        startTime = time.time()

        try:
            self.logger.info("Executing CuraEngine...")

            # 执行命令，捕获输出
            result = subprocess.run(
                cmdArgs,
                capture_output=True,
                text=True,
                timeout=600  # 10分钟超时
            )

            self.lastExecutionTime = time.time() - startTime

            # 检查执行状态
            if result.returncode != 0:
                errorMsg = result.stderr or result.stdout or "Unknown error"
                raise SliceException(
                    f"CuraEngine execution failed (code {result.returncode}): {errorMsg}"
                )

            # 记录输出日志
            if result.stdout:
                self.logger.debug(f"CuraEngine output: {result.stdout[:500]}")

        except subprocess.TimeoutExpired:
            raise SliceException("CuraEngine execution timeout (>600s)")
        except Exception as e:
            raise SliceException(f"CuraEngine execution error: {str(e)}")

    def _validateOutputFile(self, outputPath: str) -> None:
        """
        验证输出G代码文件

        参数:
            outputPath: 输出文件路径

        异常:
            SliceException: 输出文件验证失败
        """
        if not os.path.exists(outputPath):
            raise SliceException(f"Output G-code file not generated: {outputPath}")

        fileSize = os.path.getsize(outputPath)

        if fileSize == 0:
            raise SliceException("Output G-code file is empty")

        self.logger.info(f"Output file verified: {outputPath} ({fileSize / 1024:.2f} KB)")

    def getLastExecutionTime(self) -> float:
        """获取最后一次切片耗时"""
        return self.lastExecutionTime
