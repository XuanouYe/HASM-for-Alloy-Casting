import os
import json
import subprocess
import logging
from pathlib import Path
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass, asdict
from datetime import datetime


# ===================== 日志配置 =====================
def setupLogger(logLevel: int = logging.INFO) -> logging.Logger:
    """配置日志记录器"""
    logger = logging.getLogger('ModelSlicer')
    logger.setLevel(logLevel)

    # 控制台处理器pip insall conan==1.56.0
    consoleHandler = logging.StreamHandler()
    consoleHandler.setLevel(logLevel)

    # 日志格式
    formatter = logging.Formatter(
        '%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )
    consoleHandler.setFormatter(formatter)

    # 添加文件处理器
    logDir = Path('logs')
    logDir.mkdir(exist_ok=True)
    fileHandler = logging.FileHandler(
        logDir / f'slicer_{datetime.now().strftime("%Y%m%d_%H%M%S")}.log'
    )
    fileHandler.setLevel(logLevel)
    fileHandler.setFormatter(formatter)

    logger.addHandler(consoleHandler)
    logger.addHandler(fileHandler)

    return logger


logger = setupLogger(logging.INFO)


# ===================== 数据结构 =====================
@dataclass
class SlicerConfig:
    """切片机配置参数"""
    layerHeight: float = 0.2  # 层高 (mm)
    wallThickness: float = 1.2  # 壳厚 (mm)
    topBottomThickness: float = 1.0  # 顶底厚度 (mm)
    infillDensity: float = 20.0  # 填充密度 (%)
    infillPattern: str = "grid"  # 填充图案
    printSpeed: float = 50.0  # 打印速度 (mm/s)
    nozzleTemperature: float = 200.0  # 喷头温度 (°C)
    bedTemperature: float = 60.0  # 床温度 (°C)
    nozzleDiameter: float = 0.4  # 喷嘴直径 (mm)
    lineWidth: float = 0.4  # 线宽 (mm)
    retractionEnabled: bool = True  # 是否启用回抽
    supportType: str = "tree"  # 支撑类型

    def toDict(self) -> Dict:
        """转换为字典"""
        return asdict(self)

    def toJsonFile(self, filePath: str) -> None:
        """保存为JSON文件"""
        with open(filePath, 'w') as f:
            json.dump(self.toDict(), f, indent=4)
        logger.info(f"配置已保存至: {filePath}")


@dataclass
class SliceResult:
    """切片结果"""
    modelPath: str  # 原始模型路径
    gcodeOutputPath: str  # 生成的G-Code路径
    sliceTime: float  # 切片耗时 (秒)
    layerCount: int  # 层数
    estimatedPrintTime: float  # 预估打印时间 (秒)
    estimatedFilamentUsage: float  # 预估耗材 (克)
    status: str  # 状态 (success/error)
    message: str  # 消息

    def toDict(self) -> Dict:
        """转换为字典"""
        return asdict(self)

    def toJsonFile(self, filePath: str) -> None:
        """保存为JSON文件"""
        with open(filePath, 'w') as f:
            json.dump(self.toDict(), f, indent=4)
        logger.info(f"结果已保存至: {filePath}")


# ===================== 核心切片类 =====================
class ModelSlicer:
    """模型切片管理器"""

    def __init__(self, curaEnginePath="external.CuraEngine-main"):
        self.curaEnginePath = curaEnginePath or self._findCuraEngine()
        if not self.curaEnginePath:
            raise FileNotFoundError("CuraEngine not found")
        logger.info(f"CuraEngine Path: {self.curaEnginePath}")

    @staticmethod
    def _findCuraEngine() -> Optional[str]:
        possibleNames = ['CuraEngine', 'CuraEngine.exe']
        for name in possibleNames:
            result = subprocess.run(
                ['which' if os.name != 'nt' else 'where', name],
                capture_output=True,
                text=True
            )
            if result.returncode == 0:
                return result.stdout.strip().split('\n')[0]
        return None

    def validateModelFile(self, modelPath: str) -> bool:
        """
        验证模型文件

        Args:
            modelPath: 模型文件路径

        Returns:
            是否有效
        """
        modelFile = Path(modelPath)

        if not modelFile.exists():
            logger.error(f"模型文件不存在: {modelPath}")
            return False

        supportedFormats = ['.stl', '.obj', '.3mf', '.ply']
        if modelFile.suffix.lower() not in supportedFormats:
            logger.error(f"不支持的文件格式: {modelFile.suffix}")
            return False

        if modelFile.stat().st_size == 0:
            logger.error(f"模型文件为空: {modelPath}")
            return False

        logger.info(f"模型文件验证通过: {modelPath}")
        return True

    def buildCuraEngineCommand(
            self,
            modelPath: str,
            outputPath: str,
            config: SlicerConfig,
            printerProfile: Optional[str] = None
    ) -> List[str]:
        """
        构建CuraEngine命令行

        Args:
            modelPath: 输入模型文件路径
            outputPath: 输出G-Code路径
            config: 切片配置
            printerProfile: 打印机配置文件路径

        Returns:
            命令行参数列表
        """
        cmd = [self.curaEnginePath, 'slice']

        # 添加冗长日志
        cmd.append('-v')

        # 添加打印机配置
        if printerProfile and Path(printerProfile).exists():
            cmd.extend(['-j', printerProfile])

        # 添加模型文件
        cmd.extend(['-l', modelPath])

        # 添加切片参数
        settingMap = {
            'layer_height': config.layerHeight,
            'wall_thickness': config.wallThickness,
            'top_bottom_thickness': config.topBottomThickness,
            'infill_sparse_density': config.infillDensity,
            'infill_pattern': config.infillPattern,
            'speed_print': config.printSpeed,
            'material_print_temperature': config.nozzleTemperature,
            'material_bed_temperature': config.bedTemperature,
            'machine_nozzle_diameter': config.nozzleDiameter,
            'line_width': config.lineWidth,
            'retraction_enable': str(config.retractionEnabled).lower(),
            'support_type': config.supportType,
        }

        for key, value in settingMap.items():
            cmd.extend(['-s', f'{key}={value}'])

        # 添加输出路径
        cmd.extend(['-o', outputPath])

        logger.info(f"构建的命令: {' '.join(cmd)}")
        return cmd

    def slice(
            self,
            modelPath: str,
            outputPath: Optional[str] = None,
            config: Optional[SlicerConfig] = None,
            printerProfile: Optional[str] = None,
            verbose: bool = True
    ) -> SliceResult:
        """
        执行切片操作

        Args:
            modelPath: 输入模型文件路径
            outputPath: 输出G-Code路径，如果为None则自动生成
            config: 切片配置，如果为None则使用默认配置
            printerProfile: 打印机配置文件路径
            verbose: 是否输出详细信息

        Returns:
            切片结果对象
        """
        import time

        # 参数处理
        if not self.validateModelFile(modelPath):
            return SliceResult(
                modelPath=modelPath,
                gcodeOutputPath="",
                sliceTime=0.0,
                layerCount=0,
                estimatedPrintTime=0.0,
                estimatedFilamentUsage=0.0,
                status="error",
                message="模型文件验证失败"
            )

        if outputPath is None:
            modelFile = Path(modelPath)
            outputPath = str(modelFile.parent / f"{modelFile.stem}.gcode")

        if config is None:
            config = SlicerConfig()

        logger.info(f"开始切片: {modelPath}")
        logger.info(f"输出路径: {outputPath}")

        try:
            # 构建命令
            cmd = self.buildCuraEngineCommand(modelPath, outputPath, config, printerProfile)

            # 执行切片
            startTime = time.time()
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=600  # 10分钟超时
            )
            sliceTime = time.time() - startTime

            if result.returncode != 0:
                errorMsg = result.stderr or "未知错误"
                logger.error(f"CuraEngine切片失败: {errorMsg}")
                return SliceResult(
                    modelPath=modelPath,
                    gcodeOutputPath=outputPath,
                    sliceTime=sliceTime,
                    layerCount=0,
                    estimatedPrintTime=0.0,
                    estimatedFilamentUsage=0.0,
                    status="error",
                    message=f"切片失败: {errorMsg}"
                )

            # 检查输出文件
            if not Path(outputPath).exists():
                logger.error(f"输出文件生成失败: {outputPath}")
                return SliceResult(
                    modelPath=modelPath,
                    gcodeOutputPath=outputPath,
                    sliceTime=sliceTime,
                    layerCount=0,
                    estimatedPrintTime=0.0,
                    estimatedFilamentUsage=0.0,
                    status="error",
                    message="输出文件生成失败"
                )

            # 解析G-Code统计信息
            layerCount, printTime, filament = self._parseGCode(outputPath)

            logger.info(f"切片成功! 耗时: {sliceTime:.2f}秒")
            logger.info(f"生成层数: {layerCount}")
            logger.info(f"预估打印时间: {printTime:.1f}秒 ({self._formatTime(printTime)})")
            logger.info(f"预估耗材: {filament:.2f}克")

            return SliceResult(
                modelPath=modelPath,
                gcodeOutputPath=outputPath,
                sliceTime=sliceTime,
                layerCount=layerCount,
                estimatedPrintTime=printTime,
                estimatedFilamentUsage=filament,
                status="success",
                message="切片成功"
            )

        except subprocess.TimeoutExpired:
            logger.error("切片超时（>10分钟）")
            return SliceResult(
                modelPath=modelPath,
                gcodeOutputPath=outputPath,
                sliceTime=0.0,
                layerCount=0,
                estimatedPrintTime=0.0,
                estimatedFilamentUsage=0.0,
                status="error",
                message="切片超时"
            )
        except Exception as e:
            logger.error(f"切片异常: {str(e)}")
            return SliceResult(
                modelPath=modelPath,
                gcodeOutputPath=outputPath,
                sliceTime=0.0,
                layerCount=0,
                estimatedPrintTime=0.0,
                estimatedFilamentUsage=0.0,
                status="error",
                message=f"异常: {str(e)}"
            )

    @staticmethod
    def _parseGCode(gcodeFilePath: str) -> Tuple[int, float, float]:
        """
        解析G-Code文件统计信息

        Args:
            gcodeFilePath: G-Code文件路径

        Returns:
            (层数, 打印时间秒, 耗材克数)
        """
        layerCount = 0
        totalTime = 0.0
        filamentUsage = 0.0

        try:
            with open(gcodeFilePath, 'r', encoding='utf-8', errors='ignore') as f:
                lines = f.readlines()

            for line in lines:
                line = line.strip()

                # 计算层数 (Z轴层号)
                if line.startswith(';LAYER:'):
                    layerCount += 1

                # 提取打印时间注释
                if line.startswith(';TIME:'):
                    try:
                        totalTime = float(line.split(':')[1])
                    except (IndexError, ValueError):
                        pass

                # 提取耗材长度并转换为克数 (简单估算)
                if line.startswith(';FILAMENT:'):
                    try:
                        filamentLength = float(line.split(':')[1])
                        filamentUsage = filamentLength * 0.0001  # 简单转换
                    except (IndexError, ValueError):
                        pass

            # 如果未读取到层数，用启发式方法估算
            if layerCount == 0:
                layerCount = max(1, len([l for l in lines if 'G1 Z' in l]) // 10)

        except Exception as e:
            logger.warning(f"解析G-Code失败: {str(e)}")

        return layerCount, totalTime, filamentUsage

    @staticmethod
    def _formatTime(seconds: float) -> str:
        """格式化时间"""
        hours = int(seconds // 3600)
        minutes = int((seconds % 3600) // 60)
        secs = int(seconds % 60)

        if hours > 0:
            return f"{hours}小时{minutes}分{secs}秒"
        elif minutes > 0:
            return f"{minutes}分{secs}秒"
        else:
            return f"{secs}秒"


# ===================== NC代码转换器 =====================
class GCodeToNCConverter:
    """G-Code到NC代码转换器"""

    @staticmethod
    def convertGCodeToNC(
            gcodeFilePath: str,
            ncOutputPath: Optional[str] = None,
            machineConfig: Optional[Dict] = None
    ) -> Tuple[bool, str]:
        """
        将G-Code转换为NC代码

        Args:
            gcodeFilePath: 输入G-Code文件路径
            ncOutputPath: 输出NC代码路径，如果为None则自动生成
            machineConfig: 机床配置参数

        Returns:
            (转换成功, 输出路径或错误消息)
        """
        try:
            gcodeFile = Path(gcodeFilePath)
            if not gcodeFile.exists():
                logger.error(f"G-Code文件不存在: {gcodeFilePath}")
                return False, f"文件不存在: {gcodeFilePath}"

            if ncOutputPath is None:
                ncOutputPath = str(gcodeFile.parent / f"{gcodeFile.stem}.nc")

            logger.info(f"开始转换G-Code到NC: {gcodeFilePath}")

            # 读取G-Code
            with open(gcodeFilePath, 'r', encoding='utf-8', errors='ignore') as f:
                gcodeLines = f.readlines()

            # 转换
            ncLines = []
            ncLines.append("; NC代码文件 - 从G-Code转换\n")
            ncLines.append(f"; 原始文件: {gcodeFile.name}\n")
            ncLines.append(f"; 生成时间: {datetime.now().isoformat()}\n")
            ncLines.append(";\n")
            ncLines.append("N0 O0001\n")  # 程序号
            ncLines.append(";\n")

            lineNumber = 10

            for gccodeLine in gcodeLines:
                gccodeLine = gccodeLine.strip()

                # 跳过注释和空行
                if not gccodeLine or gccodeLine.startswith(';'):
                    continue

                # 简单的G-Code到NC映射
                ncLine = GCodeToNCConverter._translateGCodeToNC(gccodeLine, lineNumber)
                if ncLine:
                    ncLines.append(ncLine)
                    lineNumber += 10

            # 添加程序结束
            ncLines.append(f"N{lineNumber} M30\n")
            ncLines.append("%\n")

            # 写入NC文件
            with open(ncOutputPath, 'w', encoding='utf-8') as f:
                f.writelines(ncLines)

            logger.info(f"NC代码转换成功: {ncOutputPath}")
            return True, ncOutputPath

        except Exception as e:
            errorMsg = f"转换异常: {str(e)}"
            logger.error(errorMsg)
            return False, errorMsg

    @staticmethod
    def _translateGCodeToNC(gcodeLine: str, lineNumber: int) -> Optional[str]:
        """
        翻译单行G-Code到NC代码

        Args:
            gcodeLine: G-Code行
            lineNumber: 行号

        Returns:
            NC代码行，如果无法翻译则返回None
        """
        parts = gcodeLine.upper().split()
        if not parts:
            return None

        ncCommand = f"N{lineNumber} "

        for part in parts:
            if part.startswith('G'):
                # G指令
                gCode = part[1:]
                if gCode in ['0', '00']:  # 快速定位
                    ncCommand += 'G00 '
                elif gCode in ['1', '01']:  # 直线插补
                    ncCommand += 'G01 '
                elif gCode in ['2', '02']:  # 顺时针圆弧
                    ncCommand += 'G02 '
                elif gCode in ['3', '03']:  # 逆时针圆弧
                    ncCommand += 'G03 '
                elif gCode == '28':  # 回参考点
                    ncCommand += 'G28 '
                elif gCode == '90':  # 绝对坐标
                    ncCommand += 'G90 '
                elif gCode == '91':  # 相对坐标
                    ncCommand += 'G91 '

            elif part.startswith('M'):
                # M指令
                mCode = part[1:]
                if mCode in ['0', '00']:
                    ncCommand += 'M00 '
                elif mCode in ['1', '01']:
                    ncCommand += 'M01 '
                elif mCode in ['2', '02']:
                    ncCommand += 'M02 '
                elif mCode in ['3', '03']:
                    ncCommand += 'M03 '
                elif mCode in ['4', '04']:
                    ncCommand += 'M04 '
                elif mCode in ['5', '05']:
                    ncCommand += 'M05 '
                elif mCode == '30':
                    ncCommand += 'M30 '

            elif part.startswith('X'):
                # X轴坐标
                ncCommand += f"X{part[1:]} "
            elif part.startswith('Y'):
                # Y轴坐标
                ncCommand += f"Y{part[1:]} "
            elif part.startswith('Z'):
                # Z轴坐标
                ncCommand += f"Z{part[1:]} "
            elif part.startswith('F'):
                # 进给速度
                ncCommand += f"F{part[1:]} "
            elif part.startswith('S'):
                # 主轴速度
                ncCommand += f"S{part[1:]} "
            elif part.startswith('T'):
                # 刀具号
                ncCommand += f"T{part[1:]} "
            elif part.startswith('I'):
                # 圆心X偏移
                ncCommand += f"I{part[1:]} "
            elif part.startswith('J'):
                # 圆心Y偏移
                ncCommand += f"J{part[1:]} "

        return ncCommand.rstrip() + "\n" if ncCommand.rstrip() != f"N{lineNumber}" else None


# ===================== 主程序 =====================
def main():
    """主程序"""

    print("=" * 60)
    print("模具切片与NC代码生成系统")
    print("=" * 60)

    # 初始化切片器
    try:
        slicer = ModelSlicer()
    except FileNotFoundError as e:
        logger.error(str(e))
        print("\n【错误】无法找到CuraEngine")
        print("请确保：")
        print("  1. 已安装CuraEngine")
        print("  2. CuraEngine已添加到系统PATH")
        return

    # 示例配置
    customConfig = SlicerConfig(
        layerHeight=0.15,
        wallThickness=1.2,
        infillDensity=15.0,
        printSpeed=60.0,
        nozzleTemperature=210.0,
    )

    # 示例模型路径（需要自行替换）
    modelPath = "monk.stl"

    if not Path(modelPath).exists():
        print(f"\n【警告】示例模型不存在: {modelPath}")
        return

    # 执行切片
    print(f"\n[1] 切片处理中...")
    sliceResult = slicer.slice(
        modelPath=modelPath,
        config=customConfig,
        verbose=True
    )

    print(f"\n[2] 切片结果:")
    print(f"    状态: {sliceResult.status}")
    print(f"    G-Code路径: {sliceResult.gcodeOutputPath}")
    print(f"    耗时: {sliceResult.sliceTime:.2f}秒")
    print(f"    层数: {sliceResult.layerCount}")
    print(f"    预估打印时间: {sliceResult.estimatedPrintTime:.1f}秒")
    print(f"    预估耗材: {sliceResult.estimatedFilamentUsage:.2f}克")

    # 保存结果
    resultPath = Path(modelPath).parent / "slice_result.json"
    sliceResult.toJsonFile(str(resultPath))

    # 转换为NC代码
    if sliceResult.status == "success":
        print(f"\n[3] 转换为NC代码...")
        success, ncPath = GCodeToNCConverter.convertGCodeToNC(
            sliceResult.gcodeOutputPath
        )

        if success:
            print(f"    NC代码生成成功: {ncPath}")
        else:
            print(f"    NC代码生成失败: {ncPath}")

    print("\n" + "=" * 60)


if __name__ == "__main__":
    main()
