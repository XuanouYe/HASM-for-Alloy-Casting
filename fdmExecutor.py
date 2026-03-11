import time
import posixpath
import subprocess
import struct
import math
import logging
from pathlib import Path, PureWindowsPath
from typing import Dict, Optional, List, Tuple, Any
from controlConfig import ConfigManager

logger = logging.getLogger(__name__)


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
                    minX = min(minX, x); maxX = max(maxX, x)
                    minY = min(minY, y); maxY = max(maxY, y)
                    minZ = min(minZ, z); maxZ = max(maxZ, z)
                f.read(2)
        return (minX, maxX, minY, maxY, minZ, maxZ)

    def windowsPathToWsl(self, windowsPath: str) -> str:
        """Windows 路径 → WSL 路径，保留原始大小写（Linux 文件系统区分大小写）。"""
        normalized = str(Path(windowsPath).resolve())
        winPath = PureWindowsPath(normalized)
        drive = winPath.drive.replace(":", "").lower()
        pathParts = winPath.parts[1:]
        return posixpath.join("/mnt", drive, *pathParts)

    def wslPathToWindows(self, wslPath: str) -> str:
        """WSL 路径 → Windows 路径，使用 Path 拼接避免双重转义。"""
        if not wslPath.startswith("/mnt/"):
            return wslPath
        parts = wslPath.split("/")
        drive = parts[2].upper() + ":"
        pathParts = parts[3:]
        return str(Path(drive + "\\", *pathParts))

    def ensureOutputDirectory(self, outputPath: str) -> str:
        wslPath = self.windowsPathToWsl(outputPath)
        outputDir = posixpath.dirname(wslPath)
        if outputDir:
            subprocess.run(["wsl", "mkdir", "-p", outputDir], capture_output=True, text=True)
        return wslPath

    def applyMachineLimits(self, settings: Dict[str, str],
                           axisLimits: Dict[str, Tuple[float, float]]) -> None:
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

    def buildCommandArgs(self, stlPath: str, outputPath: str,
                         definitionFiles: Optional[List[str]] = None,
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
        """
        执行 CuraEngine 切片命令。
        FIX: 检查 returncode，失败时将 stderr 内容拼入异常消息抛出，
             不再静默忽略切片错误。
        """
        startTime = time.time()
        result = subprocess.run(
            cmdArgs,
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace"
        )
        self.lastExecutionTime = time.time() - startTime

        if result.returncode != 0:
            stderr_snippet = (result.stderr or "")[-2000:]
            stdout_snippet = (result.stdout or "")[-500:]
            cmd_str = " ".join(str(c) for c in cmdArgs)
            raise RuntimeError(
                f"CuraEngine 切片失败 (returncode={result.returncode})\n"
                f"命令: {cmd_str}\n"
                f"--- stderr (末尾) ---\n{stderr_snippet}\n"
                f"--- stdout (末尾) ---\n{stdout_snippet}"
            )

    def applyRetractionCompensation(self, filePath: str,
                                    bufferLength: float,
                                    retractDist: float,
                                    reloadSpeed: float,
                                    retractSpeedMms: float) -> None:
        """
        颗粒料回抽后处理（后处理专用，与 CuraEngine 内置回抽完全独立）。

        在每段连续挤出路径末尾插入 C 轴回抽指令，
        在下一段挤出开始前插入 C 轴重载指令。

        参数含义：
          bufferLength    — 路径末尾多少 mm 内触发回抽（mm，XY 平面距离）
          retractDist     — 回抽量（C 轴当量 mm，即螺杆等效后退量）
          reloadSpeed     — 重载进给速度（mm/min）
          retractSpeedMms — 回抽速度（mm/s，内部转换为 mm/min）

        注意：此函数在 replaceExtruderAxis（E→C 替换）之前调用，
              文件中此时仍为 E 轴；回抽指令直接写 C 轴（不经过 E→C 替换），
              因此插入顺序为：
                1. applyRetractionCompensation（操作 E 轴数据，插入 C 轴回抽）
                2. replaceExtruderAxis（将剩余 E 轴替换为 C 轴）
        """
        logger.info(
            f"[回抽后处理] 开始 | 文件={filePath} | "
            f"buffer={bufferLength}mm retract={retractDist}mm "
            f"reload={reloadSpeed}mm/min speed={retractSpeedMms}mm/s"
        )

        with open(filePath, 'r') as f:
            lines = f.readlines()

        retractFeedrate = retractSpeedMms * 60.0
        optimizedLines = []
        segmentBuffer = []
        currentX = 0.0; currentY = 0.0; currentE = 0.0
        needsReload = False

        def dist(x1, y1, x2, y2):
            return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

        def flushBuffer():
            nonlocal needsReload
            if not segmentBuffer:
                return
            totalDist = sum(s["dist"] for s in segmentBuffer)
            if totalDist <= bufferLength:
                for s in segmentBuffer[:-1]:
                    optimizedLines.append(s["raw"])
                last = segmentBuffer[-1]
                optimizedLines.append(
                    f"G1 X{last['x']:.3f} Y{last['y']:.3f} "
                    f"C{currentE - retractDist:.5f} F{retractFeedrate:.1f}\n")
            else:
                accumulated = 0.0; splitIndex = -1
                for i in range(len(segmentBuffer) - 1, -1, -1):
                    accumulated += segmentBuffer[i]["dist"]
                    if accumulated >= bufferLength:
                        splitIndex = i; break
                for i in range(splitIndex):
                    optimizedLines.append(segmentBuffer[i]["raw"])
                sp = segmentBuffer[splitIndex]
                overflow = accumulated - bufferLength
                ratio = overflow / sp["dist"]
                sx = sp["prevX"] + (sp["x"] - sp["prevX"]) * ratio
                sy = sp["prevY"] + (sp["y"] - sp["prevY"]) * ratio
                se = sp["prevE"] + (sp["e"] - sp["prevE"]) * ratio
                sf = sp["f"] if sp["f"] else "2400"
                optimizedLines.append(f"G1 X{sx:.3f} Y{sy:.3f} C{se:.5f} F{sf}\n")
                final = segmentBuffer[-1]
                optimizedLines.append(
                    f"G1 X{final['x']:.3f} Y{final['y']:.3f} "
                    f"C{currentE - retractDist:.5f} F{retractFeedrate:.1f}\n")
            segmentBuffer.clear()
            needsReload = True

        isExtruding = False
        for line in lines:
            parts = line.strip().split()
            if not parts:
                optimizedLines.append(line); continue
            cmd = parts[0]
            if cmd in ["G0", "G1"]:
                nextX = currentX; nextY = currentY; nextE = currentE
                nextF = None; hasXy = False; hasE = False
                for p in parts[1:]:
                    if p.startswith("X"):   nextX = float(p[1:]); hasXy = True
                    elif p.startswith("Y"): nextY = float(p[1:]); hasXy = True
                    elif p.startswith("E"): nextE = float(p[1:]); hasE = True
                    elif p.startswith("F"): nextF = p[1:]
                isExtrusion = hasXy and hasE and nextE > currentE
                if isExtrusion:
                    if needsReload:
                        optimizedLines.append(f"G1 C{currentE:.5f} F{reloadSpeed:.1f}\n")
                    needsReload = False
                    segmentBuffer.append({
                        "raw": line, "x": nextX, "y": nextY, "e": nextE, "f": nextF,
                        "prevX": currentX, "prevY": currentY, "prevE": currentE,
                        "dist": dist(currentX, currentY, nextX, nextY)
                    })
                    isExtruding = True
                else:
                    if isExtruding:
                        flushBuffer()
                    isExtruding = False
                    optimizedLines.append(line)
                if hasXy: currentX = nextX; currentY = nextY
                if hasE:  currentE = nextE
            elif cmd == "G92":
                if isExtruding: flushBuffer()
                isExtruding = False
                for p in parts[1:]:
                    if p.startswith("E"): currentE = float(p[1:])
                optimizedLines.append(line)
            else:
                optimizedLines.append(line)

        if segmentBuffer:
            flushBuffer()

        with open(filePath, 'w') as f:
            f.writelines(optimizedLines)

        logger.info(f"[回抽后处理] 完成 | 处理行数={len(lines)} → {len(optimizedLines)}")

    def replaceExtruderAxis(self, filePath: str, extrusionScaleFactor: float = 1.0) -> None:
        """
        将 G 代码中所有剩余 E 轴替换为 C 轴，并乘以 extrusionScaleFactor。
        此步骤在 applyRetractionCompensation 之后执行。
        """
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
                        scaledVal = float(token[1:]) * extrusionScaleFactor
                        newTokens.append(f"C{scaledVal:.5f}")
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

    def updateGcodeBoundingBox(self, filePath: str) -> None:
        minX = minY = minZ = float('inf')
        maxX = maxY = maxZ = float('-inf')
        with open(filePath, 'r') as f:
            lines = f.readlines()
        for line in lines:
            if line.startswith(('G0', 'G1', 'G2', 'G3')):
                parts = line.split(';')[0].split()
                for part in parts[1:]:
                    if part.startswith('X'):
                        val = float(part[1:]); minX = min(minX, val); maxX = max(maxX, val)
                    elif part.startswith('Y'):
                        val = float(part[1:]); minY = min(minY, val); maxY = max(maxY, val)
                    elif part.startswith('Z'):
                        val = float(part[1:]); minZ = min(minZ, val); maxZ = max(maxZ, val)
        for v in [minX, minY, minZ, maxX, maxY, maxZ]:
            if v in (float('inf'), float('-inf')):
                minX = minY = minZ = maxX = maxY = maxZ = 0.0; break
        for i in range(len(lines)):
            if   lines[i].startswith(';MINX:'): lines[i] = f';MINX:{minX:.3f}\n'
            elif lines[i].startswith(';MINY:'): lines[i] = f';MINY:{minY:.3f}\n'
            elif lines[i].startswith(';MINZ:'): lines[i] = f';MINZ:{minZ:.3f}\n'
            elif lines[i].startswith(';MAXX:'): lines[i] = f';MAXX:{maxX:.3f}\n'
            elif lines[i].startswith(';MAXY:'): lines[i] = f';MAXY:{maxY:.3f}\n'
            elif lines[i].startswith(';MAXZ:'): lines[i] = f';MAXZ:{maxZ:.3f}\n'
            elif lines[i].startswith(';TARGET_MACHINE.NAME'): break
        with open(filePath, 'w') as f:
            f.writelines(lines)

    def generateGcode(self, stlPath: str, outputPath: str,
                      settings: Optional[Dict[str, str]] = None,
                      definitionFiles: Optional[List[str]] = None,
                      autoDropToBuildPlate: bool = True,
                      autoCenterXY: bool = True,
                      axisLimits: Optional[Dict[str, Tuple[float, float]]] = None,
                      additiveConfig: Optional[Dict[str, Any]] = None,
                      retractionConfig: Optional[Dict[str, Any]] = None) -> str:
        """
        完整 G 代码生成流水线：
          1. 路径转换 & 目录创建
          2. 机器轴限位注入
          3. STL 包围盒计算 → 自动落床 / 居中
          4. CuraEngine 切片（内置回抽已在 generateCuraConfig 中禁用）
          5. 验证输出文件存在
          6. [可选] applyRetractionCompensation — 后处理回抽（由 retractionConfig 控制）
          7. replaceExtruderAxis — E→C 轴替换 + 物理换算
          8. updateGcodeBoundingBox — 更新文件头包围盒注释

        参数：
          retractionConfig — 由 ConfigManager.getRetractionConfig() 返回，
                             None 表示跳过回抽后处理。
        """
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
                    targetX = targetY = 0.0
                else:
                    width = float(settings.get("machine_width", "200.0"))
                    depth = float(settings.get("machine_depth", "200.0"))
                    targetX = width / 2.0
                    targetY = depth / 2.0
                if "mesh_position_x" not in settings:
                    settings["mesh_position_x"] = str(targetX - (minX + maxX) / 2.0)
                if "mesh_position_y" not in settings:
                    settings["mesh_position_y"] = str(targetY - (minY + maxY) / 2.0)

        cmdArgs = self.buildCommandArgs(
            stlPath=wslStlPath, outputPath=wslOutputPath,
            definitionFiles=definitionFiles, settings=settings)

        logger.info(f"[切片] 开始 | STL={stlPath}")
        self.executeSlice(cmdArgs)
        logger.info(f"[切片] 完成 | 耗时={self.lastExecutionTime:.1f}s")

        windowsOutputPath = self.wslPathToWindows(wslOutputPath)

        # 验证输出文件存在（CuraEngine 静默失败时的兜底检查）
        if not Path(windowsOutputPath).exists():
            raise RuntimeError(
                f"CuraEngine 未生成输出文件: {windowsOutputPath}\n"
                f"请检查：\n"
                f"  1. WSL 引擎路径是否正确（当前: {self.wslEnginePath}）\n"
                f"  2. definition 文件路径是否可被 WSL 访问\n"
                f"  3. 输入 STL 文件是否有效（路径: {stlPath}）\n"
                f"  4. 输出目录是否有写入权限"
            )

        # FIX: 后处理回抽由 retractionConfig 参数控制，与 CuraEngine 内置回抽完全隔离
        if retractionConfig and retractionConfig.get("enabled", False):
            self.applyRetractionCompensation(
                filePath=windowsOutputPath,
                bufferLength=retractionConfig["bufferLength"],
                retractDist=retractionConfig["distance"],
                reloadSpeed=retractionConfig["reloadSpeed"],
                retractSpeedMms=retractionConfig["speed"],
            )
        else:
            logger.info("[回抽后处理] 已跳过（retractionEnabled=False）")

        scaleFactor = float((additiveConfig or {}).get("extrusionScaleFactor", 1.0))
        logger.info(f"[E→C轴替换] scaleFactor={scaleFactor}")
        self.replaceExtruderAxis(windowsOutputPath, extrusionScaleFactor=scaleFactor)
        self.updateGcodeBoundingBox(windowsOutputPath)

        return windowsOutputPath


def generateGcodeInterface(stlPath: str, outputPath: str,
                           processConfig: Dict[str, Any],
                           axisLimits: Optional[Dict[str, Tuple[float, float]]] = None) -> Dict[str, str]:
    """
    FDM G 代码生成入口，供 GUI MainController 和 WorkflowManager 调用。

    processConfig 约定（两种调用方式均支持）：
      方式 A — GUI/MainController 调用：
        processConfig = {"additive": {...}, "casting": {...}, "fdm": {...}, ...}
        （完整 config dict，additive 段在外层键下）

      方式 B — WorkflowManager 调用（FIX: 修正原来直接传 stepConfig 的断裂）：
        WorkflowManager.execute() 应传完整 config 而非展开的 stepConfig，
        详见 controlWorkflow.py 修正说明。
    """
    cm = ConfigManager()
    defaultConfig = cm.getDefaultConfig()

    # FIX: 支持两种调用形态
    # 若 processConfig 含 "additive" 键 → 完整 config（方式 A，正常路径）
    # 若不含 → 可能是已展开的 additive 段（方式 B 的旧调用，做兼容包装）
    if "additive" in processConfig:
        additiveConfig = processConfig.get("additive") or defaultConfig.get("additive") or {}
        fdmConfig = processConfig.get("fdm") or defaultConfig.get("fdm") or {}
    else:
        # 兼容 WorkflowManager 旧调用：直接传入的是 additive 段内容
        logger.warning(
            "[generateGcodeInterface] processConfig 不含 'additive' 键，"
            "按旧调用兼容处理。建议 WorkflowManager 传完整 config dict。"
        )
        additiveConfig = processConfig or defaultConfig.get("additive") or {}
        fdmConfig = defaultConfig.get("fdm") or {}

    settings = cm.generateCuraConfig(additiveConfig)

    # FIX: 引擎路径从 Windows 格式转换为 WSL 路径，保留原始大小写
    _tmp = CuraEngineController("")
    rawEnginePath = fdmConfig.get(
        "wslEnginePath",
        "C:\\Users\\XuanouYe\\Desktop\\Thesis\\04-Implementation\\PC\\external\\CuraEngine\\build\\Release\\CuraEngine"
    )
    enginePath = rawEnginePath if rawEnginePath.startswith("/") else _tmp.windowsPathToWsl(rawEnginePath)
    logger.info(f"[generateGcodeInterface] CuraEngine WSL 路径: {enginePath}")

    defs = fdmConfig.get("definitionFiles") or [
        "C:\\Users\\XuanouYe\\Desktop\\Thesis\\04-Implementation\\HASM-for-Alloy-Casting\\external\\Cura\\resources\\definitions\\fdmprinter.def.json",
        "C:\\Users\\XuanouYe\\Desktop\\Thesis\\04-Implementation\\HASM-for-Alloy-Casting\\external\\Cura\\resources\\definitions\\fdmextruder.def.json"
    ]
    autoDrop   = fdmConfig.get("autoDropToBuildPlate", True)
    autoCenter = fdmConfig.get("autoCenterXY", True)

    if not axisLimits:
        rawLimits = additiveConfig.get("axisLimits", {
            "X": [-100.0, 100.0], "Y": [-100.0, 100.0], "Z": [0.0, 100.0]
        })
        axisLimits = {
            axis: tuple(v) if isinstance(v, list) else v
            for axis, v in rawLimits.items()
        }

    # FIX: 使用 ConfigManager.getRetractionConfig() 统一提取回抽参数
    retractionConfig = cm.getRetractionConfig(additiveConfig)

    controller = CuraEngineController(enginePath)
    resultPath = controller.generateGcode(
        stlPath=stlPath,
        outputPath=outputPath,
        settings=settings,
        definitionFiles=defs,
        autoDropToBuildPlate=autoDrop,
        autoCenterXY=autoCenter,
        axisLimits=axisLimits,
        additiveConfig=additiveConfig,
        retractionConfig=retractionConfig,
    )
    return {"gcodePath": resultPath, "status": "success"}
