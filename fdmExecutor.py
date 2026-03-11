import time
import posixpath
import subprocess
import struct
import math
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
                    minX = min(minX, x); maxX = max(maxX, x)
                    minY = min(minY, y); maxY = max(maxY, y)
                    minZ = min(minZ, z); maxZ = max(maxZ, z)
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
        startTime = time.time()
        subprocess.run(cmdArgs, capture_output=True, text=True, encoding="utf-8", errors="replace")
        self.lastExecutionTime = time.time() - startTime

    def applyRetractionCompensation(self, filePath: str, bufferLength: float,
                                    retractDist: float, reloadSpeed: float,
                                    retractSpeedMms: float) -> None:
        """
        颗粒料回抽后处理。
        在每段连续挤出路径末尾插入回抽指令（C 轴负向），
        在下一段挤出开始前插入重载指令（C 轴正向恢复）。
        retractDist 单位：mm（C 轴当量）
        reloadSpeed 单位：mm/min
        retractSpeedMms 单位：mm/s（内部转换为 mm/min）
        """
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
                    f"G1 X{last['x']:.3f} Y{last['y']:.3f} C{currentE - retractDist:.5f} F{retractFeedrate:.1f}\n")
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
                    f"G1 X{final['x']:.3f} Y{final['y']:.3f} C{currentE - retractDist:.5f} F{retractFeedrate:.1f}\n")
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

    def replaceExtruderAxis(self, filePath: str, extrusionScaleFactor: float = 1.0) -> None:
        """
        将 G 代码中所有 E 轴替换为 C 轴，并乘以 extrusionScaleFactor。

        extrusionScaleFactor 是颗粒料挤出机的 E→C 物理换算系数：
          C_实际 = E_CuraEngine × extrusionScaleFactor
        CuraEngine 以 material_diameter 推导出的 E 值是"等效线材长度（mm）"，
        而颗粒机 C 轴的物理含义（螺杆转动量/泵送量）需要通过标定实验确定此系数。
        默认值 1.0 表示不做缩放（需要用户标定后填入实际值）。
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
                        val = float(part[1:])
                        minX = min(minX, val); maxX = max(maxX, val)
                    elif part.startswith('Y'):
                        val = float(part[1:])
                        minY = min(minY, val); maxY = max(maxY, val)
                    elif part.startswith('Z'):
                        val = float(part[1:])
                        minZ = min(minZ, val); maxZ = max(maxZ, val)
        for v in [minX, minY, minZ, maxX, maxY, maxZ]:
            if v in (float('inf'), float('-inf')):
                minX = minY = minZ = maxX = maxY = maxZ = 0.0
                break
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
                      additiveConfig: Optional[Dict[str, Any]] = None) -> str:
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
        self.executeSlice(cmdArgs)
        windowsOutputPath = self.wslPathToWindows(wslOutputPath)

        # FIX: 默认值由 True 改为 False，避免 additiveConfig 缺省时静默执行回抽
        if additiveConfig and additiveConfig.get("retractionEnabled", False):
            bufferLen   = float(additiveConfig.get("retractionBufferLength", 8.0))
            retractDist = float(additiveConfig.get("retractionDistance", 1.0))
            reloadSpeed = float(additiveConfig.get("retractionReloadSpeed", 120.0))
            retractSpeed= float(additiveConfig.get("retractionSpeed", 10.0))
            self.applyRetractionCompensation(
                windowsOutputPath, bufferLen, retractDist, reloadSpeed, retractSpeed)

        # FIX: 传入 extrusionScaleFactor，支持颗粒料 E→C 物理换算
        scaleFactor = float(additiveConfig.get("extrusionScaleFactor", 1.0)) if additiveConfig else 1.0
        self.replaceExtruderAxis(windowsOutputPath, extrusionScaleFactor=scaleFactor)
        self.updateGcodeBoundingBox(windowsOutputPath)
        return windowsOutputPath


def generateGcodeInterface(stlPath: str, outputPath: str,
                           processConfig: Dict[str, Any],
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
    autoDrop   = processConfig.get("autoDropToBuildPlate", True)
    autoCenter = processConfig.get("autoCenterXY", True)
    if not axisLimits:
        axisLimits = additiveConfig.get("axisLimits", {
            "X": (-100.0, 100.0), "Y": (-100.0, 100.0), "Z": (0.0, 100.0)
        })
        for axis, limits in axisLimits.items():
            if isinstance(limits, list) and len(limits) == 2:
                axisLimits[axis] = tuple(limits)
    controller = CuraEngineController(enginePath)
    resultPath = controller.generateGcode(
        stlPath=stlPath, outputPath=outputPath,
        settings=settings, definitionFiles=defs,
        autoDropToBuildPlate=autoDrop, autoCenterXY=autoCenter,
        axisLimits=axisLimits, additiveConfig=additiveConfig)
    return {"gcodePath": resultPath, "status": "success"}
