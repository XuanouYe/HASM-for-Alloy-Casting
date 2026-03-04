import json
import math
import re
import sys
import tempfile
import os

from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple
from cnc.pathDesigner import generateCncJobInterface


rootDir = Path(__file__).resolve().parent.parent
if str(rootDir) not in sys.path:
    sys.path.append(str(rootDir))

from controlConfig import parameterSchema


def loadClJson(inputJsonPath: str) -> Dict[str, Any]:
    with open(inputJsonPath, "r", encoding="utf-8") as f:
        return json.load(f)


def loadPostConfig(rawConfig: Dict[str, Any]) -> Dict[str, Any]:
    cfg: Dict[str, Any] = {}
    subtractiveSchema = parameterSchema.get("subtractive", {})
    for k, v in subtractiveSchema.items():
        if "default" in v:
            cfg[k] = dict(v["default"]) if isinstance(v["default"], dict) else v["default"]
    subtractiveRaw = rawConfig.get("subtractive", {})
    for k, v in subtractiveRaw.items():
        if isinstance(v, dict) and k in cfg and isinstance(cfg[k], dict):
            cfg[k].update(v)
        else:
            cfg[k] = v
    return cfg

def computeRotaryAngles(
        toolAxisVec: List[float],
        kinematics: Dict[str, Any],
) -> Tuple[float, float]:
    dx = float(toolAxisVec[0])
    dy = float(toolAxisVec[1])
    dz = float(toolAxisVec[2])
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length > 0.0:
        dx, dy, dz = dx / length, dy / length, dz / length

    rotOrder = str(kinematics.get("rotationOrder", "XZ")).upper()
    aSign = float(kinematics.get("aSign", 1.0))
    bSign = float(kinematics.get("bSign", 1.0))
    aLim = kinematics.get("aAxisLimit", [-120.0, 120.0])
    bLim = kinematics.get("bAxisLimit", [-360.0, 360.0])

    if rotOrder == "XZ":
        dzCl = max(-1.0, min(1.0, dz))
        aRad = math.acos(dzCl)
        sinA = math.sin(aRad)
        bRad = math.atan2(dx, -dy) if sinA >= 1e-9 else 0.0
    elif rotOrder == "YZ":
        dzCl = max(-1.0, min(1.0, dz))
        aRad = math.acos(dzCl)
        sinA = math.sin(aRad)
        bRad = math.atan2(dy, dx) if sinA >= 1e-9 else 0.0
    elif rotOrder == "XY":
        aRad = math.atan2(-dy, dz)
        bRad = math.asin(max(-1.0, min(1.0, dx)))
    else:
        dzCl = max(-1.0, min(1.0, dz))
        aRad = math.acos(dzCl)
        sinA = math.sin(aRad)
        bRad = math.atan2(dx, -dy) if sinA >= 1e-9 else 0.0

    aDeg = math.degrees(aRad) * aSign
    bDeg = math.degrees(bRad) * bSign
    aDeg = max(float(aLim[0]), min(float(aLim[1]), aDeg))
    bDeg = max(float(bLim[0]), min(float(bLim[1]), bDeg))
    return aDeg, bDeg


def formatBlock(template: str, **kwargs) -> str:
    filled: Dict[str, str] = {}
    for k, v in kwargs.items():
        if v is None:
            filled[k] = ""
        elif isinstance(v, float):
            filled[k] = f"{v:.3f}"
        elif isinstance(v, int):
            filled[k] = str(v)
        else:
            filled[k] = str(v)
    result = template
    for k, v in filled.items():
        result = result.replace("{" + k + "}", v)
    result = re.sub(r"[ \t]+", " ", result).strip()
    return result


def _fv(val: float, decimals: int) -> str:
    return f"{val:.{decimals}f}"


def _buildProgramHeader(
        clData: Dict[str, Any],
        postCfg: Dict[str, Any],
) -> List[str]:
    jobId = str(clData.get("jobId", "PROGRAM"))
    wcsId = str(clData.get("wcsId", "WCS0"))
    unitCode = "G21" if str(postCfg.get("unit", "mm")).lower() == "mm" else "G20"
    absCode = "G90" if postCfg.get("absoluteMode", True) else "G91"
    wcsCode = str(postCfg.get("wcsCode", "G54"))
    spindleSpeed = int(postCfg.get("spindleSpeed", 5000))
    spindleDir = (
        "M3" if str(postCfg.get("spindleDirection", "CW")).upper() == "CW" else "M4"
    )
    coolantEnabled = bool(postCfg.get("coolantEnabled", True))
    coolantCode = str(postCfg.get("coolantCode", "M8"))
    machineModel = str(postCfg.get("machineModel", "Default_5Axis_CNC"))

    blocks = [
        "%",
        f"(PROGRAM: {jobId})",
        f"(MACHINE: {machineModel})",
        f"(WCS: {wcsId})",
        "(POST: gcodeProcessor.py)",
        formatBlock("{u} {a} G17 G40 G49 G80", u=unitCode, a=absCode),
        wcsCode,
        f"{spindleDir} S{spindleSpeed}",
    ]
    if coolantEnabled:
        blocks.append(coolantCode)
    return blocks


def _buildProgramFooter(postCfg: Dict[str, Any]) -> List[str]:
    coolantEnabled = bool(postCfg.get("coolantEnabled", True))
    blocks: List[str] = []
    if coolantEnabled:
        blocks.append("M9")
    blocks.extend(["M5", "G91 G28 Z0", "G90", "M30", "%"])
    return blocks


def generateGcode(
        clData: Dict[str, Any],
        postCfg: Dict[str, Any],
) -> List[str]:
    outFmt = postCfg.get("outputFormat", {})
    coordDec = int(outFmt.get("coordDecimals", 3))
    angleDec = int(outFmt.get("angleDecimals", 3))
    feedDec = int(outFmt.get("feedDecimals", 1))
    useLineNums = bool(outFmt.get("lineNumbers", False))
    lineNumInc = int(outFmt.get("lineNumberIncrement", 10))
    mergeF = bool(postCfg.get("feedrateMergeStrategy", True))
    kinematics = postCfg.get("kinematics", {})
    aName = str(kinematics.get("aAxisName", "A"))
    bName = str(kinematics.get("bAxisName", "B"))
    defaultFeed = float(postCfg.get("feedRate", 500.0))

    allLines: List[str] = []
    nCounter = [lineNumInc]

    def emit(block: str) -> None:
        if useLineNums:
            allLines.append(f"N{nCounter[0]} {block}")
            nCounter[0] += lineNumInc
        else:
            allLines.append(block)

    for hLine in _buildProgramHeader(clData, postCfg):
        emit(hLine)

    steps = sorted(
        clData.get("steps", []),
        key=lambda s: int(s.get("stepId", 0)),
    )

    for step in steps:
        stepId = int(step.get("stepId", 0))
        stepType = str(step.get("stepType", "unknown"))
        emit(f"(STEP {stepId}: {stepType})")

        segMap: Dict[int, Dict[str, Any]] = {
            int(sg["segmentId"]): sg for sg in step.get("segments", [])
        }

        ptsBySegment: Dict[int, List[Dict[str, Any]]] = {}
        for pt in sorted(
                step.get("clPoints", []),
                key=lambda p: int(p.get("pointId", 0)),
        ):
            sid = int(pt.get("segmentId", 0))
            ptsBySegment.setdefault(sid, []).append(pt)

        for sid in sorted(ptsBySegment.keys()):
            segAxis = segMap.get(sid, {}).get("toolAxis", [0.0, 0.0, 1.0])
            emit(f"(SEGMENT {sid})")

            prevF: Optional[float] = None
            prevA: Optional[float] = None
            prevB: Optional[float] = None

            for pt in ptsBySegment[sid]:
                pos = pt.get("position", [0.0, 0.0, 0.0])
                ptAxis: List[float] = pt.get("toolAxis") or segAxis
                feedrate = float(pt.get("feedrate", defaultFeed))

                x = float(pos[0])
                y = float(pos[1])
                z = float(pos[2])
                aDeg, bDeg = computeRotaryAngles(ptAxis, kinematics)

                parts = ["G1"]
                parts.append(f"X{_fv(x, coordDec)}")
                parts.append(f"Y{_fv(y, coordDec)}")
                parts.append(f"Z{_fv(z, coordDec)}")

                if prevA is None or abs(aDeg - prevA) > 1e-4:
                    parts.append(f"{aName}{_fv(aDeg, angleDec)}")
                    prevA = aDeg
                if prevB is None or abs(bDeg - prevB) > 1e-4:
                    parts.append(f"{bName}{_fv(bDeg, angleDec)}")
                    prevB = bDeg

                if not mergeF or prevF is None or abs(feedrate - prevF) > 1e-4:
                    parts.append(f"F{_fv(feedrate, feedDec)}")
                    prevF = feedrate

                emit(" ".join(parts))

    for fLine in _buildProgramFooter(postCfg):
        emit(fLine)

    return allLines


def writeGcodeFile(gcodeLines: List[str], outputPath: str) -> None:
    Path(outputPath).parent.mkdir(parents=True, exist_ok=True)
    with open(outputPath, "w", encoding="utf-8") as f:
        f.write("\n".join(gcodeLines))
        if gcodeLines:
            f.write("\n")


def generateGcodeFromClJson(
        inputJsonPath: str,
        processConfig: Dict[str, Any],
        outputGcodePath: str,
) -> None:
    clData = loadClJson(inputJsonPath)
    postCfg = loadPostConfig(processConfig)
    gcodeLines = generateGcode(clData, postCfg)
    writeGcodeFile(gcodeLines, outputGcodePath)


def generateCncGcodeInterface(partStl: str, moldStl: str, gateStl: str, riserStl: str, outputGcodePath: str, processConfig: Dict[str, Any], visualize: bool = False) -> Dict[str, Any]:
    tempJsonPath = os.path.join(tempfile.gettempdir(), "tempCncCl.json")
    clData = generateCncJobInterface(partStl, moldStl, gateStl, riserStl, tempJsonPath, processConfig, visualize=visualize)
    generateGcodeFromClJson(tempJsonPath, processConfig, outputGcodePath)
    return clData


if __name__ == '__main__':
    rootDir = Path(__file__).resolve().parent.parent
    tempCncDir = rootDir / "tempCncFiles"

    if not tempCncDir.exists():
        tempCncDir.mkdir(parents=True, exist_ok=True)
        print(f"Created directory {tempCncDir}, please run cnc/pathDesigner.py first to generate cncToolpath.json")

    inputJson = str(tempCncDir / "cncToolpath.json")
    outputGcode = str(tempCncDir / "cnc.gcode")

    testConfig = {
        "subtractive": {
            "toolDiameter": 6.0,
            "toolSafetyMargin": 0.5,
            "feedRate": 500.0,
            "stepOver": 1.5,
            "layerStepDown": 1.0,
            "safeHeight": 5.0,
            "waterlineStepDown": 0.5,
            "axisMode": "hemisphere",
            "axisCount": 9,
            "angleThreshold": 1.047
        }
    }

    if Path(inputJson).exists():
        print(f"Found {inputJson}, generating G-code...")
        generateGcodeFromClJson(
            inputJsonPath=inputJson,
            processConfig=testConfig,
            outputGcodePath=outputGcode
        )
        print(f"G-code successfully generated: {outputGcode}")
    else:
        print(f"Input file not found: {inputJson}. Please run cnc/pathDesigner.py first.")
