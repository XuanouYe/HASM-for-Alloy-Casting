import json
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

rootDir = Path(__file__).resolve().parent.parent
if str(rootDir) not in sys.path:
    sys.path.append(str(rootDir))

from controlConfig import parameterSchema
from cnc.interface import generateCncJobInterface
from cnc.machineKinematics import XyzacTrtKinematics
from cnc.pathLinker import ClPathLinker


def loadClJson(inputJsonPath: str) -> Dict[str, Any]:
    with open(inputJsonPath, "r", encoding="utf-8") as fileHandle:
        return json.load(fileHandle)


def loadPostConfig(rawConfig: Dict[str, Any]) -> Dict[str, Any]:
    cfg: Dict[str, Any] = {}
    subtractiveSchema = parameterSchema.get("subtractive", {})
    for keyVal, valueVal in subtractiveSchema.items():
        if "default" in valueVal:
            cfg[keyVal] = dict(valueVal["default"]) if isinstance(valueVal["default"], dict) else valueVal["default"]
    subtractiveRaw = rawConfig.get("subtractive", {})
    for keyVal, valueVal in subtractiveRaw.items():
        if isinstance(valueVal, dict) and keyVal in cfg and isinstance(cfg[keyVal], dict):
            cfg[keyVal].update(valueVal)
        else:
            cfg[keyVal] = valueVal
    return cfg


def loadKinematics(postCfg: Dict[str, Any]) -> XyzacTrtKinematics:
    kinematicsCfg = postCfg.get("kinematics", {})
    machineType = str(kinematicsCfg.get("machineType", "xyzac-trt")).lower()
    if machineType == "xyzac-trt":
        return XyzacTrtKinematics(kinematicsCfg)
    return XyzacTrtKinematics(kinematicsCfg)


def loadLinkerConfig(processConfig: Dict[str, Any], clData: Dict[str, Any]) -> Dict[str, Any]:
    subtractiveSchema = parameterSchema.get("subtractive", {})
    subtractiveCfg = processConfig.get("subtractive", {})
    clLinkerCfg = clData.get("linkerConfig", {})
    defaultStepEnable = {
        "shellRemoval": True,
        "riserRemoval": True,
        "partFinishing": False,
        "gateRemoval": True
    }
    linkerCfg = {
        "safeHeight": float(subtractiveCfg.get("safeHeight", subtractiveSchema.get("safeHeight", {}).get("default", 5.0))),
        "maxRetractOffset": float(subtractiveCfg.get("maxRetractOffset", subtractiveSchema.get("maxRetractOffset", {}).get("default", 100.0))),
        "directLinkThreshold": float(subtractiveCfg.get("directLinkThreshold", subtractiveSchema.get("directLinkThreshold", {}).get("default", 2.0))),
        "rotationChangeThreshold": float(subtractiveCfg.get("rotationChangeThreshold", subtractiveSchema.get("rotationChangeThreshold", {}).get("default", 5.0))),
        "rotationRetractAngle": float(subtractiveCfg.get("rotationRetractAngle", subtractiveSchema.get("rotationRetractAngle", {}).get("default", 30.0))),
        "rotationSafeZ": float(subtractiveCfg.get("rotationSafeZ", subtractiveSchema.get("rotationSafeZ", {}).get("default", 30.0))),
        "linkFeedRate": float(subtractiveCfg.get("linkFeedRate", subtractiveSchema.get("linkFeedRate", {}).get("default", 2000.0))),
        "stepLinkingEnabled": dict(defaultStepEnable),
    }
    linkerCfg["stepLinkingEnabled"].update(clLinkerCfg.get("stepLinkingEnabled", {}))
    if isinstance(subtractiveCfg.get("stepLinkingEnabled"), dict):
        linkerCfg["stepLinkingEnabled"].update(subtractiveCfg["stepLinkingEnabled"])
    for keyVal in ("safeHeight", "maxRetractOffset", "directLinkThreshold", "rotationChangeThreshold", "rotationRetractAngle", "rotationSafeZ", "linkFeedRate"):
        if keyVal in clLinkerCfg and keyVal not in subtractiveCfg:
            linkerCfg[keyVal] = float(clLinkerCfg[keyVal])
    return linkerCfg


def formatValue(valueVal: float, decimals: int) -> str:
    return f"{valueVal:.{decimals}f}"


def buildProgramHeader(clData: Dict[str, Any], postCfg: Dict[str, Any]) -> List[str]:
    jobId = str(clData.get("jobId", "PROGRAM"))
    wcsId = str(clData.get("wcsId", "WCS0"))
    unitCode = "G21" if str(postCfg.get("unit", "mm")).lower() == "mm" else "G20"
    absCode = "G90" if postCfg.get("absoluteMode", True) else "G91"
    wcsCode = str(postCfg.get("wcsCode", "G54"))
    spindleSpeed = int(postCfg.get("spindleSpeed", 5000))
    spindleDir = "M3" if str(postCfg.get("spindleDirection", "CW")).upper() == "CW" else "M4"
    coolantEnabled = bool(postCfg.get("coolantEnabled", True))
    coolantCode = str(postCfg.get("coolantCode", "M8"))
    machineModel = str(postCfg.get("machineModel", "Default_5Axis_CNC"))
    headerBlocks = [
        "%",
        f"(PROGRAM: {jobId})",
        f"(MACHINE: {machineModel})",
        f"(WCS: {wcsId})",
        "(POST: gcodeProcessor.py)",
        f"{unitCode} {absCode} G17 G40 G49 G80",
        wcsCode,
        "M06 T01",
        f"{spindleDir} S{spindleSpeed}",
    ]
    if coolantEnabled:
        headerBlocks.append(coolantCode)
    return headerBlocks


def buildProgramFooter(postCfg: Dict[str, Any], coordDec: int) -> List[str]:
    coolantEnabled = bool(postCfg.get("coolantEnabled", True))
    footerBlocks: List[str] = []
    if coolantEnabled:
        footerBlocks.append("M9")
    footerBlocks.extend(["M5", f"G90 G0 Z{formatValue(0.0, coordDec)}", "M30", "%"])
    return footerBlocks


def generateGcode(clData: Dict[str, Any], postCfg: Dict[str, Any]) -> List[str]:
    outputFmt = postCfg.get("outputFormat", {})
    coordDec = int(outputFmt.get("coordDecimals", 3))
    angleDec = int(outputFmt.get("angleDecimals", 3))
    feedDec = int(outputFmt.get("feedDecimals", 1))
    useLineNums = bool(outputFmt.get("lineNumbers", False))
    lineNumInc = int(outputFmt.get("lineNumberIncrement", 10))
    mergeFeed = bool(postCfg.get("feedrateMergeStrategy", True))
    kinematicsCfg = postCfg.get("kinematics", {})
    aAxisName = str(kinematicsCfg.get("aAxisName", "A"))
    cAxisName = str(kinematicsCfg.get("cAxisName", "C"))
    defaultFeed = float(postCfg.get("feedRate", 500.0))
    axisEps = float(postCfg.get("axisOutputEps", 1e-4))

    kinematicsSolver = loadKinematics(postCfg)
    allLines: List[str] = []
    lineCounter = [lineNumInc]

    def emit(blockVal: str) -> None:
        if useLineNums:
            allLines.append(f"N{lineCounter[0]} {blockVal}")
            lineCounter[0] += lineNumInc
        else:
            allLines.append(blockVal)

    for headerLine in buildProgramHeader(clData, postCfg):
        emit(headerLine)

    prevFeed: Optional[float] = None
    prevAxes: Dict[str, Optional[float]] = {"X": None, "Y": None, "Z": None, aAxisName: None, cAxisName: None}
    stepsList = sorted(clData.get("steps", []), key=lambda stepItem: int(stepItem.get("stepId", 0)))

    for stepItem in stepsList:
        stepId = int(stepItem.get("stepId", 0))
        stepType = str(stepItem.get("stepType", "unknown"))
        emit(f"(STEP {stepId}: {stepType})")
        pointsList = sorted(stepItem.get("clPoints", []), key=lambda pointItem: int(pointItem.get("pointId", 0)))
        for pointItem in pointsList:
            pointPos = pointItem.get("position", [0.0, 0.0, 0.0])
            pointAxis = pointItem.get("toolAxis", [0.0, 0.0, 1.0])
            motionType = str(pointItem.get("motionType", "cut")).lower()
            feedrate = float(pointItem.get("feedrate", defaultFeed))
            px, py, pz, aDeg, cDeg = kinematicsSolver.convertPoint(pointPos, pointAxis)
            currAxes = {"X": px, "Y": py, "Z": pz, aAxisName: aDeg, cAxisName: cDeg}

            if motionType in {"rapid", "retract", "approach"}:
                blockParts = ["G0"]
            else:
                blockParts = ["G1"]

            if motionType == "retract":
                if prevAxes["Z"] is None or abs(currAxes["Z"] - prevAxes["Z"]) > axisEps:
                    blockParts.append(f"Z{formatValue(currAxes['Z'], coordDec)}")
                    prevAxes["Z"] = currAxes["Z"]
            else:
                for axisName, axisVal in currAxes.items():
                    if prevAxes[axisName] is None or abs(axisVal - float(prevAxes[axisName])) > axisEps:
                        decimals = angleDec if axisName in {aAxisName, cAxisName} else coordDec
                        blockParts.append(f"{axisName}{formatValue(axisVal, decimals)}")
                        prevAxes[axisName] = axisVal

            if blockParts == ["G0"]:
                continue

            if motionType == "cut":
                if (not mergeFeed) or prevFeed is None or abs(feedrate - prevFeed) > axisEps:
                    blockParts.append(f"F{formatValue(feedrate, feedDec)}")
                    prevFeed = feedrate
            emit(" ".join(blockParts))

    for footerLine in buildProgramFooter(postCfg, coordDec):
        emit(footerLine)
    return allLines


def writeGcodeFile(gcodeLines: List[str], outputPath: str) -> None:
    Path(outputPath).parent.mkdir(parents=True, exist_ok=True)
    with open(outputPath, "w", encoding="utf-8") as fileHandle:
        fileHandle.write("\n".join(gcodeLines))
        if gcodeLines:
            fileHandle.write("\n")


def generateGcodeFromClJson(inputJsonPath: str, processConfig: Dict[str, Any], outputGcodePath: str) -> None:
    clData = loadClJson(inputJsonPath)
    postCfg = loadPostConfig(processConfig)
    linkerCfg = loadLinkerConfig(processConfig, clData)
    linkedData = ClPathLinker(linkerCfg).processClData(clData)
    gcodeLines = generateGcode(linkedData, postCfg)
    writeGcodeFile(gcodeLines, outputGcodePath)


def generateCncGcodeInterface(partStl: str, moldStl: str, gateStl: str, riserStl: str, outputGcodePath: str, processConfig: Dict[str, Any], visualize: bool = False) -> Dict[str, Any]:
    clData = generateCncJobInterface(partStl, moldStl, gateStl, riserStl, outputGcodePath + ".json", processConfig, visualize=visualize)
    postCfg = loadPostConfig(processConfig)
    linkerCfg = loadLinkerConfig(processConfig, clData)
    linkedData = ClPathLinker(linkerCfg).processClData(clData)
    gcodeLines = generateGcode(linkedData, postCfg)
    writeGcodeFile(gcodeLines, outputGcodePath)
    return linkedData


if __name__ == '__main__':
    tempCncDir = Path(__file__).resolve().parent.parent / "tempCncFiles"
    if not tempCncDir.exists():
        tempCncDir.mkdir(parents=True, exist_ok=True)
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
            "axisCount": 18,
            "angleThreshold": 1.047,
        }
    }
    if Path(inputJson).exists():
        generateGcodeFromClJson(inputJson, testConfig, outputGcode)
