from pathlib import Path
from typing import Any, Dict
from cnc.geometryUtils import concatenateMeshes, generateHemisphereAxes, generateSphereAxes
from cnc.pathGenerator import FiveAxisCncPathGenerator
from cnc.visualization import PathVisualizer


def generateCncJobInterface(partStl: str, moldStl: str, gateStl: str, riserStl: str,
                             outputJsonPath: str, processConfig: Dict[str, Any],
                             jobId: str = "JOB_AUTO", visualize: bool = False) -> Dict[str, Any]:
    subCfg = processConfig.get("subtractive", {})
    toolDiameter = float(subCfg.get("toolDiameter", 6.0))
    toolLength = float(subCfg.get("toolLength", toolDiameter * 4.0))
    shankDiameter = float(subCfg.get("shankDiameter", toolDiameter))
    safetyMargin = float(subCfg.get("toolSafetyMargin", 0.5))
    sdfVoxelSize = float(subCfg.get("sdfVoxelSize", max(toolDiameter * 0.075, 0.2)))

    feedrate = float(subCfg.get("feedRate", 500.0))
    stepOver = float(subCfg.get("stepOver", 1.0))
    layerStep = float(subCfg.get("layerStepDown", 1.0))
    safeHeight = float(subCfg.get("safeHeight", 5.0))
    minToolpathZ = float(subCfg.get("minToolpathZ", 3.0))
    maxRetractOffset = float(subCfg.get("maxRetractOffset", 100.0))
    angleThreshold = float(subCfg.get("angleThreshold", 1.047))

    toolParams = {
        "diameter": toolDiameter,
        "toolLength": toolLength,
        "shankDiameter": shankDiameter,
        "safetyMargin": safetyMargin,
        "sdfVoxelSize": sdfVoxelSize,
        "minToolpathZ": minToolpathZ
    }

    axisMode = str(subCfg.get("axisMode", "hemisphere"))
    axisCount = int(subCfg.get("axisCount", 48))
    minAxisZ = float(subCfg.get("minAxisZ", 0.02))

    candidateAxes = generateHemisphereAxes(axisCount, minAxisZ) if axisMode == "hemisphere" else subCfg.get("candidateAxes", [[0.0, 0.0, 1.0]])
    step3CandidateAxes = generateSphereAxes(max(axisCount, 64)) if axisMode == "hemisphere" else candidateAxes

    enableStep1 = bool(subCfg.get("enableStep1ShellRemoval", True))
    enableStep2 = bool(subCfg.get("enableStep2RiserRemoval", True))
    enableStep3 = bool(subCfg.get("enableStep3PartFinishing", True))
    enableStep4 = bool(subCfg.get("enableStep4GateRemoval", True))

    shellStepOver = float(subCfg.get("shellStepOver", stepOver))
    shellLayerStep = float(subCfg.get("shellLayerStepDown", layerStep))
    shellFeedrate = float(subCfg.get("shellFeedRate", feedrate))
    shellRoughStock = float(subCfg.get("shellRoughStock", 0.0))

    step1TiltAngleDeg = float(subCfg.get("step1TiltAngleDeg", 35.0))
    step1TiltCount = int(subCfg.get("step1TiltCount", 2))
    step1UseContour = bool(subCfg.get("step1UseContour", True))
    step1ContourPasses = int(subCfg.get("step1ContourPasses", 2))
    step1SafeClearance = float(subCfg.get("step1SafeClearance", safetyMargin * 1.5))

    allStepDefs = [
        {
            "stepId": 1,
            "stepType": "shellRemoval",
            "enabled": enableStep1,
            "params": {
                "mode": "shellRemovalRoughing",
                "stepOver": shellStepOver,
                "layerStep": shellLayerStep,
                "safeHeight": safeHeight,
                "maxRetractOffset": maxRetractOffset,
                "feedrate": shellFeedrate,
                "roughStock": shellRoughStock,
                "sweepTol": sdfVoxelSize,
                "enablePathLinking": True,
                "step1TiltAngleDeg": step1TiltAngleDeg,
                "step1TiltCount": step1TiltCount,
                "step1UseContour": step1UseContour,
                "step1ContourPasses": step1ContourPasses,
                "step1SafeClearance": step1SafeClearance,
            }
        },
        {
            "stepId": 2,
            "stepType": "riserRemoval",
            "enabled": enableStep2,
            "params": {
                "mode": str(subCfg.get("riserMode", "dropRaster")),
                "stepOver": float(subCfg.get("riserStepOver", max(stepOver * 0.8, 0.8))),
                "safeHeight": safeHeight,
                "maxRetractOffset": maxRetractOffset,
                "feedrate": float(subCfg.get("riserFeedRate", feedrate)),
                "angleThreshold": angleThreshold,
                "sweepTol": sdfVoxelSize,
                "enablePathLinking": True
            }
        },
        {
            "stepId": 3,
            "stepType": "partFinishing",
            "enabled": enableStep3,
            "params": {
                "mode": "isoplanarpatchfinishing",
                "stepOver": float(subCfg.get("finishStepOver", 0.45)),
                "projectionStep": float(subCfg.get("finishProjectionStep", 0.25)),
                "safeHeight": safeHeight,
                "maxRetractOffset": maxRetractOffset,
                "feedrate": float(subCfg.get("finishFeedRate", feedrate * 0.85)),
                "finishStock": float(subCfg.get("finishStock", 0.03)),
                "sweepTol": sdfVoxelSize,
                "enablePathLinking": False
            }
        },
        {
            "stepId": 4,
            "stepType": "gateRemoval",
            "enabled": enableStep4,
            "params": {
                "mode": "dropRaster",
                "stepOver": float(subCfg.get("gateStepOver", stepOver)),
                "safeHeight": safeHeight,
                "maxRetractOffset": maxRetractOffset,
                "feedrate": float(subCfg.get("gateFeedRate", feedrate)),
                "angleThreshold": angleThreshold,
                "sweepTol": sdfVoxelSize,
                "enablePathLinking": True
            }
        }
    ]

    stepParams = [stepDef["params"] for stepDef in allStepDefs]

    axisStrategyParams = {
        "candidateAxes": candidateAxes,
        "step3CandidateAxes": step3CandidateAxes,
        "step3AxisCount": int(subCfg.get("step3AxisCount", min(16, axisCount))),
        "step3AxisSampleCount": int(subCfg.get("step3AxisSampleCount", 16000)),
        "step3MinNormalDot": 0.0,
        "step3TargetCoverage": float(subCfg.get("step3TargetCoverage", 0.995)),
        "step3AxisDiversityDot": float(subCfg.get("step3AxisDiversityDot", 0.985)),
        "step1TiltAngleDeg": step1TiltAngleDeg,
        "step1TiltCount": step1TiltCount,
    }

    generator = FiveAxisCncPathGenerator(version="4.0")
    clData = generator.generateJob(partStl, moldStl, gateStl, riserStl, toolParams, stepParams,
                                    axisStrategyParams, "WCS_MAIN", jobId)

    rawSteps = clData.get("steps", [])
    for stepIndex, stepData in enumerate(rawSteps):
        if stepIndex < len(allStepDefs):
            stepData["stepId"] = allStepDefs[stepIndex]["stepId"]
            stepData["stepType"] = allStepDefs[stepIndex]["stepType"]

    enabledStepMap = {stepDef["stepId"]: stepDef["enabled"] for stepDef in allStepDefs}
    filteredSteps = []
    for stepIndex, stepData in enumerate(rawSteps):
        stepId = int(stepData.get("stepId", stepIndex + 1))
        if enabledStepMap.get(stepId, False):
            filteredSteps.append(stepData)
    clData["steps"] = filteredSteps

    linkerConfig = {
        "safeHeight": safeHeight,
        "maxRetractOffset": maxRetractOffset,
        "directLinkThreshold": float(subCfg.get("directLinkThreshold", 2.0)),
        "rotationChangeThreshold": float(subCfg.get("rotationChangeThreshold", 5.0)),
        "rotationRetractAngle": float(subCfg.get("rotationRetractAngle", 30.0)),
        "rotationSafeZ": float(subCfg.get("rotationSafeZ", 30.0)),
        "linkFeedRate": float(subCfg.get("linkFeedRate", 2000.0)),
        "stepLinkingEnabled": {
            "shellRemoval": enableStep1,
            "riserRemoval": enableStep2,
            "partFinishing": enableStep3,
            "gateRemoval": enableStep4
        }
    }
    clData["linkerConfig"] = linkerConfig
    generator.exportClJson(clData, outputJsonPath)

    if visualize:
        partMesh = generator.loadMesh(partStl)
        gateMesh = generator.loadMesh(gateStl)
        riserMesh = generator.loadMesh(riserStl)
        displayMesh = concatenateMeshes([partMesh, gateMesh, riserMesh])
        activeStepTypes = {stepData.get("stepType", "") for stepData in clData.get("steps", [])}
        PathVisualizer().visualize(displayMesh, clData, None, activeStepTypes)

    return clData


if __name__ == "__main__":
    tempCncDir = Path("../tempCncFiles")
    if tempCncDir.exists():
        generateCncJobInterface(
            partStl=str(tempCncDir / "part.stl"),
            moldStl=str(tempCncDir / "mold.stl"),
            gateStl=str(tempCncDir / "gate.stl"),
            riserStl=str(tempCncDir / "riser.stl"),
            outputJsonPath=str(tempCncDir / "cncToolpath.json"),
            processConfig={
                "subtractive": {
                    "toolDiameter": 6.0,
                    "toolLength": 24.0,
                    "shankDiameter": 6.0,
                    "toolSafetyMargin": 0.5,
                    "sdfVoxelSize": 0.3,
                    "feedRate": 500.0,
                    "stepOver": 1.2,
                    "layerStepDown": 1.0,
                    "safeHeight": 5.0,
                    "minToolpathZ": 3.0,
                    "maxRetractOffset": 100.0,
                    "axisMode": "hemisphere",
                    "axisCount": 48,
                    "minAxisZ": 0.02,
                    "finishStepOver": 0.45,
                    "finishProjectionStep": 0.25,
                    "finishStock": 0.03,
                    "step3AxisCount": 16,
                    "step3AxisSampleCount": 16000,
                    "step3TargetCoverage": 0.995,
                    "step3AxisDiversityDot": 0.985,
                    "angleThreshold": 1.047,
                    "step1TiltAngleDeg": 35.0,
                    "step1TiltCount": 2,
                    "step1UseContour": True,
                    "step1ContourPasses": 2,
                    "step1SafeClearance": 0.75,
                }
            },
            jobId="JOB_TEST",
            visualize=True
        )