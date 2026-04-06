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

    toolParams = {
        "diameter": toolDiameter,
        "toolLength": toolLength,
        "shankDiameter": shankDiameter,
        "safetyMargin": safetyMargin,
        "sdfVoxelSize": sdfVoxelSize
    }

    feedrate = float(subCfg.get("feedRate", 500.0))
    stepOver = float(subCfg.get("stepOver", 1.0))
    layerStep = float(subCfg.get("layerStepDown", 1.0))
    safeHeight = float(subCfg.get("safeHeight", 5.0))
    maxRetractOffset = float(subCfg.get("maxRetractOffset", 100.0))
    angleThreshold = float(subCfg.get("angleThreshold", 1.047))

    axisMode = str(subCfg.get("axisMode", "hemisphere"))
    axisCount = int(subCfg.get("axisCount", 48))
    minAxisZ = float(subCfg.get("minAxisZ", 0.02))

    candidateAxes = generateHemisphereAxes(axisCount, minAxisZ) if axisMode == "hemisphere" else subCfg.get("candidateAxes", [[0.0, 0.0, 1.0]])
    step3CandidateAxes = generateSphereAxes(max(axisCount, 64)) if axisMode == "hemisphere" else candidateAxes

    stepParams = [
        {
            "mode": "zLevelRoughing",
            "stepOver": float(subCfg.get("shellStepOver", stepOver)),
            "layerStep": float(subCfg.get("shellLayerStepDown", layerStep)),
            "safeHeight": safeHeight,
            "maxRetractOffset": maxRetractOffset,
            "feedrate": float(subCfg.get("shellFeedRate", feedrate)),
            "roughStock": float(subCfg.get("shellRoughStock", 0.0)),
            "sweepTol": sdfVoxelSize,
            "enablePathLinking": True
        },
        {
            "mode": str(subCfg.get("riserMode", "dropRaster")),
            "stepOver": float(subCfg.get("riserStepOver", max(stepOver * 0.8, 0.8))),
            "safeHeight": safeHeight,
            "maxRetractOffset": maxRetractOffset,
            "feedrate": float(subCfg.get("riserFeedRate", feedrate)),
            "angleThreshold": angleThreshold,
            "sweepTol": sdfVoxelSize,
            "enablePathLinking": True
        },
        {
            "mode": "isoplanarpatchfinishing",
            "stepOver": float(subCfg.get("finishStepOver", 0.45)),
            "projectionStep": float(subCfg.get("finishProjectionStep", 0.25)),
            "safeHeight": safeHeight,
            "maxRetractOffset": maxRetractOffset,
            "feedrate": float(subCfg.get("finishFeedRate", feedrate * 0.85)),
            "finishStock": float(subCfg.get("finishStock", 0.03)),
            "sweepTol": sdfVoxelSize,
            "enablePathLinking": False
        },
        {
            "mode": "dropRaster",
            "stepOver": float(subCfg.get("gateStepOver", stepOver)),
            "safeHeight": safeHeight,
            "maxRetractOffset": maxRetractOffset,
            "feedrate": float(subCfg.get("gateFeedRate", feedrate)),
            "angleThreshold": angleThreshold,
            "sweepTol": sdfVoxelSize,
            "enablePathLinking": True
        }
    ]

    axisStrategyParams = {
        "candidateAxes": candidateAxes,
        "step3CandidateAxes": step3CandidateAxes,
        "step3AxisCount": int(subCfg.get("step3AxisCount", min(16, axisCount))),
        "step3AxisSampleCount": int(subCfg.get("step3AxisSampleCount", 16000)),
        "step3MinNormalDot": 0.0,
        "step3TargetCoverage": float(subCfg.get("step3TargetCoverage", 0.995)),
        "step3AxisDiversityDot": float(subCfg.get("step3AxisDiversityDot", 0.985))
    }

    generator = FiveAxisCncPathGenerator(version="4.0")
    clData = generator.generateJob(partStl, moldStl, gateStl, riserStl, toolParams, stepParams,
                                    axisStrategyParams, "WCS_MAIN", jobId)

    linkerConfig = {
        "safeHeight": safeHeight,
        "maxRetractOffset": maxRetractOffset,
        "directLinkThreshold": float(subCfg.get("directLinkThreshold", 2.0)),
        "rotationChangeThreshold": float(subCfg.get("rotationChangeThreshold", 5.0)),
        "rotationRetractAngle": float(subCfg.get("rotationRetractAngle", 30.0)),
        "rotationSafeZ": float(subCfg.get("rotationSafeZ", 30.0)),
        "linkFeedRate": float(subCfg.get("linkFeedRate", 2000.0)),
        "stepLinkingEnabled": {
            "shellRemoval": True,
            "riserRemoval": True,
            "partFinishing": False,
            "gateRemoval": True
        }
    }
    clData["linkerConfig"] = linkerConfig
    generator.exportClJson(clData, outputJsonPath)

    if visualize:
        partMesh = generator.loadMesh(partStl)
        gateMesh = generator.loadMesh(gateStl)
        riserMesh = generator.loadMesh(riserStl)
        displayMesh = concatenateMeshes([partMesh, gateMesh, riserMesh])
        PathVisualizer().visualize(displayMesh, clData, None, {"shellRemoval", "riserRemoval", "partFinishing", "gateRemoval"})

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
                    "angleThreshold": 1.047
                }
            },
            jobId="JOB_TEST",
            visualize=True
        )
