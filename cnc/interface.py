from pathlib import Path
from typing import Any, Dict
import trimesh
from .geometryUtils import generateHemisphereAxes
from .pathGenerator import FiveAxisCncPathGenerator
from .visualization import PathVisualizer


def generateCncJobInterface(partStl: str, moldStl: str, gateStl: str, riserStl: str, outputJsonPath: str,
                            processConfig: Dict[str, Any], jobId: str = 'JOB_AUTO', visualize: bool = False) -> Dict[str, Any]:
    subtractiveConfig = processConfig.get('subtractive', {})
    toolParams = {
        'type': 'ball',
        'diameter': float(subtractiveConfig.get('toolDiameter', 6.0)),
        'safetyMargin': float(subtractiveConfig.get('toolSafetyMargin', 0.5))
    }
    feedrate = float(subtractiveConfig.get('feedRate', 500.0))
    stepOver = float(subtractiveConfig.get('stepOver', 1.5))
    layerStep = float(subtractiveConfig.get('layerStepDown', 1.0))
    safeHeight = float(subtractiveConfig.get('safeHeight', 5.0))
    waterlineStep = float(subtractiveConfig.get('waterlineStepDown', 0.5))
    angleThreshold = float(subtractiveConfig.get('angleThreshold', 1.047))
    axisMode = str(subtractiveConfig.get('axisMode', 'hemisphere'))
    axisCount = int(subtractiveConfig.get('axisCount', 18))
    candidateAxes = generateHemisphereAxes(axisCount, float(subtractiveConfig.get('minAxisZ', 0.02))) if axisMode == 'hemisphere' else subtractiveConfig.get('candidateAxes', [[0.0, 0.0, 1.0]])
    stepParams = [
        {
            'mode': 'zLevelRoughing',
            'stepOver': stepOver,
            'layerStep': layerStep,
            'safeHeight': safeHeight,
            'feedrate': feedrate,
            'projectionStep': float(subtractiveConfig.get('projectionStep', 0.5))
        },
        {
            'mode': 'waterline',
            'sampling': stepOver,
            'layerStep': waterlineStep,
            'safeHeight': safeHeight,
            'feedrate': feedrate
        },
        {
            'mode': 'dropRaster',
            'stepOver': stepOver,
            'safeHeight': safeHeight,
            'feedrate': feedrate,
            'angleThreshold': angleThreshold
        }
    ]
    axisStrategyParams = {
        'candidateAxes': candidateAxes,
        'shellBaseAxisCount': int(subtractiveConfig.get('shellBaseAxisCount', max(axisCount * 3, 48))),
        'shellLocalAxisCount': int(subtractiveConfig.get('shellLocalAxisCount', 16)),
        'shellMaxAxes': int(subtractiveConfig.get('shellMaxAxes', 12)),
        'shellCoverageTarget': float(subtractiveConfig.get('shellCoverageTarget', 0.985)),
        'shellCoverageSamples': int(subtractiveConfig.get('shellCoverageSamples', 8000)),
        'shellAccessAngleDeg': float(subtractiveConfig.get('shellAccessAngleDeg', 72.0)),
        'shellMinAxisZ': float(subtractiveConfig.get('shellMinAxisZ', 0.02)),
        'shellMinAxisScore': float(subtractiveConfig.get('shellMinAxisScore', 20.0)),
        'shellCoverageStrict': bool(subtractiveConfig.get('shellCoverageStrict', True))
    }
    generator = FiveAxisCncPathGenerator(version='2.0')
    clData = generator.generateJob(partStl, moldStl, gateStl, riserStl, toolParams, stepParams, axisStrategyParams, 'WCS_MAIN', jobId)
    generator.exportClJson(clData, outputJsonPath)
    if visualize:
        partMesh = generator.loadMesh(partStl)
        gateMesh = generator.loadMesh(gateStl)
        riserMesh = generator.loadMesh(riserStl)
        stepCollisionMeshes = [trimesh.util.concatenate([partMesh, gateMesh, riserMesh]), gateMesh, partMesh]
        PathVisualizer().visualize(partMesh, clData, stepCollisionMeshes)
    return clData


if __name__ == '__main__':
    tempCncDir = Path('../tempCncFiles')
    if tempCncDir.exists():
        generateCncJobInterface(
            partStl=str(tempCncDir / 'part.stl'),
            moldStl=str(tempCncDir / 'mold.stl'),
            gateStl=str(tempCncDir / 'gate.stl'),
            riserStl=str(tempCncDir / 'riser.stl'),
            outputJsonPath=str(tempCncDir / 'cncToolpath.json'),
            processConfig={
                'subtractive': {
                    'toolDiameter': 6.0,
                    'toolSafetyMargin': 0.5,
                    'feedRate': 500.0,
                    'stepOver': 1.5,
                    'layerStepDown': 1.0,
                    'safeHeight': 5.0,
                    'waterlineStepDown': 0.5,
                    'axisMode': 'hemisphere',
                    'axisCount': 18,
                    'angleThreshold': 1.047,
                    'projectionStep': 0.5,
                    'shellBaseAxisCount': 48,
                    'shellLocalAxisCount': 16,
                    'shellMaxAxes': 12,
                    'shellCoverageTarget': 0.985,
                    'shellCoverageSamples': 8000,
                    'shellCoverageStrict': True
                }
            },
            jobId='JOB_TEST',
            visualize=True
        )
