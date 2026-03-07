from pathlib import Path
from typing import Any, Dict
from .geometryUtils import concatenateMeshes, generateHemisphereAxes
from .pathGenerator import FiveAxisCncPathGenerator
from .visualization import PathVisualizer

def generateCncJobInterface(partStl: str, moldStl: str, gateStl: str, riserStl: str, outputJsonPath: str, processConfig: Dict[str, Any], jobId: str = 'JOB_AUTO', visualize: bool = False) -> Dict[str, Any]:
    subtractiveConfig = processConfig.get('subtractive', {})
    toolParams = {
        'type': 'ball',
        'diameter': float(subtractiveConfig.get('toolDiameter', 6.0)),
        'safetyMargin': float(subtractiveConfig.get('toolSafetyMargin', 0.5))
    }
    feedrate = float(subtractiveConfig.get('feedRate', 500.0))
    stepOver = float(subtractiveConfig.get('stepOver', 1.0))
    layerStep = float(subtractiveConfig.get('layerStepDown', 1.0))
    safeHeight = float(subtractiveConfig.get('safeHeight', 5.0))
    angleThreshold = float(subtractiveConfig.get('angleThreshold', 1.047))
    axisMode = str(subtractiveConfig.get('axisMode', 'hemisphere'))
    axisCount = int(subtractiveConfig.get('axisCount', 48))
    minAxisZ = float(subtractiveConfig.get('minAxisZ', 0.02))
    shellRoughStock = float(subtractiveConfig.get('shellRoughStock', 1.0))
    candidateAxes = generateHemisphereAxes(axisCount, minAxisZ) if axisMode == 'hemisphere' else subtractiveConfig.get('candidateAxes', [[0.0, 0.0, 1.0]])
    stepParams = [
        {
            'mode': 'zLevelRoughing',
            'stepOver': float(subtractiveConfig.get('shellStepOver', stepOver)),
            'layerStep': float(subtractiveConfig.get('shellLayerStepDown', layerStep)),
            'safeHeight': safeHeight,
            'feedrate': float(subtractiveConfig.get('shellFeedRate', feedrate)),
            'enablePathLinking': True
        },
        {
            'mode': str(subtractiveConfig.get('riserMode', 'dropRaster')),
            'stepOver': float(subtractiveConfig.get('riserStepOver', max(stepOver * 0.8, 0.8))),
            'safeHeight': safeHeight,
            'feedrate': float(subtractiveConfig.get('riserFeedRate', feedrate)),
            'angleThreshold': angleThreshold,
            'enablePathLinking': True
        },
        {
            'mode': 'surfaceFinishing',
            'stepOver': float(subtractiveConfig.get('finishStepOver', max(stepOver * 0.55, 0.6))),
            'projectionStep': float(subtractiveConfig.get('finishProjectionStep', 0.35)),
            'safeHeight': safeHeight,
            'feedrate': float(subtractiveConfig.get('finishFeedRate', feedrate * 0.85)),
            'scanAxis': 'x',
            'finishNormalAngleDeg': float(subtractiveConfig.get('finishNormalAngleDeg', 88.0)),
            'collisionSampleCount': int(subtractiveConfig.get('finishCollisionSampleCount', 18000)),
            'keepOutSampleCount': int(subtractiveConfig.get('finishKeepOutSampleCount', 6000)),
            'collisionClearance': float(subtractiveConfig.get('finishCollisionClearance', 0.18)),
            'contactPatchRadius': float(subtractiveConfig.get('finishContactPatchRadius', 1.2)),
            'lineGapTolerance': float(subtractiveConfig.get('finishLineGapTolerance', 1.0)),
            'finishStock': float(subtractiveConfig.get('finishStock', 0.03)),
            'enablePathLinking': False
        },
        {
            'mode': 'dropRaster',
            'stepOver': float(subtractiveConfig.get('gateStepOver', stepOver)),
            'safeHeight': safeHeight,
            'feedrate': float(subtractiveConfig.get('gateFeedRate', feedrate)),
            'angleThreshold': angleThreshold,
            'enablePathLinking': True
        }
    ]
    axisStrategyParams = {
        'candidateAxes': candidateAxes,
        'step3AxisCount': int(subtractiveConfig.get('step3AxisCount', 12)),
        'step3AxisSampleCount': int(subtractiveConfig.get('step3AxisSampleCount', 18000)),
        'step3TargetCoverage': float(subtractiveConfig.get('step3TargetCoverage', 0.95)),
        'axisCount': axisCount,
        'minAxisZ': minAxisZ,
        'finishNormalAngleDeg': float(subtractiveConfig.get('finishNormalAngleDeg', 88.0)),
        'shellRoughStock': shellRoughStock
    }
    generator = FiveAxisCncPathGenerator(version='2.3')
    clData = generator.generateJob(partStl, moldStl, gateStl, riserStl, toolParams, stepParams, axisStrategyParams, 'WCS_MAIN', jobId)
    generator.exportClJson(clData, outputJsonPath)
    if visualize:
        partMesh = generator.loadMesh(partStl)
        gateMesh = generator.loadMesh(gateStl)
        riserMesh = generator.loadMesh(riserStl)
        displayMesh = concatenateMeshes([partMesh, gateMesh, riserMesh])
        stepCollisionMeshes = [
            concatenateMeshes([partMesh, gateMesh, riserMesh]),
            concatenateMeshes([partMesh, gateMesh]),
            partMesh,
            partMesh
        ]
        visualizer = PathVisualizer()
        visualizer.visualize(displayMesh, clData, stepCollisionMeshes, {'partFinishing'})
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
                    'shellFeedRate': 500.0,
                    'riserFeedRate': 480.0,
                    'finishFeedRate': 420.0,
                    'gateFeedRate': 500.0,
                    'stepOver': 1.2,
                    'shellStepOver': 1.2,
                    'riserStepOver': 1.0,
                    'gateStepOver': 1.2,
                    'finishStepOver': 0.6,
                    'layerStepDown': 1.0,
                    'shellLayerStepDown': 1.0,
                    'safeHeight': 5.0,
                    'axisMode': 'hemisphere',
                    'axisCount': 48,
                    'minAxisZ': 0.02,
                    'finishProjectionStep': 0.35,
                    'finishNormalAngleDeg': 88.0,
                    'finishCollisionSampleCount': 18000,
                    'finishKeepOutSampleCount': 6000,
                    'finishCollisionClearance': 0.18,
                    'finishContactPatchRadius': 1.2,
                    'finishLineGapTolerance': 1.0,
                    'finishStock': 0.03,
                    'shellRoughStock': 1.0,
                    'step3AxisCount': 12,
                    'step3AxisSampleCount': 18000,
                    'step3TargetCoverage': 0.95,
                    'angleThreshold': 1.047
                }
            },
            jobId='JOB_TEST',
            visualize=True
        )
