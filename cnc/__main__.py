from pathlib import Path
from cnc.interface import generateCncJobInterface


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
                    'shellBaseAxisCount': 36,
                    'shellLocalAxisCount': 10,
                    'shellMaxAxes': 8,
                    'shellCoverageTarget': 0.88,
                    'shellCoverageSamples': 5000,
                    'shellCoverageStrict': False
                }
            },
            jobId='JOB_TEST',
            visualize=True
        )
