if __name__ == '__main__':
    from cnc.interface import generateCncJobInterface as runEntry
    from pathlib import Path
    tempCncDir = Path('../tempCncFiles')
    if tempCncDir.exists():
        runEntry(
            partStl=str(tempCncDir / 'part.stl'),
            moldStl=str(tempCncDir / 'mold.stl'),
            gateStl=str(tempCncDir / 'gate.stl'),
            riserStl=str(tempCncDir / 'riser.stl'),
            outputJsonPath=str(tempCncDir / 'cncToolpath.json'),
            processConfig={'subtractive': {}},
            jobId='JOB_TEST',
            visualize=True
        )
