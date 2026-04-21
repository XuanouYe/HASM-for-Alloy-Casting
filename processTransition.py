from typing import List

import numpy as np

MILL_TO_NOZZLE_OFFSET = np.array([-2.1098, -196.7119, 2.0125], dtype=float)
CASTING_TO_MILL_OFFSET = np.array([60.5389, -89.9562], dtype=float)


def buildAdditiveEndSegment() -> List[str]:
    g92Pos = -MILL_TO_NOZZLE_OFFSET
    return [
        f"G0 X{0.0:.4f} Y{0.0:.4f}",
        f"G92 X{g92Pos[0]:.4f} Y{g92Pos[1]:.4f} Z{g92Pos[2]:.4f}",
    ]


def buildCastingApproachSegment(sprueInletPos: np.ndarray, safeZ: float) -> List[str]:
    spruePos = np.asarray(sprueInletPos, dtype=float)
    castingX = spruePos[0] - CASTING_TO_MILL_OFFSET[0]
    castingY = spruePos[1] - CASTING_TO_MILL_OFFSET[1]
    return [
        f"G0 Z{float(safeZ):.4f}",
        f"G0 X{castingX:.4f} Y{castingY:.4f}",
    ]


def appendTransitionToGcode(fdmGcodePath: str, sprueInletPos: np.ndarray, safeZ: float) -> None:
    additiveSegment = buildAdditiveEndSegment()
    castingSegment = buildCastingApproachSegment(sprueInletPos, safeZ)
    with open(fdmGcodePath, 'a') as gcodeFile:
        for line in additiveSegment:
            gcodeFile.write(f"{line}\n")
        for line in castingSegment:
            gcodeFile.write(f"{line}\n")
