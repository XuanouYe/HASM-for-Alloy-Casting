from typing import Tuple
import numpy as np
from scipy.spatial.transform import Rotation

class FlatEndMillTool:
    def __init__(self, radius: float, length: float, shankRadius: float):
        self.radius = float(radius)
        self.length = float(length)
        self.shankRadius = float(shankRadius)

    def sampleToolSurfaceLocal(self, diskSampleCount: int = 32, ringSampleCount: int = 8) -> np.ndarray:
        points = []
        for i in range(diskSampleCount):
            angle = 2.0 * np.pi * i / diskSampleCount
            for r in np.linspace(0.0, self.radius, 4):
                points.append([r * np.cos(angle), r * np.sin(angle), 0.0])
        ringZValues = np.linspace(0.0, self.length, ringSampleCount + 1)
        for zVal in ringZValues:
            for i in range(diskSampleCount):
                angle = 2.0 * np.pi * i / diskSampleCount
                points.append([self.radius * np.cos(angle), self.radius * np.sin(angle), zVal])
                if self.shankRadius > 0.0 and zVal > self.length * 0.5:
                    points.append([self.shankRadius * np.cos(angle), self.shankRadius * np.sin(angle), zVal])
        return np.asarray(points, dtype=float)

    def worldSurfaceSamples(self, tipCenter: np.ndarray, axisVec: np.ndarray, diskSampleCount: int = 32, ringSampleCount: int = 8) -> np.ndarray:
        localPts = self.sampleToolSurfaceLocal(diskSampleCount, ringSampleCount)
        zUnit = np.array([0.0, 0.0, 1.0], dtype=float)
        axis = np.asarray(axisVec, dtype=float)
        axisNorm = np.linalg.norm(axis)
        if axisNorm < 1e-9:
            axis = zUnit.copy()
        else:
            axis = axis / axisNorm
        rotAxis = np.cross(zUnit, axis)
        sinVal = np.linalg.norm(rotAxis)
        cosVal = float(np.dot(zUnit, axis))
        if sinVal < 1e-9:
            rotMat = np.eye(3, dtype=float) if cosVal > 0.0 else -np.eye(3, dtype=float)
        else:
            rotAxis = rotAxis / sinVal
            angle = np.arctan2(sinVal, cosVal)
            rotMat = Rotation.from_rotvec(angle * rotAxis).as_matrix()
        worldPts = (rotMat @ localPts.T).T + np.asarray(tipCenter, dtype=float)
        return worldPts
