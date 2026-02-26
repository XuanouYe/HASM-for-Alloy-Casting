import trimesh
import numpy as np
from dataModel import MachiningRegionResult


class AccessibilityAnalyzer:
    def __init__(self, mesh: trimesh.Trimesh):
        self.mesh = mesh
        self.mesh.fix_normals()
        self.zMin = self.mesh.bounds[0, 2]

    def _generateHemisphereDirections(self, normal: np.ndarray, numRays: int = 64) -> np.ndarray:
        goldenRatio = (1 + np.sqrt(5)) / 2
        indices = np.arange(numRays)
        theta = 2 * np.pi * indices / goldenRatio
        phi = np.arccos(1 - 2 * (indices + 0.5) / numRays)
        x = np.sin(phi) * np.cos(theta)
        y = np.sin(phi) * np.sin(theta)
        z = np.cos(phi)
        fibDirections = np.stack([x, y, z], axis=-1)
        numHorizontal = 90
        thetaH = np.linspace(0, 2 * np.pi, numHorizontal, endpoint=False)
        xH = np.cos(thetaH)
        yH = np.sin(thetaH)
        zH = np.zeros_like(thetaH)
        horizDirections = np.stack([xH, yH, zH], axis=-1)
        allDirections = np.vstack([fibDirections, horizDirections])
        normalNormalized = normal / np.linalg.norm(normal)
        dotProducts = allDirections @ normalNormalized
        validMask = dotProducts >= 1e-6
        return allDirections[validMask]

    def _checkPlaneIntersection(self, origins: np.ndarray, directions: np.ndarray) -> np.ndarray:
        return directions[:, 2] < -1e-6

    def analyze(self, numSamples: int = 10000, raysPerPoint: int = 180, batchSize: int = 100, normalOffset: float = 1e-6, exportCsv: bool = False, csvPath: str = None) -> MachiningRegionResult:
        points, faceIndices = trimesh.sample.sample_surface(self.mesh, numSamples)
        normals = self.mesh.face_normals[faceIndices]
        unmachinableMask = np.zeros(numSamples, dtype=bool)
        for i in range(numSamples):
            point = points[i]
            normal = normals[i]
            if abs(point[2] - self.zMin) < 1.0:
                unmachinableMask[i] = True
                continue
            rayDirections = self.generateHemisphereDirections(normal, raysPerPoint)
            numValidRays = len(rayDirections)
            if numValidRays == 0:
                unmachinableMask[i] = True
                continue
            rayOrigins = np.tile(point + normal * normalOffset, (numValidRays, 1))
            hitCount = 0
            for batchStart in range(0, numValidRays, batchSize):
                batchEnd = min(batchStart + batchSize, numValidRays)
                batchOrigins = rayOrigins[batchStart:batchEnd]
                batchDirections = rayDirections[batchStart:batchEnd]
                meshHits = self.mesh.ray.intersects_any(batchOrigins, batchDirections)
                planeHits = self.checkPlaneIntersection(batchOrigins, batchDirections)
                totalHits = np.logical_or(meshHits, planeHits)
                hitCount += np.sum(totalHits)
            if hitCount > numValidRays * 0.2:
                unmachinableMask[i] = True
        unmachinablePoints = points[unmachinableMask]
        unmachinablePercent = len(unmachinablePoints) / len(points) * 100
        if exportCsv:
            data = np.column_stack((points, unmachinableMask.astype(int)))
            outputTarget = csvPath if csvPath else "accessibility_scores.csv"
            np.savetxt(outputTarget, data, delimiter=",", header="x,y,z,unmachining", comments="", fmt="%.6f")
        return MachiningRegionResult(
            points=points,
            unmachinableMask=unmachinableMask,
            unmachinablePoints=unmachinablePoints,
            unmachinablePercent=float(unmachinablePercent),
            totalPoints=len(points),
            unmachinablePointsCount=len(unmachinablePoints)
        )


def analyzeMoldAccessibility(
        mesh: trimesh.Trimesh,
        numSamples: int = 10000,
        raysPerPoint: int = 64,
        exportCsv: bool = True
) -> MachiningRegionResult:
    analyzer = AccessibilityAnalyzer(mesh)
    result = analyzer.analyze(numSamples=numSamples, raysPerPoint=raysPerPoint, exportCsv=exportCsv)
    print(f"Total sampled points:       {result.totalPoints}")
    print(f"Unmachining points:         {result.unmachinablePointsCount}")
    print(f"Unmachining percentage:     {result.unmachinablePercent:.2f}%")
    return result


if __name__ == "__main__":
    from geometryAdapters import loadMeshFromFile
    stlFilePath = "testModels/hollow.cylinder.down.stl"
    mesh = loadMeshFromFile(stlFilePath)
    result = analyzeMoldAccessibility(mesh=mesh, numSamples=5000, raysPerPoint=90, exportCsv=False)
