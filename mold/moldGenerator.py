import numpy as np
from typing import List, Optional

from dataModel import GatingComponents
from mold.gatingSystem import createGatingSystem
from pathlib import Path
import trimesh

from geometryAdapters import loadMeshFromFile, exportMeshToStl


class MoldGenerator:
    def __init__(self, config: Optional[dict] = None):
        self.config = config or {}
        self.boundingBoxOffset = float(self.config.get("boundingBoxOffset", 2.0))
        self.booleanEngine = self.config.get("booleanEngine", "manifold")

    def generateMoldShell(self, inputMesh: trimesh.Trimesh) -> trimesh.Trimesh:
        inputMesh = self.ensureTrimesh(inputMesh)
        boundingBox = self._calculateBoundingBox(inputMesh)
        blankMesh = self._createBlankMesh(boundingBox)
        return self._booleanDifference(blankMesh, inputMesh)

    def ensureTrimesh(self, meshOrScene) -> trimesh.Trimesh:
        if isinstance(meshOrScene, trimesh.Scene):
            return meshOrScene.dump(concatenate=True)
        return meshOrScene

    def _calculateBoundingBox(self, mesh: trimesh.Trimesh) -> np.ndarray:
        bounds = np.array(mesh.bounds, dtype=float)
        offset = self.boundingBoxOffset
        bounds[0, :] -= offset
        bounds[1, 0] += offset
        bounds[1, 1] += offset
        bounds[1, :] = np.maximum(bounds[1, :], bounds[0, :] + 1e-6)
        return bounds

    def _createBlankMesh(self, bbox: np.ndarray) -> trimesh.Trimesh:
        return trimesh.creation.box(bounds=bbox)

    def _booleanDifference(self, blankMesh: trimesh.Trimesh, inputMesh: trimesh.Trimesh) -> trimesh.Trimesh:
        result = trimesh.boolean.difference([blankMesh, inputMesh], engine=self.booleanEngine, check_volume=False)
        return self.ensureTrimesh(result)

    def generateGating(self, castingMesh: trimesh.Trimesh) -> GatingComponents:
        gatingConfig = {
            "targetFillTime": self.config.get("targetFillTime", 5.0),
            "sprueInletOffset": self.config.get("sprueInletOffset", 5.0),
            "boundingBoxOffset": self.boundingBoxOffset,
            "runnerDiameter": self.config.get("runnerDiameter", 6.0)
        }
        return createGatingSystem(castingMesh=castingMesh, config=gatingConfig)

    def optimizeOrientation(self, partMesh: trimesh.Trimesh) -> trimesh.Trimesh:
        return partMesh

    def adjustStructure(self, moldShell: trimesh.Trimesh) -> trimesh.Trimesh:
        return moldShell

    def normalizeMeshesToWcs(self, meshesToConsider: List[trimesh.Trimesh]) -> np.ndarray:
        scene = trimesh.Scene(meshesToConsider)
        bounds = scene.bounds
        centerX = (bounds[0][0] + bounds[1][0]) / 2.0
        centerY = (bounds[0][1] + bounds[1][1]) / 2.0
        minZ = bounds[0][2]

        matrix = np.eye(4)
        matrix[:3, 3] = [-centerX, -centerY, -minZ]

        for mesh in meshesToConsider:
            mesh.apply_transform(matrix)

        return matrix


def executeMoldWorkflow(
        inputStlPath: str,
        outputStlPath: str,
        tempCncDir: str = "../tempCncFiles",
        adjustOrientation: bool = False,
        addGating: bool = True,
        surfaceOffset: bool = False,
        config: Optional[dict] = None
) -> trimesh.Trimesh:
    workflowConfig = config or {}
    moldGen = MoldGenerator(workflowConfig)
    currentMesh = moldGen.ensureTrimesh(loadMeshFromFile(inputStlPath))

    if adjustOrientation:
        currentMesh = moldGen.optimizeOrientation(currentMesh)

    gateMesh = None
    riserMesh = None
    gatingMeshForNorm = None
    cavityMesh = currentMesh

    if addGating:
        gatingComponents = moldGen.generateGating(currentMesh)
        gateMesh = gatingComponents.gateMesh
        riserMesh = gatingComponents.riserMesh
        gatingMeshForNorm = gatingComponents.systemMesh
        cavityMesh = gatingComponents.castingWithSystemMesh

    moldShell = moldGen.generateMoldShell(cavityMesh)

    if surfaceOffset:
        moldShell = moldGen.adjustStructure(moldShell)

    meshesToNormalize = [currentMesh, moldShell]
    if gatingMeshForNorm is not None:
        meshesToNormalize.extend([gateMesh, riserMesh, gatingMeshForNorm])

    moldGen.normalizeMeshesToWcs(meshesToNormalize)

    exportMeshToStl(moldShell, outputStlPath)

    tempDirPath = Path(tempCncDir)
    tempDirPath.mkdir(parents=True, exist_ok=True)
    exportMeshToStl(currentMesh, str(tempDirPath / "part.stl"))
    exportMeshToStl(moldShell, str(tempDirPath / "mold.stl"))

    if gateMesh is not None:
        exportMeshToStl(gateMesh, str(tempDirPath / "gate.stl"))
    if riserMesh is not None:
        exportMeshToStl(riserMesh, str(tempDirPath / "riser.stl"))

    return moldShell


if __name__ == '__main__':
    testInputPath = "../testModels/half.cylinder.stl"
    testOutputPath = "../testModels/half.cylinder.mold.stl"
    finalMold = executeMoldWorkflow(
        inputStlPath=testInputPath,
        outputStlPath=testOutputPath,
        adjustOrientation=False,
        addGating=True,
        surfaceOffset=False,
    )