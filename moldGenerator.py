import numpy as np
import trimesh
from typing import Tuple, Optional
from pathlib import Path

from dataModel import GatingComponents
from geometryAdapters import loadMeshFromFile, exportMeshToStl
from moldGatingSystem import createGatingSystem


class MoldGenerator:
    def __init__(self, config=None):
        self.config = config or {}
        self.boundingBoxOffset = float(self.config.get("boundingBoxOffset", 2.0))
        self.booleanEngine = self.config.get("booleanEngine", None)

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
        offset = float(self.boundingBoxOffset)
        bounds[0, :] -= offset
        bounds[1, 0] += offset
        bounds[1, 1] += offset
        eps = 1e-6
        bounds[1, :] = np.maximum(bounds[1, :], bounds[0, :] + eps)
        return bounds

    def _createBlankMesh(self, bbox: np.ndarray) -> trimesh.Trimesh:
        return trimesh.creation.box(bounds=bbox)

    def _booleanDifference(self, blankMesh: trimesh.Trimesh, inputMesh: trimesh.Trimesh) -> trimesh.Trimesh:
        enginesToTry = []
        if self.booleanEngine:
            enginesToTry.append(self.booleanEngine)
        for e in ["manifold", "scad", "blender", None]:
            if e not in enginesToTry:
                enginesToTry.append(e)
        for engine in enginesToTry:
            try:
                result = trimesh.boolean.difference([blankMesh, inputMesh], engine=engine, check_volume=False)
                if isinstance(result, trimesh.Scene):
                    result = result.dump(concatenate=True)
                if isinstance(result, trimesh.Trimesh):
                    return result
            except Exception:
                continue
        raise RuntimeError("Boolean difference failed with all engines")

    def generateGating(self, castingMesh: trimesh.Trimesh, config: dict) -> GatingComponents:
        gatingConfig = {
            "targetFillTime": config.get("targetFillTime", 5.0),
            "sprueInletOffset": config.get("sprueInletOffset", 5.0),
            "boundingBoxOffset": config.get("boundingBoxOffset", 2.0)
        }
        return createGatingSystem(castingMesh=castingMesh, config=gatingConfig)

    def optimizeOrientation(self, moldShell: trimesh.Trimesh, config: dict) -> trimesh.Trimesh:
        return moldShell

    def adjustStructure(self, moldShell: trimesh.Trimesh, config: dict) -> trimesh.Trimesh:
        return moldShell

    def normalizeMeshesToWcs(self, partMesh: trimesh.Trimesh, moldMesh: trimesh.Trimesh,
                             gatingMesh: Optional[trimesh.Trimesh]) -> Tuple[
        trimesh.Trimesh, trimesh.Trimesh, Optional[trimesh.Trimesh], np.ndarray]:
        meshesToConsider = [partMesh, moldMesh]
        if gatingMesh is not None:
            meshesToConsider.append(gatingMesh)
        minZ = min(mesh.bounds[0][2] for mesh in meshesToConsider)
        minX = min(mesh.bounds[0][0] for mesh in meshesToConsider)
        maxX = max(mesh.bounds[1][0] for mesh in meshesToConsider)
        minY = min(mesh.bounds[0][1] for mesh in meshesToConsider)
        maxY = max(mesh.bounds[1][1] for mesh in meshesToConsider)
        centerX = (minX + maxX) / 2.0
        centerY = (minY + maxY) / 2.0
        translation = np.array([-centerX, -centerY, -minZ])
        matrix = np.eye(4)
        matrix[:3, 3] = translation
        partMesh.apply_transform(matrix)
        moldMesh.apply_transform(matrix)
        if gatingMesh is not None:
            gatingMesh.apply_transform(matrix)
        return partMesh, moldMesh, gatingMesh, matrix


def executeMoldWorkflow(
        inputStlPath: str,
        outputStlPath: str,
        adjustOrientation: bool = False,
        addGating: bool = True,
        surfaceOffset: bool = False,
        config: Optional[dict] = None
) -> trimesh.Trimesh:
    workflowConfig = config or {}
    moldGen = MoldGenerator(workflowConfig)
    originalMesh = loadMeshFromFile(inputStlPath)
    currentMesh = moldGen.ensureTrimesh(originalMesh)

    if adjustOrientation:
        currentMesh = moldGen.optimizeOrientation(currentMesh, workflowConfig)

    gatingMesh = None
    cavityMesh = currentMesh

    if addGating:
        gatingComponents = moldGen.generateGating(currentMesh, workflowConfig)
        gatingMesh = gatingComponents.systemMesh
        cavityMesh = gatingComponents.castingWithSystemMesh

    moldShell = moldGen.generateMoldShell(cavityMesh)

    if surfaceOffset:
        moldShell = moldGen.adjustStructure(moldShell, workflowConfig)

    partMeshWcs, moldMeshWcs, gatingMeshWcs, transformMatrix = moldGen.normalizeMeshesToWcs(
        currentMesh, moldShell, gatingMesh
    )

    exportMeshToStl(moldMeshWcs, outputStlPath)

    return moldMeshWcs


if __name__ == '__main__':
    testInputPath = "testModels/cylinder.down.stl"
    testOutputPath = "testModels/cylinder.mold.stl"
    testConfig = {
        "targetFillTime": 3.0,
        "sprueInletOffset": 5.0,
        "boundingBoxOffset": 2.0,
        "booleanEngine": "manifold"
    }

    finalMold = executeMoldWorkflow(
        inputStlPath=testInputPath,
        outputStlPath=testOutputPath,
        adjustOrientation=False,
        addGating=True,
        surfaceOffset=False,
        config=testConfig
    )
