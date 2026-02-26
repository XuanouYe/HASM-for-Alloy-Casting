import numpy as np
import trimesh
from typing import Tuple, Optional
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
        if isinstance(meshOrScene, trimesh.Trimesh):
            return meshOrScene
        if isinstance(meshOrScene, trimesh.Scene):
            dumped = meshOrScene.dump(concatenate=True)
            if isinstance(dumped, trimesh.Trimesh):
                return dumped
        raise TypeError(f"Unsupported mesh type: {type(meshOrScene)}")

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
        blankMesh = trimesh.creation.box(bounds=bbox)
        blankMesh.process(validate=True)
        return blankMesh

    def _booleanDifference(self, blankMesh: trimesh.Trimesh, inputMesh: trimesh.Trimesh) -> trimesh.Trimesh:
        blank = blankMesh.copy()
        part = inputMesh.copy()
        blank.process(validate=True)
        part.process(validate=True)
        enginesToTry = []
        if self.booleanEngine is not None:
            enginesToTry.append(self.booleanEngine)
        for e in ["manifold", "blender", "scad", None]:
            if e not in enginesToTry:
                enginesToTry.append(e)
        lastError = None
        for engine in enginesToTry:
            try:
                result = trimesh.boolean.difference([blank, part], engine=engine, check_volume=False)
                if result is None:
                    raise RuntimeError(f"Boolean returned None (engine={engine})")
                if isinstance(result, trimesh.Scene):
                    result = result.dump(concatenate=True)
                if not isinstance(result, trimesh.Trimesh):
                    raise RuntimeError(f"Unexpected boolean result type: {type(result)}")
                result.process(validate=True)
                return result
            except Exception as exc:
                lastError = exc
        raise RuntimeError(
            "Boolean difference failed with all engines tried."
        ) from lastError

    def generateGating(self, castingMesh: trimesh.Trimesh, config: dict) -> trimesh.Trimesh:
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
                             gatingMesh: Optional[trimesh.Trimesh]) -> Tuple[trimesh.Trimesh, trimesh.Trimesh, Optional[trimesh.Trimesh], np.ndarray]:
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

    def generateMoldAssets(self, inputStlPath: str, paths: dict, config: dict) -> np.ndarray:
        from geometryAdapters import loadMeshFromFile, exportMeshToStl
        partMesh = loadMeshFromFile(inputStlPath)
        moldShell = self.generateMoldShell(partMesh)
        gatingMesh = None
        if config.get("addGating", False):
            gatingMesh = self.generateGating(partMesh, config)
        if config.get("optimizeOrientation", False):
            moldShell = self.optimizeOrientation(moldShell, config)
        if config.get("adjustStructure", False):
            moldShell = self.adjustStructure(moldShell, config)
        partMesh, moldShell, gatingMesh, transformMatrix = self.normalizeMeshesToWcs(partMesh, moldShell, gatingMesh)
        exportMeshToStl(partMesh, paths["part"])
        exportMeshToStl(moldShell, paths["moldShell"])
        if gatingMesh is not None:
            exportMeshToStl(gatingMesh, paths["gating"])
        return transformMatrix


def main():
    from manufacturingManifest import ManufacturingManifest
    inputStlPath = "testModels/cube.with.groove.stl"
    projectId = "test_project_001"
    manifest = ManufacturingManifest(projectId)
    paths = {
        "part": "testModels/cube.with.groove.normalized.stl",
        "moldShell": "testModels/cube.with.groove.mold.normalized.stl",
        "gating": "testModels/cube.with.groove.gating.normalized.stl"
    }
    config = {
        "boundingBoxOffset": 2.0,
        "supportAngle": 45.0,
        "layerHeight": 0.2,
        "smoothHeight": 0.4,
        "booleanEngine": None,
        "addGating": True,
        "optimizeOrientation": False,
        "adjustStructure": False,
        "targetFillTime": 5.0,
        "sprueInletOffset": 5.0
    }
    moldGen = MoldGenerator(config=config)
    transformMatrix = moldGen.generateMoldAssets(inputStlPath, paths, config)
    manifest.setWcsTransform(transformMatrix)
    manifest.addParameters("moldConfig", config)
    manifest.addFile("partStl", paths["part"])
    manifest.addFile("moldStl", paths["moldShell"])
    if config.get("addGating", False):
        manifest.addFile("gatingStl", paths["gating"])
    manifest.save(f"{projectId}_manifest.json")


if __name__ == "__main__":
    main()
