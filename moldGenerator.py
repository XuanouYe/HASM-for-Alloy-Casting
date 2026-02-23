import numpy as np
import trimesh
from moldGatingSystem import createGatingSystem
from moldSupportRegionDetector import calculateSupportRegions


class MoldGenerator:
    def __init__(self, config=None):
        self.config = config or {}
        self.boundingBoxOffset = float(self.config.get("boundingBoxOffset", 2.0))
        self.booleanEngine = self.config.get("booleanEngine", None)

    def generateMoldShell(self, inputMesh: trimesh.Trimesh) -> trimesh.Trimesh:
        inputMesh = self.ensureTrimesh(inputMesh)
        boundingBox = self._calculateBoundingBox(inputMesh)
        blankMesh = self._createBlankMesh(boundingBox)
        moldShell = self._booleanDifference(blankMesh, inputMesh)
        return moldShell

    def ensureTrimesh(self, meshOrScene):
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
                    raise RuntimeError(f"Unexpected boolean result type: {type(result)} (engine={engine})")
                result.process(validate=True)
                return result
            except Exception as exc:
                lastError = exc

        raise RuntimeError(
            "Boolean difference failed with all engines tried. "
            "Try installing/configuring a backend (manifold or blender) and ensure meshes are watertight."
        ) from lastError

    def addGating(self, castingMesh: trimesh.Trimesh, config: dict) -> trimesh.Trimesh:
        gatingConfig = {
            "targetFillTime": config.get("targetFillTime", 5.0),
            "sprueInletOffset": config.get("sprueInletOffset", 5.0),
            "boundingBoxOffset": config.get("boundingBoxOffset", 2.0)
        }
        gatingMesh = createGatingSystem(
            castingMesh=castingMesh,
            config=gatingConfig,
            visualize=False
        )
        combinedMesh = trimesh.util.concatenate([castingMesh, gatingMesh])
        combinedMesh.fix_normals()
        return combinedMesh

    def optimizeOrientation(moldShell: trimesh.Trimesh, config: dict) -> trimesh.Trimesh:
        return moldShell

    def adjustStructure(moldShell: trimesh.Trimesh, config: dict) -> trimesh.Trimesh:
        return moldShell


def main():
    inputStlPath = "testModels/hollow.cylinder.down.stl"
    outputPath = "cylinder.down.mold.stl"

    config = {
        "boundingBoxOffset": 2.0,
        "supportAngle": 45.0,
        "layerHeight": 0.2,
        "smoothHeight": 0.4,
        "booleanEngine": None,
        "addGating": False,
        "optimizeOrientation": False,
        "adjustStructure": False,
        "targetFillTime": 5.0,
        "sprueInletOffset": 5.0
    }

    inputMesh = trimesh.load(inputStlPath)
    moldGen = MoldGenerator(config=config)
    moldShell = moldGen.generateMoldShell(inputMesh)

    if config.get("addGating", False):
        castingMesh = moldGen.ensureTrimesh(inputMesh)
        castingWithGating = moldGen.addGating(castingMesh, config)
        castingWithGating.export("casting.with.gating.stl")

    supportRegions = calculateSupportRegions(moldShell, config)
    print(f"Support regions: {len(supportRegions)}")

    if config.get("optimizeOrientation", False):
        moldShell = moldGen.optimizeOrientation(moldShell, config)

    if config.get("adjustStructure", False):
        moldShell = moldGen.adjustStructure(moldShell, config)

    moldShell.export(outputPath)

    inputMeshObj = moldGen.ensureTrimesh(inputMesh)
    print(f"Model bounds: {inputMeshObj.bounds}")
    print(f"Mold shell bounds: {moldShell.bounds}")
    print(f"Mold shell watertight: {moldShell.is_watertight}")
    print(f"Mold shell vertices: {len(moldShell.vertices)}")
    print(f"Mold shell faces: {len(moldShell.faces)}")
    print(f"Output file: {outputPath}")


if __name__ == "__main__":
    main()