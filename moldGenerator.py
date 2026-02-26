import numpy as np
import trimesh
from typing import Tuple, Optional, Dict
from moldGatingSystem import createGatingSystem
from moldSupportRegionDetector import calculateSupportRegions
from moldOrientationOptimizer import optimizeMoldOrientation
from moldInnerSurfaceOffset import InnerSurfaceOffset

class MoldGenerator:
    def __init__(self, config=None):
        self.config = config or {}
        self.boundingBoxOffset = float(self.config.get("boundingBoxOffset", 2.0))
        self.booleanEngine = self.config.get("booleanEngine", None)
        self.intermediateResults = {}

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
        bounds[1, :] += offset 
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

    def optimizeOrientation(self, inputMesh: trimesh.Trimesh, config: dict) -> trimesh.Trimesh:
        # Step 1: Optimization
        print("Running Orientation Optimization...")
        results = optimizeMoldOrientation(
            inputCasting=inputMesh,
            boundingBoxOffset=float(config.get("boundingBoxOffset", 2.0)),
            supportAngle=float(config.get("supportAngle", 45.0)),
            visualize=False
        )
        optimizedMesh = results.get("bestMesh", inputMesh)
        self.intermediateResults["optimizedMesh"] = optimizedMesh
        return optimizedMesh

    def generateMoldShell(self, inputMesh: trimesh.Trimesh) -> trimesh.Trimesh:
        # Step 2: Shell Generation
        print("Generating Mold Shell...")
        inputMesh = self.ensureTrimesh(inputMesh)
        boundingBox = self._calculateBoundingBox(inputMesh)
        blankMesh = self._createBlankMesh(boundingBox)
        moldShell = self._booleanDifference(blankMesh, inputMesh)
        self.intermediateResults["moldShell"] = moldShell
        return moldShell

    def adjustStructure(self, moldShell: trimesh.Trimesh, config: dict) -> trimesh.Trimesh:
        # Step 3: Inner Surface Offset (Overhangs)
        print("Adjusting Structure (Inner Surface Offset)...")
        offsetProcessor = InnerSurfaceOffset(config)
        adjustedMold = offsetProcessor.removeInnerSurfaceOverhangs(moldShell)
        self.intermediateResults["adjustedMold"] = adjustedMold
        return adjustedMold

    def generateGating(self, castingMesh: trimesh.Trimesh, config: dict) -> trimesh.Trimesh:
        # Step 4: Gating System
        print("Generating Gating System...")
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
        self.intermediateResults["gatingMesh"] = gatingMesh
        return gatingMesh

    def normalizeMeshes(self, partMesh: trimesh.Trimesh, moldMesh: trimesh.Trimesh, gatingMesh: Optional[trimesh.Trimesh]) -> Tuple[trimesh.Trimesh, trimesh.Trimesh, Optional[trimesh.Trimesh], np.ndarray]:
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
            
        return partMesh, moldMesh, gatingMesh, translation

    def generateMoldAssets(self, inputStlPath: str, paths: dict, config: dict) -> np.ndarray:
        self.intermediateResults = {} # Reset intermediate results
        inputMesh = trimesh.load(inputStlPath)
        partMesh = self.ensureTrimesh(inputMesh)
        
        # 1. Orientation Optimization
        if config.get("optimizeOrientation", False):
            partMesh = self.optimizeOrientation(partMesh, config)

        # 2. Mold Shell Generation
        moldShell = self.generateMoldShell(partMesh)
        
        # 3. Inner Surface Adjustment
        if config.get("adjustStructure", False):
            moldShell = self.adjustStructure(moldShell, config)

        # 4. Gating System
        gatingMesh = None
        if config.get("addGating", False):
            gatingMesh = self.generateGating(partMesh, config)

        # 5. Normalization
        partMesh, moldShell, gatingMesh, translation = self.normalizeMeshes(partMesh, moldShell, gatingMesh)

        partMesh.export(paths["part"])
        moldShell.export(paths["moldShell"])
        if gatingMesh is not None:
            gatingMesh.export(paths["gating"])
            
        return translation

    def getIntermediateResult(self, stepName: str) -> Optional[trimesh.Trimesh]:
        return self.intermediateResults.get(stepName)

def main():
    from dataModel import ManifestManager
    inputStlPath = "testModels/cube.with.groove.stl"
    
    workspace = "workspace_test"
    manifestMgr = ManifestManager(workspace)
    paths = manifestMgr.getFilePaths()

    config = {
        "boundingBoxOffset": 2.0,
        "supportAngle": 45.0,
        "layerHeight": 0.2,
        "smoothHeight": 0.4,
        "booleanEngine": None,
        "addGating": True,
        "optimizeOrientation": True,
        "adjustStructure": True,
        "targetFillTime": 5.0,
        "sprueInletOffset": 5.0
    }

    moldGen = MoldGenerator(config=config)
    translation = moldGen.generateMoldAssets(inputStlPath, paths, config)
    
    manifestMgr.setWcsTransform(translation.tolist())
    manifestMgr.setConfigSnapshot(config)
    manifestPath = manifestMgr.save()
    
    print(f"Assets generated and normalized. Manifest saved to {manifestPath}")

if __name__ == "__main__":
    main()