"""
Mold Generation Module
This module provides functionality for generating mold shells from 3D meshes.
"""

import numpy as np
import trimesh

from moldSupportRegionDetector import calculateSupportRegions


class MoldGenerator:
    """
    A class to generate mold shells from 3D input meshes.

    The generator creates a bounding box around the input mesh and performs
    boolean operations to create the negative mold cavity.
    """

    def __init__(self, config=None):
        """
        Initialize the MoldGenerator with configuration parameters.

        Args:
            config (dict, optional): Configuration dictionary containing:
                - boundingBoxOffset (float): Offset for bounding box expansion
                - booleanEngine (str): Preferred boolean operation engine
        """
        self.config = config or {}
        self.boundingBoxOffset = float(self.config.get("boundingBoxOffset", 5.0))
        self.booleanEngine = self.config.get("booleanEngine", None)

    def generateMoldShell(self, inputMesh: trimesh.Trimesh) -> trimesh.Trimesh:
        """
        Generate a mold shell from the input mesh.

        Args:
            inputMesh (trimesh.Trimesh): The 3D mesh to create a mold for

        Returns:
            trimesh.Trimesh: The generated mold shell mesh
        """
        inputMesh = self._ensureTrimesh(inputMesh)
        boundingBox = self._calculateBoundingBox(inputMesh)
        blankMesh = self._createBlankMesh(boundingBox)
        moldShell = self._booleanDifference(blankMesh, inputMesh)
        return moldShell

    def _ensureTrimesh(self, meshOrScene):
        """
        Ensure the input is a trimesh.Trimesh object.

        Args:
            meshOrScene: Either a trimesh.Trimesh or trimesh.Scene object

        Returns:
            trimesh.Trimesh: A single mesh object

        Raises:
            TypeError: If the input type is not supported
        """
        if isinstance(meshOrScene, trimesh.Trimesh):
            return meshOrScene
        if isinstance(meshOrScene, trimesh.Scene):
            dumped = meshOrScene.dump(concatenate=True)
            if isinstance(dumped, trimesh.Trimesh):
                return dumped
        raise TypeError(f"Unsupported mesh type: {type(meshOrScene)}")

    def _calculateBoundingBox(self, mesh: trimesh.Trimesh) -> np.ndarray:
        """
        Calculate an expanded bounding box for the mold blank.

        Args:
            mesh (trimesh.Trimesh): Input mesh to calculate bounds for

        Returns:
            np.ndarray: 2x3 array of [[min_x, min_y, min_z], [max_x, max_y, max_z]]
        """
        bounds = np.array(mesh.bounds, dtype=float)
        offset = float(self.boundingBoxOffset)
        bounds[0, :] -= offset
        bounds[1, :] += offset
        # Ensure minimum thickness
        eps = 1e-6
        bounds[1, :] = np.maximum(bounds[1, :], bounds[0, :] + eps)
        return bounds

    def _createBlankMesh(self, bbox: np.ndarray) -> trimesh.Trimesh:
        """
        Create a solid box mesh from bounding box coordinates.

        Args:
            bbox (np.ndarray): Bounding box coordinates

        Returns:
            trimesh.Trimesh: A solid box mesh
        """
        blankMesh = trimesh.creation.box(bounds=bbox)
        blankMesh.process(validate=True)
        return blankMesh

    def _booleanDifference(self, blankMesh: trimesh.Trimesh, inputMesh: trimesh.Trimesh) -> trimesh.Trimesh:
        """
        Perform boolean difference operation to create mold cavity.

        Args:
            blankMesh (trimesh.Trimesh): Solid box mesh
            inputMesh (trimesh.Trimesh): Input part mesh

        Returns:
            trimesh.Trimesh: Mold shell with cavity

        Raises:
            RuntimeError: If boolean operation fails with all available engines
        """
        blank = blankMesh.copy()
        part = inputMesh.copy()
        blank.process(validate=True)
        part.process(validate=True)

        # Define engine priority list
        enginesToTry = []
        if self.booleanEngine is not None:
            enginesToTry.append(self.booleanEngine)
        for e in ["manifold", "blender", "scad", None]:
            if e not in enginesToTry:
                enginesToTry.append(e)

        # Try boolean operations with different engines
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


def addGating(moldShell: trimesh.Trimesh, config: dict) -> trimesh.Trimesh:
    """
    Add gating system to the mold shell.

    Args:
        moldShell (trimesh.Trimesh): The mold shell mesh
        config (dict): Configuration parameters

    Returns:
        trimesh.Trimesh: Mold shell with gating system
    """
    return moldShell


def optimizeOrientation(moldShell: trimesh.Trimesh, config: dict) -> trimesh.Trimesh:
    """
    Optimize mold orientation for manufacturing.

    Args:
        moldShell (trimesh.Trimesh): The mold shell mesh
        config (dict): Configuration parameters

    Returns:
        trimesh.Trimesh: Reoriented mold shell
    """
    return moldShell


def adjustStructure(moldShell: trimesh.Trimesh, config: dict) -> trimesh.Trimesh:
    """
    Adjust mold structure for strength and manufacturability.

    Args:
        moldShell (trimesh.Trimesh): The mold shell mesh
        config (dict): Configuration parameters

    Returns:
        trimesh.Trimesh: Adjusted mold shell
    """
    return moldShell


def main():
    """
    Main execution function for mold generation.

    Loads an input STL, generates a mold shell, applies optional modifications,
    and exports the result.
    """
    # Configuration
    inputStlPath = "cube2vertical.stl"
    outputPath = "cube2vertical.mold.stl"
    config = {
        "boundingBoxOffset": 5.0,
        "supportAngle": 45.0,
        "layerHeight": 0.2,
        "smoothHeight": 0.4,
        "booleanEngine": None,
        "addGating": False,
        "optimizeOrientation": False,
        "adjustStructure": False
    }

    # Load input mesh
    inputMesh = trimesh.load(inputStlPath)

    # Generate mold shell
    moldGen = MoldGenerator(config=config)
    moldShell = moldGen.generateMoldShell(inputMesh)

    # Apply optional modifications
    if config.get("addGating", False):
        moldShell = addGating(moldShell, config)

    # Calculate support regions
    supportRegions = calculateSupportRegions(moldShell, config)
    print(f"Support regions: {len(supportRegions)}")

    if config.get("optimizeOrientation", False):
        moldShell = optimizeOrientation(moldShell, config)

    if config.get("adjustStructure", False):
        moldShell = adjustStructure(moldShell, config)

    # Export results
    moldShell.export(outputPath)

    # Print statistics
    inputMeshObj = moldGen._ensureTrimesh(inputMesh)
    print(f"Model bounds: {inputMeshObj.bounds}")
    print(f"Mold shell bounds: {moldShell.bounds}")
    print(f"Mold shell watertight: {moldShell.is_watertight}")
    print(f"Mold shell vertices: {len(moldShell.vertices)}")
    print(f"Mold shell faces: {len(moldShell.faces)}")
    print(f"Output file: {outputPath}")


if __name__ == "__main__":
    main()
