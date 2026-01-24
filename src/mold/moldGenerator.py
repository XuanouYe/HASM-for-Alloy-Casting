from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Union

import numpy as np
import trimesh


PathLike = Union[str, Path]


def _loadAsSingleMesh(path: PathLike) -> trimesh.Trimesh:
    """
    Load model and ensure returning Trimesh (merge geometry if Scene).
    """
    mesh = trimesh.load(str(path), force="mesh")

    # trimesh.load(force="mesh") usually returns Trimesh;
    # but handle Scene case for safety
    if isinstance(mesh, trimesh.Scene):
        geoms = list(mesh.geometry.values())
        if len(geoms) == 0:
            raise ValueError(f"No usable mesh found in file: {path}")
        mesh = trimesh.util.concatenate(geoms)

    if not isinstance(mesh, trimesh.Trimesh):
        raise TypeError(f"Load result is not Trimesh: {type(mesh)}")

    return mesh


def _ensureWatertight(mesh: trimesh.Trimesh, fix: bool = True) -> trimesh.Trimesh:
    """
    Ensure mesh is watertight; boolean operations typically require closed mesh.
    """
    if mesh.is_watertight:
        return mesh

    if not fix:
        raise ValueError("Casting mesh is not watertight and auto-fix is not enabled.")

    # Attempt repair: fill holes + fix normals etc.
    mesh = mesh.copy()
    mesh.remove_duplicate_faces()
    mesh.remove_degenerate_faces()
    mesh.remove_unreferenced_vertices()
    mesh.merge_vertices()

    # Simple hole filling
    mesh.fill_holes()

    # Clean up again
    mesh.remove_duplicate_faces()
    mesh.remove_degenerate_faces()
    mesh.remove_unreferenced_vertices()
    mesh.merge_vertices()

    if not mesh.is_watertight:
        raise ValueError(
            "Casting mesh remains non-watertight after repair. "
            "Please fix as closed solid in CAD/Blender/MeshLab before boolean operation."
        )
    return mesh


def _toFloat64(mesh: trimesh.Trimesh) -> trimesh.Trimesh:
    """
    Convert vertices to float64, typically more stable for boolean operations.
    """
    mesh = mesh.copy()
    mesh.vertices = np.asarray(mesh.vertices, dtype=np.float64)
    return mesh


def _makeMoldBoxFromBounds(bounds: np.ndarray, thickness: float) -> trimesh.Trimesh:
    """
    Generate cuboid mold from casting bounds and wall thickness (axis-aligned bounding box expansion).
    bounds: shape (2, 3) -> [[xmin,ymin,zmin],[xmax,ymax,zmax]]
    """
    bounds = np.asarray(bounds, dtype=np.float64)
    if bounds.shape != (2, 3):
        raise ValueError(f"Bounds shape should be (2,3), got {bounds.shape}")

    if thickness <= 0:
        raise ValueError("Wall thickness t must be > 0")

    moldMin = bounds[0] - thickness
    moldMax = bounds[1] + thickness

    extents = moldMax - moldMin
    center = (moldMin + moldMax) / 2.0

    # Create box centered at origin, then translate to center
    transform = trimesh.transformations.translation_matrix(center)
    box = trimesh.creation.box(extents=extents, transform=transform)

    return box


@dataclass
class MoldConfig:
    wallThickness: float
    fixWatertight: bool = True
    checkVolume: bool = False
    outputPath: Optional[PathLike] = None


def generateMoldWithManifold(castingPath: PathLike, config: MoldConfig) -> trimesh.Trimesh:
    """
    Main function: generate mold (cube - casting), boolean engine fixed to manifold (manifold3d backend).
    """
    casting = _loadAsSingleMesh(castingPath)
    casting = _toFloat64(casting)
    # casting = _ensureWatertight(casting, fix=config.fixWatertight)

    moldBox = _makeMoldBoxFromBounds(casting.bounds, config.wallThickness)
    moldBox = _toFloat64(moldBox)

    mold = moldBox.difference(casting, engine="manifold", check_volume=config.checkVolume)

    if mold is None or mold.is_empty:
        raise RuntimeError("Boolean operation result is empty. Check if models intersect, are closed, and have reasonable units/scale.")

    if config.outputPath is not None:
        Path(config.outputPath).parent.mkdir(parents=True, exist_ok=True)
        mold.export(str(config.outputPath))

    return mold


def main():
    """
    Test mold generation function.
    Generate mold from existing STL file path to current directory.
    """
    import os

    # Test STL file path (replace with actual existing STL file path)
    stlFilePath = "monk.reversed.stl"  # Replace with your STL file path

    # Check if file exists
    if not Path(stlFilePath).exists():
        print(f"Error: STL file does not exist: {stlFilePath}")
        print("Please provide a valid STL file path")
        return

    # Mold configuration
    config = MoldConfig(
        wallThickness=10.0,  # Wall thickness (adjust based on your model size)
        fixWatertight=True,  # Automatically fix non-watertight meshes
        checkVolume=False,  # Turn off volume check (try turning off if boolean operation fails)
        outputPath="mold.result.stl"  # Output file path
    )

    try:
        print(f"Loading STL file: {stlFilePath}")
        print(f"Wall thickness: {config.wallThickness}")
        print(f"Output file: {config.outputPath}")

        # Generate mold
        mold = generateMoldWithManifold(stlFilePath, config)

        print("✓ Mold generation successful!")
        print(f"Mold vertices: {len(mold.vertices)}")
        print(f"Mold faces: {len(mold.faces)}")

        if Path(config.outputPath).exists():
            print(f"✓ Mold saved to: {os.path.abspath(config.outputPath)}")

        # Print some basic information
        print(f"Mold bounding box:")
        print(f"  X range: [{mold.bounds[0][0]:.2f}, {mold.bounds[1][0]:.2f}]")
        print(f"  Y range: [{mold.bounds[0][1]:.2f}, {mold.bounds[1][1]:.2f}]")
        print(f"  Z range: [{mold.bounds[0][2]:.2f}, {mold.bounds[1][2]:.2f}]")

        # Check if mold is watertight
        if mold.is_watertight:
            print("✓ Mold is watertight")
            print(f"Volume: {mold.volume:.2f}")
        else:
            print("⚠ Warning: Mold is not watertight")

    except FileNotFoundError as e:
        print(f"File error: {e}")
    except ValueError as e:
        print(f"Value error: {e}")
    except RuntimeError as e:
        print(f"Runtime error: {e}")
    except Exception as e:
        print(f"Unknown error: {type(e).__name__}: {e}")


if __name__ == "__main__":
    main()