"""
Support Region Detection Module

This module provides functionality to detect overhang regions in a 3D mesh that require support structures
during 3D printing processes, based on configurable support angle and layer parameters.
"""

import numpy as np
import trimesh
from typing import List, Optional, Union
from shapely import ops
from shapely.geometry.base import BaseGeometry


class SupportRegionDetector:
    """
    Detects regions in a 3D mesh that require support structures during additive manufacturing.

    The detection algorithm identifies overhang areas by comparing consecutive layers
    and applying geometric offset operations based on configurable support angles.
    """

    def __init__(self, config: Optional[dict] = None):
        """
        Initialize the support region detector with configuration parameters.

        Args:
            config (dict, optional): Configuration dictionary with the following keys:
                - supportAngle: Maximum overhang angle (in degrees) that can be printed without support.
                  Default: 45.0
                - layerHeight: Height of each printing layer in millimeters. Default: 0.2
                - smoothHeight: Height range to consider for support detection smoothing. Default: 0.4
                - areaEps: Minimum area threshold for considering a region as significant. Default: 1e-6
        """
        self.config = config or {}
        self.supportAngle = float(self.config.get("supportAngle", 45.0))
        self.layerHeight = float(self.config.get("layerHeight", 0.2))
        self.smoothHeight = float(self.config.get("smoothHeight", 0.4))
        self.areaEps = float(self.config.get("areaEps", 1e-6))

    def calculateSupportRegions(self, mesh: trimesh.Trimesh,
                                sliceHeights: Optional[List[float]] = None) -> List[Optional[BaseGeometry]]:
        """
        Calculate support regions for each layer of the mesh.

        The algorithm works by:
        1. Slicing the mesh into horizontal layers
        2. For each layer, comparing with previous layers within the smoothing height range
        3. Identifying areas that are not supported by lower layers based on the support angle

        Args:
            mesh (trimesh.Trimesh): The 3D mesh to analyze for support requirements
            sliceHeights (List[float], optional): Specific heights at which to slice the mesh.
                If None, heights are automatically generated based on layerHeight.

        Returns:
            List[Optional[BaseGeometry]]: List of support regions for each layer, 
                where None indicates no support needed for that layer.
        """
        if sliceHeights is None:
            sliceHeights = self._generateSliceHeights(mesh)

        # Slice mesh into 2D contours for each layer
        layerPaths = self._sliceMeshToLayers(mesh, sliceHeights)

        # Convert 2D paths to Shapely geometries
        layerGeoms = [self._path2DToShapely(p) for p in layerPaths]

        # Calculate how many layers to look below for support consideration
        layersBelow = int(round(self.smoothHeight / self.layerHeight))
        maxDistFromLowerLayer = self._calculateMaxBridgeDistance()

        supportRegionsPerLayer = []

        # Analyze each layer for support requirements (starting from layer 1)
        for layerIdx in range(1, len(layerGeoms)):
            cur = layerGeoms[layerIdx]

            # Skip empty layers
            if cur is None or cur.is_empty:
                supportRegionsPerLayer.append(None)
                continue

            # Get merged geometry from lower layers within smoothing range
            mergedBelow = self._getMergedGeomsBelow(layerGeoms, layerIdx, layersBelow, maxDistFromLowerLayer)

            # If no geometry below, entire current layer needs support
            if mergedBelow is None or mergedBelow.is_empty:
                supportRegionsPerLayer.append(cur)
                continue

            # Calculate difference between current layer and supported areas from below
            diff = cur.difference(mergedBelow)

            # Check if the difference area is significant
            if diff.is_empty or diff.area < self.areaEps:
                supportRegionsPerLayer.append(None)
            else:
                supportRegionsPerLayer.append(diff)

        return supportRegionsPerLayer

    def _generateSliceHeights(self, mesh: trimesh.Trimesh) -> np.ndarray:
        """
        Generate equally spaced slicing heights based on mesh bounds and layer height.

        Args:
            mesh (trimesh.Trimesh): Input 3D mesh

        Returns:
            np.ndarray: Array of slicing heights
        """
        zMin, zMax = mesh.bounds[:, 2]
        return np.arange(zMin + self.layerHeight / 2, zMax, self.layerHeight)

    def _sliceMeshToLayers(self, mesh: trimesh.Trimesh, heights: np.ndarray) -> List:
        """
        Slice the 3D mesh at specified heights to obtain 2D contours.

        Args:
            mesh (trimesh.Trimesh): 3D mesh to slice
            heights (np.ndarray): Array of Z-heights at which to slice

        Returns:
            List: List of 2D path objects (or None for heights with no intersection)
        """
        out = []
        for h in heights:
            # Create horizontal slice at height h
            s = mesh.section(plane_origin=[0, 0, float(h)], plane_normal=[0, 0, 1])
            if s is None:
                out.append(None)
                continue

            # Convert 3D section to 2D polygon
            p2d, _ = s.to_2D(normal=[0, 0, 1])
            out.append(p2d)
        return out

    def _calculateMaxBridgeDistance(self) -> float:
        """
        Calculate the maximum horizontal distance that can be bridged without support.

        Based on the support angle and layer height using trigonometric relationship:
        maxDistance = tan(supportAngle) * layerHeight

        Returns:
            float: Maximum bridge distance in the XY plane
        """
        return np.tan(np.deg2rad(self.supportAngle)) * self.layerHeight

    def _path2DToShapely(self, path2d) -> Optional[BaseGeometry]:
        """
        Convert a trimesh Path2D object to a Shapely geometry.

        Args:
            path2d: trimesh Path2D object containing polygon contours

        Returns:
            Optional[BaseGeometry]: Shapely geometry representing the union of all polygons,
                or None if input is empty
        """
        if path2d is None or path2d.is_empty:
            return None

        # Extract all polygons from the path
        polys = list(path2d.polygons_full)
        if len(polys) == 0:
            return None

        # Combine all polygons into a single geometry
        return ops.unary_union(polys)

    def _getMergedGeomsBelow(self, layerGeoms: List[Optional[BaseGeometry]],
                             currentLayerIdx: int, layersBelow: int,
                             offsetDist: float) -> Optional[BaseGeometry]:
        """
        Merge and offset geometries from layers below the current layer.

        Lower layers are progressively offset (expanded) based on their distance
        from the current layer to account for the support angle.

        Args:
            layerGeoms (List[Optional[BaseGeometry]]): List of Shapely geometries for all layers
            currentLayerIdx (int): Index of the current layer being analyzed
            layersBelow (int): Number of lower layers to consider for support
            offsetDist (float): Base offset distance for the first layer below

        Returns:
            Optional[BaseGeometry]: Merged and offset geometry from lower layers,
                or None if no valid geometries found
        """
        merged = None

        # Process layers below current layer, up to layersBelow count
        for offset in range(1, min(layersBelow + 1, currentLayerIdx + 1)):
            g = layerGeoms[currentLayerIdx - offset]

            if g is None or g.is_empty:
                continue

            # Apply larger offset for layers further below
            expanded = g.buffer(offsetDist * offset)

            # Union with previously processed lower layers
            if merged is None:
                merged = expanded
            else:
                merged = merged.union(expanded)

        return merged


def calculateSupportRegions(moldShell: trimesh.Trimesh, config: dict) -> List[BaseGeometry]:
    """
    Convenience function to calculate support regions for a given mesh and configuration.

    Args:
        moldShell (trimesh.Trimesh): The 3D mesh/geometry to analyze
        config (dict): Configuration parameters for support detection

    Returns:
        List[BaseGeometry]: List of support regions (excluding None values)
    """
    detector = SupportRegionDetector(config)
    supportRegions = detector.calculateSupportRegions(moldShell)
    return [r for r in supportRegions if r is not None]