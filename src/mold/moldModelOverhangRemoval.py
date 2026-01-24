import time
import numpy as np
import trimesh

from typing import Tuple, Dict, List
from scipy.spatial import ConvexHull
from src.mold.moldModelOverhangDetection import OptimizedOverhangDetector

class OverhangStructureRemoval:
    """
    Implements the Overhang Structure Removal strategy from research objectives.

    This module follows the workflow:
    1. Extract overhanging surface patches
    2. Compute envelope contours at horizontal reference planes
    3. Calculate characteristic dimensions (ScaleEnvelope, HeightEnvelope)
    4. Generate removal geometries (cone or offset curves)
    5. Remove auxiliary bodies from the model

    Based on the research paper strategy for overhang elimination.
    """

    def __init__(self, mesh: trimesh.Trimesh, buildDirection: np.ndarray = None):
        """
        Initialize overhang removal module

        Args:
            mesh: Input mesh (typically the mold)
            buildDirection: Build direction vector (default [0, 0, -1])
        """
        self.mesh = mesh.copy()
        if buildDirection is None:
            self.buildDirection = np.array([0, 0, -1])
        else:
            self.buildDirection = buildDirection / np.linalg.norm(buildDirection)
        self.topSurfaceZ = np.max(self.mesh.vertices[:, 2])
        self.overhangFaces = None
        self.envelopeContours = {}  # dict: overhang_cluster -> contour vertices
        self.removalGeometries = []  # list of geometric solids to remove
        self.statistics = {}

    def extractOverhangPatches(self, overhangFaceIndices: np.ndarray = None,
                               overhangAngle: float = 45.0) -> Dict[int, List[int]]:
        """
        Extract and cluster overhanging surface patches.

        For each overhang face, cluster connected faces into patches.
        Returns a dictionary mapping cluster_id -> list of face indices.

        Args:
            overhangFaceIndices: Pre-computed overhang face indices (optional)
            overhangAngle: Angle threshold for detection (degrees)

        Returns:
            overhangs: Dict mapping cluster_id -> list of face indices
        """
        # Detect overhangs if not provided
        if overhangFaceIndices is None:
            detector = OptimizedOverhangDetector(self.mesh, buildDirection=self.buildDirection)
            overhangFaceIndices, _ = detector.detectByFaceNormal(overhangAngle)

        if len(overhangFaceIndices) == 0:
            print("[INFO] No overhangs detected")
            return {}

        # Cluster connected overhang faces
        overhangs = {}
        visited = set()
        clusterId = 0

        for startFaceIdx in overhangFaceIndices:
            if startFaceIdx in visited:
                continue

            # BFS to find connected component
            cluster = []
            queue = [int(startFaceIdx)]

            while queue:
                faceIdx = queue.pop(0)
                if faceIdx in visited:
                    continue

                visited.add(faceIdx)
                cluster.append(faceIdx)

                # Find adjacent overhang faces
                try:
                    if hasattr(self.mesh, 'face_adjacency') and len(self.mesh.face_adjacency) > 0:
                        adjacentMask = (self.mesh.face_adjacency[:, 0] == faceIdx) | \
                                       (self.mesh.face_adjacency[:, 1] == faceIdx)
                        adjacentFaces = np.concatenate(self.mesh.face_adjacency[adjacentMask])

                        for adjFace in adjacentFaces:
                            if adjFace not in visited and adjFace in overhangFaceIndices:
                                queue.append(int(adjFace))
                except Exception:
                    pass

            if len(cluster) > 0:
                overhangs[clusterId] = cluster
                clusterId += 1

        self.overhangFaces = overhangs
        print(f"[INFO] Extracted {len(overhangs)} overhang patches")
        for cid, patch in overhangs.items():
            print(f"       Patch {cid}: {len(patch)} faces")

        return overhangs

    def computeEnvelopeContour(self, overhangFaceIndices: List[int]) -> np.ndarray:
        """
        For each vertex of an overhang patch, determine its horizontal reference plane.
        This plane intersects the inner surface, forming a closed envelope contour.

        Args:
            overhangFaceIndices: List of face indices in the overhang patch

        Returns:
            contourVertices: Array of vertices forming the envelope contour
        """
        # Get all vertices of overhang faces
        overhangVertexIndices = np.unique(self.mesh.faces[overhangFaceIndices].flatten())
        overhangVertices = self.mesh.vertices[overhangVertexIndices]

        topSurfaceZ = self.topSurfaceZ
        if topSurfaceZ is None:
            topSurfaceZ = np.max(self.mesh.vertices[:, 2])

        # For each vertex, find the horizontal plane at its Z coordinate
        # and compute intersection with the inner surface
        envelopePoints = []

        for vertexIdx in overhangVertexIndices:
            vertex = self.mesh.vertices[vertexIdx]
            planeZ = vertex[2]

            # Find all mesh vertices at approximately this height
            # (horizontal plane at this Z)
            heightTolerance = 0.1  # 0.1mm tolerance
            verticesOnPlane = self.mesh.vertices[
                np.abs(self.mesh.vertices[:, 2] - planeZ) < heightTolerance
            ]

            if len(verticesOnPlane) > 0:
                envelopePoints.append(vertex[:2])  # Use (X, Y) coordinates

        if len(envelopePoints) == 0:
            print(f"[WARNING] No envelope contour found for patch")
            return np.array([])

        envelopePoints = np.array(envelopePoints)

        # Form a convex or concave hull to represent the envelope
        # Use ConvexHull as a simple approximation
        try:
            if len(envelopePoints) >= 3:
                hull = ConvexHull(envelopePoints)
                contourVertices = envelopePoints[hull.vertices]
            else:
                contourVertices = envelopePoints
        except:
            # If convex hull fails, use the points as-is
            contourVertices = envelopePoints

        return contourVertices

    def calculateEnvelopeDimensions(self, contourVertices: np.ndarray) -> Tuple[float, float]:
        """
        Calculate characteristic dimensions of the envelope contour.

        scaleEnvelope: Half of the maximum distance between any two points on the contour
        heightEnvelope: Vertical distance between the contour plane and the top surface

        Returns:
            (scaleEnvelope, heightEnvelope)
        """
        if len(contourVertices) < 2:
            return 0.0, 0.0

        n = contourVertices.shape[0]
        if n < 3:
            raise ValueError("contourVertices must have at least 3 vertices")

        # Compute all pairwise distances
        diff = contourVertices[:, None, :] - contourVertices[None, :, :]  # (N, N, 2)
        dist2 = np.einsum('ijk,ijk->ij', diff, diff)  # squared distances
        # Take upper triangle only to avoid duplicate pairs and zero diagonal
        iUpper, jUpper = np.triu_indices(n, k=1)
        maxIdx = np.argmax(dist2[iUpper, jUpper])
        iFar = iUpper[maxIdx]
        jFar = jUpper[maxIdx]

        # Half of the farthest distance is scaleEnvelope
        scaleEnvelope = 0.5 * np.sqrt(dist2[iFar, jFar])

        # Height_envelope: vertical distance from contour plane to top surface
        topSurfaceZ = self.topSurfaceZ
        if topSurfaceZ is None:
            topSurfaceZ = np.max(self.mesh.vertices[:, 2])

        contourZ = np.mean(self.mesh.vertices[
            np.linalg.norm(self.mesh.vertices[:, :2] - contourVertices.mean(axis=0), axis=1) < scaleEnvelope
        ][:, 2]) if len(contourVertices) > 0 else 0.0

        heightEnvelope = topSurfaceZ - contourZ

        return scaleEnvelope, heightEnvelope

    def generateCone45(self,
                       baseContour: np.ndarray,
                       baseZ: float) -> trimesh.Trimesh:
        """
        Generate a cone mesh from a base contour, using the scaleEnvelope
        returned by calculateEnvelopeDimensions to avoid functional coupling.

        Args:
            baseContour: Base contour vertices (N x 2 in XY plane).
            baseZ: Z-coordinate of the base polygon.

        Returns:
            trimesh.Trimesh: Cone mesh.
        """
        n = baseContour.shape[0]
        if n < 3:
            raise ValueError("baseContour must have at least 3 vertices")

        # Get scaleEnvelope from the dedicated function
        scaleEnvelope, _ = self.calculateEnvelopeDimensions(baseContour)

        # If scaleEnvelope is non-positive, abort
        if scaleEnvelope <= 0.0:
            raise ValueError("scaleEnvelope computed from contour is non-positive")

        # Recompute the farthest pair for midpoint location
        diff = baseContour[:, None, :] - baseContour[None, :, :]  # (N, N, 2)
        dist2 = np.einsum('ijk,ijk->ij', diff, diff)  # squared distances
        iUpper, jUpper = np.triu_indices(n, k=1)
        maxIdx = np.argmax(dist2[iUpper, jUpper])
        iFar = iUpper[maxIdx]
        jFar = jUpper[maxIdx]

        p1 = baseContour[iFar]
        p2 = baseContour[jFar]

        # Midpoint of farthest pair
        midpoint = 0.5 * (p1 + p2)

        # Apex point P (same xy as midpoint, z increased by scaleEnvelope)
        apex = np.array([midpoint[0], midpoint[1], baseZ + scaleEnvelope])

        # Base vertices in 3D
        baseVertices3d = np.column_stack(
            [baseContour[:, 0],
             baseContour[:, 1],
             np.full(n, baseZ, dtype=float)]
        )

        # All vertices: base first, apex last
        vertices = np.vstack([baseVertices3d, apex[None, :]])
        apexIndex = n  # last vertex

        # Triangulate the base polygon as a fan around vertex 0
        baseFaces = []
        for i in range(1, n - 1):
            baseFaces.append([0, i, i + 1])

        # Side faces: connect apex with each edge of the base polygon
        sideFaces = []
        for i in range(n):
            j = (i + 1) % n
            sideFaces.append([apexIndex, i, j])

        faces = np.array(baseFaces + sideFaces, dtype=np.int64)

        coneMesh = trimesh.Trimesh(vertices=vertices, faces=faces, process=False)
        return coneMesh

    def generateOffsetCurve(self, contour: np.ndarray, offsetDistance: float,
                            inward: bool = True) -> np.ndarray:
        """
        Generate an inward or outward offset curve from a given contour.

        Uses the polygon offset algorithm (Clipper-like approach).

        Args:
            contour: Input contour vertices (Nx2 in XY plane)
            offsetDistance: Distance to offset
            inward: If True, offset inward; if False, offset outward

        Returns:
            offsetContour: Offset contour vertices (Nx2)
        """
        if len(contour) < 3:
            return contour.copy()

        # Simple offset algorithm using normal vectors
        n = len(contour)
        offsetContour = []

        for i in range(n):
            pPrev = contour[(i - 1) % n]
            pCurr = contour[i]
            pNext = contour[(i + 1) % n]

            # Compute normals (perpendicular to edges, pointing inward)
            edge1 = pCurr - pPrev
            edge2 = pNext - pCurr

            # Normalize edges
            edge1Len = np.linalg.norm(edge1)
            edge2Len = np.linalg.norm(edge2)

            if edge1Len == 0 or edge2Len == 0:
                offsetContour.append(pCurr)
                continue

            edge1 = edge1 / edge1Len
            edge2 = edge2 / edge2Len

            # Normal perpendicular to edges (rotate 90°)
            normal1 = np.array([-edge1[1], edge1[0]])
            normal2 = np.array([-edge2[1], edge2[0]])

            # Bisector normal
            bisector = (normal1 + normal2) / 2.0
            bisectorLen = np.linalg.norm(bisector)

            if bisectorLen > 0:
                bisector = bisector / bisectorLen
            else:
                bisector = normal1

            # Apply offset
            direction = -1.0 if inward else 1.0
            offsetPoint = pCurr + direction * offsetDistance * bisector
            offsetContour.append(offsetPoint)

        return np.array(offsetContour)

    def generateCurvedTopBody(self,
                              baseContour: np.ndarray,
                              baseZ: float) -> trimesh.Trimesh:
        """
        Generate a curved contour on the top mold surface and connect it
        to the base contour to form a 3D body, using the heightEnvelope
        returned by calculateEnvelopeDimensions.

        Args:
            baseContour: Base contour vertices (N x 2 in XY plane).
            baseZ: Z coordinate of the base contour.

        Returns:
            trimesh.Trimesh: Mesh of the generated geometric body.
        """
        n = baseContour.shape[0]
        if n < 3:
            raise ValueError("baseContour must have at least 3 vertices")

        # Get heightEnvelope from the dedicated function
        _, heightEnvelope = self.calculateEnvelopeDimensions(baseContour)
        if heightEnvelope <= 0.0:
            raise ValueError("heightEnvelope computed from contour is non-positive")

        # Step 1: inward offset in XY plane
        offset2d = self.generateOffsetCurve(
            contour=baseContour,
            offsetDistance=heightEnvelope,
            inward=True
        )

        if offset2d.shape[0] != n:
            raise ValueError("Offset contour must have same number of vertices as baseContour")

        # Step 2: lift offset curve to top surface (z = baseZ + heightEnvelope)
        topZ = baseZ + heightEnvelope
        baseVertices3d = np.column_stack(
            [baseContour[:, 0],
             baseContour[:, 1],
             np.full(n, baseZ, dtype=float)]
        )
        topVertices3d = np.column_stack(
            [offset2d[:, 0],
             offset2d[:, 1],
             np.full(n, topZ, dtype=float)]
        )

        # Stack vertices: first base ring, then top ring
        vertices = np.vstack([baseVertices3d, topVertices3d])
        baseOffset = 0
        topOffset = n

        faces = []

        # 1) Triangulate bottom polygon (fan around vertex 0 in base ring)
        for i in range(1, n - 1):
            faces.append([baseOffset + 0,
                          baseOffset + i,
                          baseOffset + i + 1])

        # 2) Triangulate top polygon (fan around vertex 0 in top ring)
        #    Reverse winding so normals point outward (up).
        for i in range(1, n - 1):
            faces.append([topOffset + 0,
                          topOffset + i + 1,
                          topOffset + i])

        # 3) Side faces between corresponding vertices of base and top
        for i in range(n):
            iNext = (i + 1) % n

            b0 = baseOffset + i
            b1 = baseOffset + iNext
            t0 = topOffset + i
            t1 = topOffset + iNext

            faces.append([b0, b1, t1])
            faces.append([b0, t1, t0])

        faces = np.array(faces, dtype=np.int64)

        mesh = trimesh.Trimesh(vertices=vertices, faces=faces, process=False)
        return mesh

    def removeOverhangGeometries(self) -> trimesh.Trimesh:
        """
        For each extracted overhang patch, build a removal geometry
        (either a 45° cone or a curved-top body) and subtract it from
        the input mesh using boolean difference.

        Strategy:
            - For each overhang cluster (patch):
                1) Compute envelope contour in XY.
                2) Compute ScaleEnvelope and HeightEnvelope.
                3) If ScaleEnvelope < HeightEnvelope:
                       Use generateCone45() on this contour.
                   Else:
                       Use generateCurvedTopBody() on this contour.
                4) Perform boolean difference: mesh = mesh - removalBody.
            - Accumulate basic statistics per cluster.
        """
        # If no patches were extracted, just return original mesh
        if not self.overhangFaces:
            print("[INFO] No overhang patches to process. Returning original mesh.")
            return self.mesh.copy()

        # Start from a copy of the original mesh
        resultMesh = self.mesh.copy()

        self.statistics = {}
        clusterProcessed = 0

        for clusterId, faceIndices in self.overhangFaces.items():
            print(f"\n[CLUSTER] Processing overhang patch {clusterId} "
                  f"with {len(faceIndices)} faces")

            # 1) Compute envelope contour in XY plane
            contourXY = self.computeEnvelopeContour(faceIndices)
            if contourXY is None or len(contourXY) < 3:
                print("  [Skip] Invalid or empty contour, skipping this patch.")
                continue

            # 2) Compute characteristic dimensions
            scaleEnvelope, heightEnvelope = self.calculateEnvelopeDimensions(contourXY)
            if scaleEnvelope <= 0.0 or heightEnvelope <= 0.0:
                print("  [Skip] Non-positive ScaleEnvelope/HeightEnvelope, skipping.")
                continue

            # Estimate contourZ as in calculateEnvelopeDimensions
            centerXY = contourXY.mean(axis=0)
            mask = np.linalg.norm(self.mesh.vertices[:, :2] - centerXY,
                                  axis=1) < scaleEnvelope
            if not np.any(mask):
                contourZ = float(np.min(self.mesh.vertices[:, 2]))
            else:
                contourZ = float(np.mean(self.mesh.vertices[mask, 2]))

            # 3) Decide which removal geometry to use
            try:
                if scaleEnvelope < heightEnvelope:
                    print("  [Strategy] Using 45° cone removal geometry "
                          f"(Scale={scaleEnvelope:.3f}, Height={heightEnvelope:.3f})")
                    removalBody = self.generateCone45(
                        baseContour=contourXY,
                        baseZ=contourZ
                    )
                    geomType = "cone_45deg"
                else:
                    print("  [Strategy] Using curved-top body removal geometry "
                          f"(Scale={scaleEnvelope:.3f}, Height={heightEnvelope:.3f})")
                    removalBody = self.generateCurvedTopBody(
                        baseContour=contourXY,
                        baseZ=contourZ,
                    )
                    geomType = "curved_top"
            except Exception as e:
                print(f"  [Error] Failed to create removal geometry for patch "
                      f"{clusterId}: {e}")
                continue

            if removalBody is None or len(removalBody.vertices) == 0:
                print("  [Skip] Removal geometry is empty, skipping.")
                continue

            print("  [Boolean] Subtracting removal geometry from mesh...")
            success = False
            diffMesh = None

            engines = ['scad', 'blender', 'manifold']
            for engine in engines:
                try:
                    diffMesh = resultMesh.difference(removalBody, engine=engine)
                    if (diffMesh is not None and
                            hasattr(diffMesh, 'faces') and
                            len(diffMesh.faces) > 0):
                        success = True
                        print(f"    [OK] Boolean difference succeeded with engine='{engine}'.")
                        break
                except Exception:
                    continue

            if not success:
                print("    [WARN] Boolean difference failed with all engines. "
                      "Applying AABB fallback clipping.")
                try:
                    rbMin, rbMax = removalBody.bounds
                    keepMask = ~np.all(
                        (resultMesh.vertices >= rbMin) &
                        (resultMesh.vertices <= rbMax),
                        axis=1
                    )
                    if np.any(keepMask):
                        faceKeep = keepMask[resultMesh.faces].all(axis=1)
                        if np.any(faceKeep):
                            diffMesh = trimesh.Trimesh(
                                vertices=resultMesh.vertices[keepMask],
                                faces=resultMesh.faces[faceKeep],
                                process=False
                            )
                            success = True
                            print("    [OK] AABB fallback clipping applied.")
                except Exception as e:
                    print(f"    [Error] AABB fallback also failed: {e}")
                    diffMesh = None

            if not success or diffMesh is None or len(diffMesh.faces) == 0:
                print("  [Skip] Removal for this patch failed; keeping previous mesh.")
                continue

            resultMesh = diffMesh

            self.statistics[clusterId] = {
                'geomType': geomType,
                'scaleEnvelope': float(scaleEnvelope),
                'heightEnvelope': float(heightEnvelope),
                'contourXY': int(len(contourXY)),
                'contourZ': float(contourZ),
                'removalBodyVertices': int(len(removalBody.vertices)),
                'removalBodyFaces': int(len(removalBody.faces))
            }
            clusterProcessed += 1
            print(f"  [Done] Patch {clusterId} processed successfully.")

        print(f"\n[INFO] Overhang removal completed: {clusterProcessed} / "
              f"{len(self.overhangFaces)} patches processed.")

        return resultMesh

    def execute(self, overhangAngle: float = 45.0) -> Tuple[trimesh.Trimesh, Dict]:
        """
        Execute complete overhang removal pipeline.

        Args:
            overhangAngle: Angle threshold for overhang detection

        Returns:
            resultMesh: Processed mesh
            metrics: Dictionary with removal statistics
        """
        print("[START] Overhang Structure Removal Module")
        startTime = time.time()

        self.extractOverhangPatches(overhangAngle=overhangAngle)

        result = self.removeOverhangGeometries()

        elapsedTime = time.time() - startTime

        metrics = {
            'extraction_time': elapsedTime,
            'clusters_processed': len(self.statistics),
            'statistics': self.statistics
        }

        print(f"[DONE] Elapsed time: {elapsedTime*1000:.1f} ms\n")

        return result, metrics

def removeOverhangStructures(mesh: trimesh.Trimesh,
                             overhangAngle: float = 45.0,
                             buildDirection: np.ndarray | None = None) -> Tuple[trimesh.Trimesh, Dict]:
    """
    High-level interface for overhang removal.

    Args:
        mesh: Input mold mesh as a trimesh.Trimesh object.
        overhangAngle: Overhang detection threshold in degrees.
        buildDirection: Optional build direction vector; if None, use [0, 0, -1].

    Returns:
        resultMesh: Mesh after removing auxiliary overhang-removal geometries.
        metrics: Dictionary with timing and per-cluster statistics.
    """
    remover = OverhangStructureRemoval(mesh=mesh, buildDirection=buildDirection)
    resultMesh, metrics = remover.execute(overhangAngle=overhangAngle)
    return resultMesh, metrics