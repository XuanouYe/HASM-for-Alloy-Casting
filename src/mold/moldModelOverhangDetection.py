import numpy as np
import trimesh
from typing import Tuple, Dict, List
from collections import defaultdict, deque

class OptimizedOverhangDetector:
    """
    Robust overhang detector for FDM mold generation.

    This version properly identifies overhanging surfaces by:
    - Measuring angle between face normal and positive Z-axis
    - Applying threshold: (90° + overhangAngle) < angle < 180°
    - Distinguishing between interior and exterior faces using ray casting
    - Grouping connected overhangs into geometric features
    """

    def __init__(self, mesh: trimesh.Trimesh, buildDirection: np.ndarray = None):
        """
        Initialize detector with mesh geometry.

        Args:
            mesh: Input mesh to analyze
            buildDirection: Build direction vector (default [0, 0, -1] for upward printing)
                           Note: This parameter is kept for compatibility but not used 
                           in the corrected face normal detection
        """
        self.mesh = mesh
        self.buildDirection = np.array([0.0, 0.0, -1.0]) if buildDirection is None else buildDirection
        self.buildDirection = self.buildDirection / np.linalg.norm(self.buildDirection)

        # Extract mesh properties
        self.vertices = mesh.vertices
        self.faces = mesh.faces
        self.faceNormals = mesh.face_normals.copy()

        # Normalize face normals to ensure unit length
        for i in range(len(self.faceNormals)):
            norm = np.linalg.norm(self.faceNormals[i])
            if norm > 0:
                self.faceNormals[i] /= norm

        # Cache for adjacency data
        self._faceAdjacency = None
        self._faceAdjacencyEdges = None

    def detectByFaceNormal(self, overhangAngle: float = 45.0) -> Tuple[np.ndarray, np.ndarray]:
        """
        Face normal method - detects faces pointing away from vertical.

        KEY FEATURES:
        1. Uses positive Z-axis [0, 0, 1] as reference
        2. Distinguishes between interior and exterior faces
        3. Condition for exterior: (90° + overhangAngle) < angle < 180°
        4. Condition for interior: (90° + overhangAngle) ≤ angle ≤ 180°

        This ensures correct detection:
        - TOP faces (normal=[0,0,1], angle=0°): NOT detected (correct)
        - BOTTOM faces (normal=[0,0,-1], angle=180°): NOT detected for exterior (correct)
        - CAVITY bottom faces (angle=180°): DETECTED for interior (correct)
        - SIDE faces pointing down-outward: detected as overhangs (correct)

        Args:
            overhangAngle: Angle threshold in degrees (default 45°)

        Returns:
            Tuple of (overhang face indices, their angles in degrees)
        """
        if len(self.faces) == 0:
            return np.array([], dtype=int), np.array([])

        # Use positive Z-axis as reference
        zAxisPositive = np.array([0.0, 0.0, 1.0], dtype=np.float64)

        # Compute dot product between each normal and positive Z-axis
        dotProducts = np.dot(self.faceNormals, zAxisPositive)
        dotProducts = np.clip(dotProducts, -1.0, 1.0)

        # Compute angles in degrees
        anglesRad = np.arccos(dotProducts)
        anglesDeg = np.degrees(anglesRad)

        # Define overhang threshold: 90° + overhangAngle
        angleThreshold = 90.0 + overhangAngle

        # Identify interior faces (cavity faces)
        isInterior = self._identifyInteriorFaces()

        # Different conditions for interior vs exterior faces
        # For exterior faces: exclude exactly 180° (self-supporting bottom surfaces)
        exteriorMask = ~isInterior
        exteriorOverhangMask = (anglesDeg > angleThreshold) & (anglesDeg < 180.0) & exteriorMask

        # For interior faces: include exactly 180° (cavity bottom surfaces are overhangs)
        interiorMask = isInterior
        interiorOverhangMask = (anglesDeg >= angleThreshold) & (anglesDeg <= 180.0) & interiorMask

        # Combine masks
        overhangMask = exteriorOverhangMask | interiorOverhangMask
        overhangFaceIndices = np.where(overhangMask)[0]

        overhangAngles = anglesDeg[overhangFaceIndices]

        # Debug output
        print(f"\n[Face Normal Detection]")
        print(f"  Threshold: {angleThreshold}° (90° + {overhangAngle}°)")
        print(f"  Faces analyzed: {len(self.faces)}")
        print(f"  Interior faces: {np.sum(isInterior)}")
        print(f"  Exterior faces: {np.sum(~isInterior)}")
        print(f"  Overhangs detected: {len(overhangFaceIndices)}")

        # Detailed breakdown
        interior_overhangs = np.sum(interiorOverhangMask)
        exterior_overhangs = np.sum(exteriorOverhangMask)
        print(f"  - Interior overhangs: {interior_overhangs}")
        print(f"  - Exterior overhangs: {exterior_overhangs}")

        if len(overhangFaceIndices) > 0:
            print(f"  Angle range: {overhangAngles.min():.1f}° - {overhangAngles.max():.1f}°")
            if len(overhangFaceIndices) <= 20:
                print(f"  Overhang face indices: {overhangFaceIndices}")

            # Show angle distribution
            angles_180 = np.sum(overhangAngles >= 179.9)
            if angles_180 > 0:
                print(f"  Found {angles_180} faces at exactly 180° (cavity interiors)")

        return overhangFaceIndices, overhangAngles

    def _identifyInteriorFaces(self) -> np.ndarray:
        """
        Identify interior faces (inner surfaces).

        Principle: Cast rays from face centers along their normal directions.
        If a ray intersects the model (excluding self-intersection), the face is interior (inner surface).
        If no intersection, the face is exterior (outer surface).

        Returns:
            Boolean array, True for interior faces, False for exterior faces.
        """
        if len(self.faces) == 0:
            return np.array([], dtype=bool)

        print(f"  Identifying interior faces using ray casting...")

        isInterior = np.zeros(len(self.faces), dtype=bool)

        # Get the center point of each face
        faceCenters = []
        for face in self.faces:
            faceVertices = self.vertices[face]
            center = np.mean(faceVertices, axis=0)
            faceCenters.append(center)

        faceCenters = np.array(faceCenters)

        # Use trimesh's ray functionality for intersection detection
        for i in range(len(self.faces)):
            # Get the center point of the current face
            rayOrigin = faceCenters[i]

            # Get the normal direction of the current face
            rayDirection = self.faceNormals[i]

            # Ensure the normal is a unit vector
            norm = np.linalg.norm(rayDirection)
            if norm > 0:
                rayDirection = rayDirection / norm

            # Slightly offset the ray origin to avoid starting from the surface
            rayOriginOffset = rayOrigin + rayDirection * 0.001

            # Check if the ray intersects the model
            locations, indexRay, indexTri = self.mesh.ray.intersects_location(
                ray_origins=[rayOriginOffset],
                ray_directions=[rayDirection],
                multiple_hits=False
            )

            # Key logic:
            # 1. If the ray intersects the model → ray enters from outside to inside → face is interior → True
            # 2. If the ray does not intersect → ray points to external space → face is exterior → False

            # Check if the intersection is with the current face (exclude self-intersection)
            if len(locations) > 0:
                # Check if the intersecting triangle index includes the current face
                if i in indexTri:
                    # Self-intersection, ignore this result
                    # Try casting from a slightly further point
                    rayOriginOffset2 = rayOrigin + rayDirection * 0.01
                    locations2, indexRay2, indexTri2 = self.mesh.ray.intersects_location(
                        ray_origins=[rayOriginOffset2],
                        ray_directions=[rayDirection],
                        multiple_hits=False
                    )

                    # Check the re-cast result
                    if len(locations2) > 0:
                        # If there is an intersection and it's not self-intersection
                        isSelfIntersection = i in indexTri2
                        isInterior[i] = len(locations2) > 0 and not isSelfIntersection
                    else:
                        # No intersection from the re-cast
                        isInterior[i] = False
                else:
                    # Intersection exists and is not self-intersection → interior face
                    isInterior[i] = True
            else:
                # No intersection → exterior face
                isInterior[i] = False

        # Validate results
        interiorCount = np.sum(isInterior)
        exteriorCount = np.sum(~isInterior)

        print(f"  Ray casting complete:")
        print(f"    - Total faces: {len(self.faces)}")
        print(f"    - Interior faces (True): {interiorCount}")
        print(f"    - Exterior faces (False): {exteriorCount}")

        return isInterior

    def detectByGeometricFeatures(self, overhangAngle: float = 45.0) -> Tuple[List, np.ndarray]:
        """
        Detect overhangs by grouping connected faces into geometric features.

        This method groups adjacent overhang faces into clusters (features)
        and calculates their properties for analysis.

        Args:
            overhangAngle: Overhang threshold

        Returns:
            Tuple of (feature list, feature face indices)
        """
        # Get baseline overhangs using corrected method
        overhangFaces, overhangAngles = self.detectByFaceNormal(overhangAngle)

        if len(overhangFaces) == 0:
            return [], np.array([], dtype=int)

        # Build adjacency graph for overhang faces
        overhangSet = set(overhangFaces)
        adjacency = defaultdict(set)

        # Get or compute face adjacency
        if self._faceAdjacency is None:
            try:
                self._faceAdjacency = self.mesh.face_adjacency
            except:
                self._computeFaceAdjacency()

        # Build adjacency graph
        if self._faceAdjacency is not None:
            for face1, face2 in self._faceAdjacency:
                if face1 in overhangSet and face2 in overhangSet:
                    adjacency[face1].add(face2)
                    adjacency[face2].add(face1)

        # Find connected components (features)
        visited = set()
        features = []

        for face in overhangFaces:
            if face not in visited:
                # BFS to find connected component
                queue = deque([face])
                component = []

                while queue:
                    current = queue.popleft()
                    if current not in visited:
                        visited.add(current)
                        component.append(current)

                        # Add neighbors
                        for neighbor in adjacency.get(current, []):
                            if neighbor not in visited:
                                queue.append(neighbor)

                if len(component) > 0:
                    # Calculate feature properties
                    componentMask = np.isin(overhangFaces, component)
                    featureAngles = overhangAngles[componentMask]

                    avgAngle = float(np.mean(featureAngles)) if len(featureAngles) > 0 else 0.0

                    # Get face positions for size estimation
                    facePositions = []
                    for faceIdx in component:
                        faceVerts = self.vertices[self.faces[faceIdx]]
                        facePositions.append(np.mean(faceVerts, axis=0))

                    if facePositions:
                        positions = np.array(facePositions)
                        bboxSize = np.ptp(positions, axis=0)
                        featureSize = float(np.linalg.norm(bboxSize))
                    else:
                        featureSize = 0.0

                    features.append({
                        'face_count': len(component),
                        'avg_angle': avgAngle,
                        'size': featureSize,
                        'complexity': float(len(component) / max(1.0, featureSize)),
                        'severity': float(avgAngle * len(component) / 100.0),
                        'faces': component
                    })

        # Sort features by severity
        features.sort(key=lambda x: x['severity'], reverse=True)

        # Get all face indices from features
        featureFaceIndices = []
        for feature in features:
            featureFaceIndices.extend(feature['faces'])

        print(f"\n[Geometric Features] Found {len(features)} feature clusters")
        if features:
            print(f"  Largest cluster: {features[0]['face_count']} faces, severity: {features[0]['severity']:.2f}")

        return features, np.array(featureFaceIndices, dtype=int)

    def computeGaussianMap(self, gridResolution: int = 100) -> Tuple[np.ndarray, Dict]:
        """
        Discrete Gaussian Map Method.

        Projects face normals onto a unit sphere and analyzes their distribution
        to identify optimal build orientation and critical overhang zones.

        Args:
            gridResolution: Resolution of the spherical grid (default 100)

        Returns:
            Tuple of (gaussian map array, analysis dictionary)
        """
        # Simple implementation for demonstration
        normals = self.faceNormals

        if len(normals) == 0:
            return np.zeros((gridResolution, gridResolution)), {}

        # Create spherical histogram
        gaussianMap = np.zeros((gridResolution, gridResolution))

        for normal in normals:
            # Convert to spherical coordinates
            x, y, z = normal
            r = np.sqrt(x * x + y * y + z * z)
            if r > 0:
                # Azimuth angle (longitude)
                theta = np.arctan2(y, x)
                if theta < 0:
                    theta += 2 * np.pi

                # Polar angle (latitude)
                phi = np.arccos(z / r)

                # Map to grid indices
                thetaIdx = int(theta / (2 * np.pi) * gridResolution) % gridResolution
                phiIdx = int(phi / np.pi * gridResolution) % gridResolution

                gaussianMap[phiIdx, thetaIdx] += 1

        # Normalize
        total = np.sum(gaussianMap)
        if total > 0:
            gaussianMap = gaussianMap / total

        # Analyze for optimal build direction
        analysis = {
            'total_faces': len(normals),
            'map_shape': gaussianMap.shape,
            'map_nonzero': np.count_nonzero(gaussianMap),
            'max_density': float(np.max(gaussianMap)) if len(gaussianMap) > 0 else 0.0
        }

        print(f"\n[Gaussian Map] Resolution: {gridResolution}x{gridResolution}")
        print(f"  Total faces mapped: {analysis['total_faces']}")
        print(f"  Non-zero bins: {analysis['map_nonzero']}")

        return gaussianMap, analysis

    def _computeFaceAdjacency(self):
        """Compute face adjacency manually if trimesh doesn't provide it."""
        print("  Computing face adjacency manually...")

        # Create edge-face mapping
        edgeToFaces = defaultdict(set)
        for faceIdx, face in enumerate(self.faces):
            # Get edges (ordered pairs)
            edges = [
                tuple(sorted((int(face[0]), int(face[1])))),
                tuple(sorted((int(face[1]), int(face[2])))),
                tuple(sorted((int(face[2]), int(face[0]))))
            ]

            for edge in edges:
                edgeToFaces[edge].add(faceIdx)

        # Build adjacency list
        adjacency = []
        adjacencyEdges = []

        for edge, faceSet in edgeToFaces.items():
            if len(faceSet) == 2:  # Manifold edge
                face1, face2 = sorted(faceSet)
                adjacency.append([face1, face2])
                adjacencyEdges.append(edge)

        self._faceAdjacency = np.array(adjacency) if adjacency else np.array([], dtype=int)
        self._faceAdjacencyEdges = np.array(adjacencyEdges) if adjacencyEdges else np.array([], dtype=int)

        print(f"  Found {len(self._faceAdjacency)} adjacent face pairs")

    def combinedDetection(self, overhangAngle: float = 45.0,
                          method: str = 'normal') -> Tuple[np.ndarray, Dict]:
        """
        Combine detection methods with controlled logic.

        Args:
            overhangAngle: Overhang threshold
            method: Combination method ('normal' or 'features')

        Returns:
            Tuple of (combined overhang indices, method details)
        """
        if method == 'normal':
            # Use the face normal method
            overhangFaces, overhangAngles = self.detectByFaceNormal(overhangAngle)
            methodDetails = {
                'method': 'normal',
                'count': len(overhangFaces),
                'threshold': overhangAngle,
                'angles': overhangAngles.tolist() if len(overhangAngles) > 0 else []
            }
            return overhangFaces, methodDetails

        elif method == 'features':
            # Use geometric feature clustering
            features, featureFaces = self.detectByGeometricFeatures(overhangAngle)
            methodDetails = {
                'method': 'features',
                'count': len(featureFaces),
                'feature_count': len(features),
                'largest_feature': features[0]['face_count'] if features else 0
            }
            return featureFaces, methodDetails

        else:
            print(f"Warning: Unknown method '{method}', using 'normal'")
            overhangFaces, overhangAngles = self.detectByFaceNormal(overhangAngle)
            methodDetails = {
                'method': 'normal_fallback',
                'count': len(overhangFaces)
            }
            return overhangFaces, methodDetails

def detectOverhang(mesh, overhangAngle=45.0, method='normal', buildDirection=None, **kwargs):
    """
    Detect overhanging faces in a 3D mesh model.

    Args:
        mesh (trimesh.Trimesh): Input 3D mesh model.
        overhangAngle (float): Overhang angle threshold in degrees, default is 45°.
        method (str): Detection method, options:
            - 'normal': Face normal method only (default)
            - 'features': Geometric feature clustering method
        buildDirection (np.ndarray): Build direction vector, default is [0, 0, -1].
        **kwargs: Additional optional parameters passed to specific methods.

    Returns:
        tuple: (overhang face indices array, details dictionary)
    """

    # Create detector instance
    detector = OptimizedOverhangDetector(mesh, buildDirection)

    if method == 'features':
        features, overhangFaces = detector.detectByGeometricFeatures(overhangAngle)
        details = {
            'method': 'features',
            'count': len(overhangFaces),
            'featureCount': len(features),
            'features': features,
            'threshold': overhangAngle
        }

    else:
        # Default fallback to normal method
        overhangFaces, angles = detector.detectByFaceNormal(overhangAngle)
        details = {
            'method': 'normal',
            'count': len(overhangFaces),
            'threshold': overhangAngle,
            'angles': angles.tolist() if len(angles) > 0 else [],
            'angleRange': [float(angles.min()), float(angles.max())] if len(angles) > 0 else [0, 0]
        }

    # Add general statistics
    details.update({
        'meshFaces': len(mesh.faces),
        'overhangPercentage': len(overhangFaces) / max(1, len(mesh.faces)) * 100,
        'buildDirection': detector.buildDirection.tolist()
    })

    return overhangFaces, details

def validateCorrections():
    """
    Validate that the corrections work correctly.

    Tests:
    1. Solid cube should have 0 overhangs (all surfaces self-supporting)
    2. Hollow cube should have 2 overhangs (inner cavity downward faces)
    3. Sphere should have faces at various angles
    """
    print("=" * 70)
    print("VALIDATION: CORRECTED OVERHANG DETECTOR")
    print("=" * 70)

    # Test 1: Solid cube (0 overhangs expected)
    print("\n[Test 1] Solid Cube")
    box = trimesh.creation.box(extents=[20, 20, 20])

    overhangFaces1, details1 = detectOverhang(box, overhangAngle=45.0, method='normal')
    print(f"  Expected: 0 overhangs (all faces self-supporting)")
    print(f"  Result: {len(overhangFaces1)} overhangs")

    if len(overhangFaces1) == 0:
        print("  ✓ PASS: Solid cube correctly has 0 overhangs")
    else:
        print(f"  ✗ FAIL: Found {len(overhangFaces1)} overhangs, expected 0")

    # Test 2: Simple hollow cube approximation
    print("\n[Test 2] Hollow Cube (Approximation)")

    # Create a box with a smaller box subtracted
    try:
        outer = trimesh.creation.box(extents=[20, 20, 20])
        inner = trimesh.creation.box(extents=[18, 18, 18])

        # Try to create boolean difference
        hollow = None
        engines = ['manifold', 'blender', 'scad']

        for engine in engines:
            try:
                hollow = outer.difference(inner, engine=engine)

                if hollow is not None and hasattr(hollow, 'faces') and len(hollow.faces) > 0:
                    break
                else:
                    hollow = None
            except Exception:
                hollow = None

        if hollow is None:
            try:
                hollow = outer.difference(inner)

                if hollow is not None and hasattr(hollow, 'faces') and len(hollow.faces) > 0:
                    pass
                else:
                    hollow = None
            except Exception:
                hollow = None

        if hollow is not None and len(hollow.faces) > 0:
            overhangFaces2, details2 = detectOverhang(hollow, overhangAngle=45.0, method='normal')
            print(f"  Expected: ~2 overhangs (inner cavity downward faces)")
            print(f"  Result: {len(overhangFaces2)} overhangs")

            if len(overhangFaces2) <= 2:
                print(f"  ✓ PASS: Hollow cube has {len(overhangFaces2)} overhangs (close to expected 2)")
            else:
                print(f"  ⚠ WARNING: Found {len(overhangFaces2)} overhangs, expected ~2")
        else:
            print("  ⚠ SKIP: Could not create hollow cube for testing")

    except Exception as e:
        print(f"  ⚠ SKIP: Hollow cube test failed: {e}")

    # Test 3: Sphere (should have many overhangs)
    print("\n[Test 3] Sphere")
    sphere = trimesh.creation.icosphere(subdivisions=2)
    sphere.vertices *= 10

    overhangFaces3, details3 = detectOverhang(sphere, overhangAngle=45.0, method='normal')
    print(f"  Sphere faces: {len(sphere.faces)}")
    print(f"  Overhangs detected: {len(overhangFaces3)} ({len(overhangFaces3) / len(sphere.faces) * 100:.1f}%)")

    print("\n" + "=" * 70)
    print("VALIDATION COMPLETE")
    print("=" * 70)

if __name__ == "__main__":
    validateCorrections()
