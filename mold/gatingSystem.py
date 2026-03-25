from dataclasses import dataclass
from typing import List, Tuple, Optional, Dict, Union
import numpy as np
import trimesh
from pathlib import Path
from dataModel import GatingComponents

@dataclass
class MoldConfig:
    cavityVolume: float
    boundingBox: Tuple[np.ndarray, np.ndarray]
    centroid: np.ndarray
    lowestZ: float

@dataclass
class RheologyParams:
    meltViscosity: float = 2e-3
    solidificationFactor: float = 1.5
    maxPressureDrop: float = 1e5
    heatTransferCoeff: float = 500.0
    meltOverheat: float = 30.0

@dataclass
class GateCandidate:
    surfacePoint: np.ndarray
    surfaceNormal: np.ndarray
    thicknessScore: float
    xWidth: float
    yWidth: float
    zHeight: float

class AutoGatingSystem:
    def __init__(self, mesh: trimesh.Trimesh, config: Optional[Dict] = None):
        if isinstance(mesh, trimesh.Scene):
            mesh = mesh.dump(concatenate=True)
        self._originalMesh = mesh
        self.minRadius = 3.0
        self.velocityLimit = 500.0
        self.bufferDistance = 5.0
        config = config or {}
        self.targetFillTime = config.get('targetFillTime', 5.0)
        self.sprueInletOffset = config.get('sprueInletOffset', 5.0)
        self.boundingBoxOffset = config.get('boundingBoxOffset', 2.0)
        self.cavityMeshOverride = config.get('cavityMesh', None)

        rConf = config.get('rheology', {})
        self.rheologyParams = RheologyParams(
            meltViscosity=rConf.get('meltViscosity', 2e-3),
            solidificationFactor=rConf.get('solidificationFactor', 1.5),
            maxPressureDrop=rConf.get('maxPressureDrop', 1e5),
            heatTransferCoeff=rConf.get('heatTransferCoeff', 500.0),
            meltOverheat=rConf.get('meltOverheat', 30.0)
        )
        self.moldConfig = self._analyzeMold(self._originalMesh)

    def _analyzeMold(self, mesh: trimesh.Trimesh) -> MoldConfig:
        return MoldConfig(
            cavityVolume=abs(mesh.volume),
            boundingBox=(mesh.bounds[0], mesh.bounds[1]),
            centroid=np.array(mesh.centroid),
            lowestZ=float(mesh.bounds[0][2])
        )

    def _getCavityMesh(self) -> trimesh.Trimesh:
        if self.cavityMeshOverride is not None:
            cavMesh = self.cavityMeshOverride
            if isinstance(cavMesh, trimesh.Scene):
                cavMesh = cavMesh.dump(concatenate=True)
            return cavMesh
        return self._originalMesh

    def _computeTargetGateZ(self, runnerRadius: float) -> float:
        bboxMin, _ = self.moldConfig.boundingBox
        return float(bboxMin[2]) + runnerRadius

    def _getSectionOuterLoopPoints(self, cavityMesh: trimesh.Trimesh, targetZ: float, sampleStep: float) -> np.ndarray:
        planeOrigin = np.array([0, 0, targetZ])
        planeNormal = np.array([0, 0, 1])
        section = cavityMesh.section(plane_origin=planeOrigin, plane_normal=planeNormal)
        if section is None:
            return np.array([]).reshape(0, 3)
        path2D, to3D = section.to_2D()
        if path2D is None or len(path2D.entities) == 0:
            return np.array([]).reshape(0, 3)
        polygons = path2D.polygons_full
        if len(polygons) == 0:
            return np.array([]).reshape(0, 3)
        areas = [p.area for p in polygons]
        maxIdx = int(np.argmax(areas))
        outerPoly = polygons[maxIdx]
        perimeter = outerPoly.length
        numSamples = max(int(perimeter / sampleStep), 10)
        arcLengths = np.linspace(0, perimeter, numSamples, endpoint=False)
        sampledPoints2D = []
        for s in arcLengths:
            pt = outerPoly.exterior.interpolate(s)
            sampledPoints2D.append([pt.x, pt.y])
        if len(sampledPoints2D) == 0:
            return np.array([]).reshape(0, 3)
        sampledPoints2D = np.array(sampledPoints2D)
        sampledPoints2DPadded = np.column_stack([sampledPoints2D, np.zeros(len(sampledPoints2D))])
        return trimesh.transform_points(sampledPoints2DPadded, to3D)

    def _projectAndGetNormal(self, cavityMesh: trimesh.Trimesh, points: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        if len(points) == 0:
            return np.array([]).reshape(0, 3), np.array([]).reshape(0, 3)
        projPoints, _, faceIndices = cavityMesh.nearest.on_surface(points)
        normals = cavityMesh.face_normals[faceIndices]
        return projPoints, normals

    def _filterSideCandidates(self, points: np.ndarray, normals: np.ndarray, targetZ: float, zTol: float,
                              normalZLimit: float, outerAlignLimit: float, cavityMesh: trimesh.Trimesh) -> Tuple[
        np.ndarray, np.ndarray]:
        if len(points) == 0:
            return np.array([]).reshape(0, 3), np.array([]).reshape(0, 3)
        bboxCenter = (cavityMesh.bounds[0] + cavityMesh.bounds[1]) / 2.0
        centerOfSlice = np.array([bboxCenter[0], bboxCenter[1], targetZ])
        zMask = np.abs(points[:, 2] - targetZ) <= zTol
        normalZMask = np.abs(normals[:, 2]) <= normalZLimit
        outwardVectors = points - centerOfSlice
        outwardVectorsNorm = outwardVectors / (np.linalg.norm(outwardVectors, axis=1, keepdims=True) + 1e-12)
        normalsNorm = normals / (np.linalg.norm(normals, axis=1, keepdims=True) + 1e-12)
        alignment = np.sum(normalsNorm * outwardVectorsNorm, axis=1)
        outerMask = alignment > outerAlignLimit
        finalMask = zMask & normalZMask & outerMask
        return points[finalMask], normals[finalMask]

    def _computeAxisExtentsByRays(self, cavityMesh: trimesh.Trimesh, pIn: np.ndarray) -> Tuple[
        float, float, float, float]:
        rayIntersector = trimesh.ray.ray_triangle.RayMeshIntersector(cavityMesh)
        directions = np.array([[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1]])
        rayOrigins = np.tile(pIn, (6, 1))
        locations, indexRay, _ = rayIntersector.intersects_location(rayOrigins, directions, multiple_hits=False)
        dists = np.full(6, -1.0)
        for i in range(len(indexRay)):
            dists[indexRay[i]] = np.linalg.norm(locations[i] - pIn)
        xWidth = dists[0] + dists[1] if dists[0] >= 0 and dists[1] >= 0 else -1.0
        yWidth = dists[2] + dists[3] if dists[2] >= 0 and dists[3] >= 0 else -1.0
        zHeight = dists[4] + dists[5] if dists[4] >= 0 and dists[5] >= 0 else -1.0

        diagDirs = np.array([[1, 1, 0], [-1, -1, 0], [1, -1, 0], [-1, 1, 0]], dtype=float)
        diagDirsNorm = diagDirs / np.linalg.norm(diagDirs, axis=1, keepdims=True)
        diagOrigins = np.tile(pIn, (4, 1))
        diagLocs, diagIdxRay, _ = rayIntersector.intersects_location(diagOrigins, diagDirsNorm, multiple_hits=False)
        diagDists = np.full(4, -1.0)
        for i in range(len(diagIdxRay)):
            diagDists[diagIdxRay[i]] = np.linalg.norm(diagLocs[i] - pIn)
        diagWidth1 = diagDists[0] + diagDists[1] if diagDists[0] >= 0 and diagDists[1] >= 0 else -1.0
        diagWidth2 = diagDists[2] + diagDists[3] if diagDists[2] >= 0 and diagDists[3] >= 0 else -1.0
        diagMin = min(diagWidth1, diagWidth2)
        return xWidth, yWidth, zHeight, diagMin

    def _scoreCandidateThickness(self, xWidth: float, yWidth: float, zHeight: float, diagMin: float,
                                 runnerRadius: float, minThicknessFactor: float = 1.0, diagFactor: float = 1.0) -> \
            Tuple[bool, float]:
        if xWidth < 0 or yWidth < 0 or zHeight < 0:
            return False, 0.0
        minDim = min(xWidth, yWidth, zHeight)
        if minDim < minThicknessFactor * 2.0 * runnerRadius:
            return False, 0.0
        if 0 < diagMin < diagFactor * 2.0 * runnerRadius:
            return False, 0.0
        bulkScore = (xWidth * yWidth * zHeight) ** (1.0 / 3.0)
        maxDim = max(xWidth, yWidth, zHeight)
        slenderPenalty = minDim / (maxDim + 1e-9)
        return True, bulkScore * (0.5 + 0.5 * slenderPenalty)

    def _selectBestGateCandidate(self, candidates: List[GateCandidate]) -> Optional[GateCandidate]:
        if len(candidates) == 0:
            return None
        candidates.sort(key=lambda c: c.thicknessScore, reverse=True)
        return candidates[0]

    def _proposeGateLocation(self) -> Tuple[np.ndarray, np.ndarray]:
        cavityMesh = self._getCavityMesh()
        bboxMin, bboxMax = self.moldConfig.boundingBox
        zMin, zMax = float(bboxMin[2]), float(bboxMax[2])
        runnerRadius = self._computeRheologyRadius()
        modelScale = float(np.max(cavityMesh.bounding_box.extents))
        targetZ = self._computeTargetGateZ(runnerRadius)
        sampleStep = max(runnerRadius * 0.5, modelScale * 0.005)
        zTol = modelScale * 0.02
        maxZShift = zMin + 0.5 * (zMax - zMin)
        dzStep = runnerRadius * 0.5
        epsIn = max(modelScale * 1e-4, 0.05)

        searchConfigs = [
            (1.5, 1.0),
            (0.8, 0.5)
        ]

        for minThick, diagFact in searchConfigs:
            candidates = []
            currentZ = targetZ
            while currentZ <= maxZShift:
                sectionPoints = self._getSectionOuterLoopPoints(cavityMesh, currentZ, sampleStep)
                if len(sectionPoints) == 0:
                    currentZ += dzStep
                    continue
                projPoints, normals = self._projectAndGetNormal(cavityMesh, sectionPoints)
                filteredPoints, filteredNormals = self._filterSideCandidates(projPoints, normals, currentZ, zTol, 0.3,
                                                                             0.3, cavityMesh)

                for idx in range(len(filteredPoints)):
                    pSurf = filteredPoints[idx]
                    nSurf = filteredNormals[idx]
                    pIn = pSurf - nSurf * epsIn
                    xWidth, yWidth, zHeight, diagMin = self._computeAxisExtentsByRays(cavityMesh, pIn)
                    isValid, score = self._scoreCandidateThickness(xWidth, yWidth, zHeight, diagMin, runnerRadius,
                                                                   minThicknessFactor=minThick, diagFactor=diagFact)
                    if isValid:
                        candidates.append(
                            GateCandidate(surfacePoint=pSurf, surfaceNormal=nSurf, thicknessScore=score, xWidth=xWidth,
                                          yWidth=yWidth, zHeight=zHeight))

                if len(candidates) > 0:
                    break
                currentZ += dzStep

            bestCandidate = self._selectBestGateCandidate(candidates)
            if bestCandidate is not None:
                return bestCandidate.surfacePoint, bestCandidate.surfaceNormal

        pts = self._getSectionOuterLoopPoints(cavityMesh, targetZ, sampleStep)
        if len(pts) > 0:
            projPt, _, faceIdx = cavityMesh.nearest.on_surface([pts[0]])
            return projPt[0], cavityMesh.face_normals[faceIdx[0]]
        return np.array([bboxMax[0], (bboxMin[1] + bboxMax[1]) / 2.0, targetZ]), np.array([1.0, 0.0, 0.0])

    def _calculateRunnerRadius(self) -> float:
        cavityVolumeM3 = self.moldConfig.cavityVolume * 1e-9
        flowRate = cavityVolumeM3 / max(self.targetFillTime, 0.1)
        minArea = flowRate / (self.velocityLimit * 1e-3)
        radius = np.sqrt(minArea / np.pi) * 1000.0
        bboxMin, bboxMax = self.moldConfig.boundingBox
        maxAllowed = float(np.min(bboxMax - bboxMin)) * 0.25
        return float(np.clip(max(radius, self.minRadius), self.minRadius, maxAllowed))

    def _computeRheologyRadius(self, runnerLengthMm: float = None) -> float:
        if runnerLengthMm is None:
            bboxMin, bboxMax = self.moldConfig.boundingBox
            runnerLengthMm = float(np.max(bboxMax - bboxMin)) * 0.6
        runnerLengthM = runnerLengthMm * 1e-3
        rp = self.rheologyParams
        cavityVolumeM3 = self.moldConfig.cavityVolume * 1e-9
        flowRate = cavityVolumeM3 / max(self.targetFillTime, 0.1)
        effectiveViscosity = rp.meltViscosity * rp.solidificationFactor
        numerator = 8.0 * effectiveViscosity * runnerLengthM * flowRate
        denominator = np.pi * rp.maxPressureDrop
        rheologyRadiusMm = (
            (numerator / denominator) ** 0.25) * 1000.0 if denominator > 0 and numerator > 0 else self.minRadius
        thermalFactor = 1.0 + 0.1 * (rp.heatTransferCoeff / 500.0)
        rheologyRadiusMm *= thermalFactor
        velocityRadius = self._calculateRunnerRadius()
        finalRadius = max(rheologyRadiusMm, velocityRadius, self.minRadius)
        bboxMin, bboxMax = self.moldConfig.boundingBox
        maxAllowed = float(np.min(bboxMax - bboxMin)) * 0.25
        return float(min(finalRadius, maxAllowed))

    def _generateRunnerPath(self, gateSurface: np.ndarray, gateNormal: np.ndarray, runnerZ: float, runnerRadius: float,
                            sprueInletZ: Optional[float] = None) -> List[np.ndarray]:
        gateDir2D = np.array([gateNormal[0], gateNormal[1]])
        gateDir2DNorm = np.linalg.norm(gateDir2D)
        if gateDir2DNorm < 1e-9:
            gateDir2D = gateSurface[:2] - self.moldConfig.centroid[:2]
            gateDir2DNorm = max(np.linalg.norm(gateDir2D), 1e-9)
        gateDir2D = gateDir2D / gateDir2DNorm
        outsideOffset = runnerRadius + self.bufferDistance
        bboxMin, bboxMax = self.moldConfig.boundingBox
        bboxExpand = runnerRadius + self.bufferDistance
        while True:
            runnerOutside = gateSurface + np.array([gateDir2D[0], gateDir2D[1], 0.0]) * outsideOffset
            if (runnerOutside[0] < bboxMin[0] - bboxExpand or runnerOutside[0] > bboxMax[0] + bboxExpand or
                    runnerOutside[1] < bboxMin[1] - bboxExpand or runnerOutside[1] > bboxMax[1] + bboxExpand):
                break
            outsideOffset += self.bufferDistance
        sprueZ = float(sprueInletZ) if sprueInletZ is not None else (float(bboxMax[2]) + self.boundingBoxOffset)
        sprueInlet = np.array([runnerOutside[0], runnerOutside[1], sprueZ])
        sprueBottom = np.array([runnerOutside[0], runnerOutside[1], runnerZ])
        adjGateSurface = np.array([gateSurface[0], gateSurface[1], runnerZ])
        return [sprueInlet, sprueBottom, adjGateSurface]

    def _createGatingMesh(self, path: List[np.ndarray], radius: float) -> trimesh.Trimesh:
        meshes = []
        for i in range(len(path) - 1):
            p1, p2 = path[i], path[i + 1]
            segmentLength = np.linalg.norm(p2 - p1)
            if segmentLength < 1e-6:
                continue
            meshes.append(trimesh.creation.cylinder(radius=radius, segment=[p1, p2], sections=32))
        for i in range(1, len(path) - 1):
            sph = trimesh.creation.icosphere(radius=radius, subdivisions=3)
            sph.apply_translation(path[i])
            meshes.append(sph)
        if not meshes:
            return trimesh.Trimesh()
        combined = trimesh.boolean.union(meshes)
        combined.fix_normals()
        return combined

    def _proposeRiserAttachmentPoint(self, cavityMesh: trimesh.Trimesh, gateSurface: np.ndarray) -> np.ndarray:
        bboxMin, bboxMax = cavityMesh.bounds[0], cavityMesh.bounds[1]
        zMax = float(bboxMax[2])
        modelScale = float(np.max(cavityMesh.bounding_box.extents))
        zTol = modelScale * 0.002
        vNodes = cavityMesh.vertices[cavityMesh.faces]
        topMask = np.all(np.abs(vNodes[:, :, 2] - zMax) <= zTol, axis=1)
        topFaceIndices = np.where(topMask)[0]
        if topFaceIndices.size == 0:
            centers = cavityMesh.triangles.mean(axis=1)
            topFaceIndices = np.where((centers[:, 2] >= zMax - zTol) & (centers[:, 2] <= zMax))[0]
        if topFaceIndices.size == 0:
            bestTopPoint = np.array([(bboxMin[0] + bboxMax[0]) / 2.0, (bboxMin[1] + bboxMax[1]) / 2.0, zMax])
            projPoints, _, _ = cavityMesh.nearest.on_surface([bestTopPoint])
            return projPoints[0]
        topArea = float(cavityMesh.area_faces[topFaceIndices].sum())
        bboxArea2D = (float(bboxMax[0]) - float(bboxMin[0])) * (float(bboxMax[1]) - float(bboxMin[1]))
        centersTop = cavityMesh.triangles[topFaceIndices].mean(axis=1)
        areasTop = cavityMesh.area_faces[topFaceIndices]
        if bboxArea2D > 1e-12 and topArea / bboxArea2D >= 0.05:
            nCandidates = min(200, len(centersTop))
            indices = np.linspace(0, len(centersTop) - 1, nCandidates, dtype=int)
            candidates = centersTop[indices]
            dist2D = np.hypot(candidates[:, 0] - gateSurface[0], candidates[:, 1] - gateSurface[1])
            bestTopPoint = candidates[int(np.argmax(dist2D))]
        else:
            areaSum = areasTop.sum()
            bestTopPoint = np.average(centersTop, axis=0, weights=areasTop) if areaSum > 1e-12 else centersTop.mean(
                axis=0)
        projPoints, _, _ = cavityMesh.nearest.on_surface([bestTopPoint])
        return projPoints[0]

    def _computeRiserDimensions(self, runnerRadius: float, attachmentZ: float, sprueInletZ: float, modelScale: float,
                                bboxExtents: np.ndarray) -> Dict:
        riserTopZ = sprueInletZ
        neckLength = 2.0 * runnerRadius
        neckDiameter = 2.0 * runnerRadius
        riserDiameter = float(np.clip(4.0 * runnerRadius, 2.5 * runnerRadius, float(np.min(bboxExtents)) * 0.4))
        minRiserTopZ = attachmentZ + neckLength + runnerRadius
        if riserTopZ <= minRiserTopZ:
            riserTopZ = max(minRiserTopZ, sprueInletZ)
        return {'riserTopZ': riserTopZ, 'neckLength': neckLength, 'neckDiameter': neckDiameter,
                'riserDiameter': riserDiameter}

    def _createRiserMesh(self, attachmentPoint: np.ndarray, riserTopZ: float, riserDiameter: float, neckDiameter: float,
                         neckLength: float, runnerRadius: float) -> trimesh.Trimesh:
        x, y = float(attachmentPoint[0]), float(attachmentPoint[1])
        neckStartZ = float(attachmentPoint[2]) - 0.2 * runnerRadius
        neckEndZ = float(attachmentPoint[2]) + neckLength
        neckCylinder = trimesh.creation.cylinder(radius=neckDiameter / 2.0,
                                                 segment=[(x, y, neckStartZ), (x, y, neckEndZ)], sections=32)
        riserCylinder = trimesh.creation.cylinder(radius=riserDiameter / 2.0,
                                                  segment=[(x, y, neckEndZ), (x, y, riserTopZ)], sections=32)
        combined = trimesh.boolean.union([neckCylinder, riserCylinder])
        combined.fix_normals()
        return combined

    def generateComponents(self) -> GatingComponents:
        gateSurface, gateNormal = self._proposeGateLocation()
        runnerRadius = self._computeRheologyRadius()
        runnerZ = self._computeTargetGateZ(runnerRadius)
        cavityMesh = self._getCavityMesh()
        bboxExtents = np.array(cavityMesh.bounding_box.extents)
        bboxMin, bboxMax = self.moldConfig.boundingBox
        sprueInletZ = float(bboxMax[2]) + self.boundingBoxOffset
        attachmentPoint = self._proposeRiserAttachmentPoint(cavityMesh, gateSurface)
        dims = self._computeRiserDimensions(
            runnerRadius, float(attachmentPoint[2]), sprueInletZ,
            float(np.max(bboxExtents)), bboxExtents)
        riserMesh = self._createRiserMesh(
            attachmentPoint, dims['riserTopZ'], dims['riserDiameter'],
            dims['neckDiameter'], dims['neckLength'], runnerRadius)
        runnerPath = self._generateRunnerPath(gateSurface, gateNormal, runnerZ, runnerRadius,
                                              sprueInletZ=dims['riserTopZ'])
        gateMesh = self._createGatingMesh(runnerPath, runnerRadius)
        systemMesh = trimesh.boolean.union([riserMesh, gateMesh])
        systemMesh.fix_normals()
        castingWithRiserMesh = trimesh.boolean.union([self._originalMesh, riserMesh])
        castingWithRiserMesh.fix_normals()
        castingWithSystemMesh = trimesh.boolean.union([self._originalMesh, systemMesh])
        castingWithSystemMesh.fix_normals()
        return GatingComponents(
            gateMesh=gateMesh,
            riserMesh=riserMesh,
            systemMesh=systemMesh,
            castingWithSystemMesh=castingWithSystemMesh,
            gateSurface=gateSurface,
            runnerPath=runnerPath,
            runnerRadius=runnerRadius
        )

def createGatingSystem(
        castingMesh: trimesh.Trimesh,
        config: Optional[Dict] = None,
        componentStlDir: Optional[Union[str, Path]] = None
) -> GatingComponents:
    if config is None:
        config = {}
    gatingSystem = AutoGatingSystem(castingMesh, config)
    gatingComponents = gatingSystem.generateComponents()
    if componentStlDir is not None:
        from geometryAdapters import exportMeshToStl
        componentStlDir = Path(componentStlDir)
        componentStlDir.mkdir(parents=True, exist_ok=True)
        exportMeshToStl(gatingComponents.gateMesh, str(componentStlDir / "gate.stl"))
        exportMeshToStl(gatingComponents.riserMesh, str(componentStlDir / "riser.stl"))
        exportMeshToStl(gatingComponents.systemMesh, str(componentStlDir / "system.stl"))
        exportMeshToStl(gatingComponents.castingWithSystemMesh, str(componentStlDir / "casting.with.system.stl"))
    return gatingComponents
