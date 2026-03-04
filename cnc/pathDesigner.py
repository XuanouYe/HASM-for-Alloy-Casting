import json
from typing import List, Dict, Optional, Any
import numpy as np
import trimesh
import vtk
from scipy.spatial.distance import cdist
from scipy.spatial import cKDTree
from pathlib import Path
from shapely.geometry import LineString, MultiPolygon
from shapely.ops import unary_union
from shapely.validation import make_valid
from abc import ABC, abstractmethod


def normalizeVector(vec: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(vec))
    if norm > 0.0:
        return vec / norm
    return vec


def buildRotationFromTo(fromVec: np.ndarray, toVec: np.ndarray) -> np.ndarray:
    fromVec = normalizeVector(fromVec)
    toVec = normalizeVector(toVec)
    v = np.cross(fromVec, toVec)
    c = float(np.dot(fromVec, toVec))
    s = float(np.linalg.norm(v))
    if s == 0.0:
        if c > 0.0:
            return np.eye(3)
        axis = np.array([1.0, 0.0, 0.0])
        if abs(fromVec[0]) > 0.9:
            axis = np.array([0.0, 1.0, 0.0])
        v = normalizeVector(np.cross(fromVec, axis))
        vx = np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])
        return np.eye(3) + 2.0 * (vx @ vx)
    vx = np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])
    r = np.eye(3) + vx + (vx @ vx) * ((1.0 - c) / (s * s))
    return r


def applyRotation(points: np.ndarray, rotMat: np.ndarray) -> np.ndarray:
    if len(points) == 0:
        return points
    return (rotMat @ points.T).T


def generateHemisphereAxes(numAxes: int) -> List[List[float]]:
    axes = []
    phi = np.pi * (3.0 - np.sqrt(5.0))
    for i in range(numAxes):
        z = 1.0 - (i / float(numAxes - 1)) if numAxes > 1 else 1.0
        if z < 0.0:
            break
        radius = np.sqrt(1.0 - z * z)
        theta = phi * i
        x = np.cos(theta) * radius
        y = np.sin(theta) * radius
        axes.append([float(x), float(y), float(z)])
    return axes


class PointCloudIPW:
    def __init__(self, mesh: trimesh.Trimesh, sample_count: int = 50000):
        try:
            self.points, _ = trimesh.sample.sample_surface(mesh, sample_count)
        except:
            self.points = np.zeros((0, 3))
        self.active_mask = np.ones(len(self.points), dtype=bool)

    def get_active_points_wcs(self) -> np.ndarray:
        return self.points[self.active_mask]

    def filter_paths_local(self, paths_local: List[np.ndarray], rotToToolFrame: np.ndarray, tool_radius: float,
                           step_over: float) -> List[np.ndarray]:
        if not paths_local or not np.any(self.active_mask):
            return []
        active_pts_wcs = self.get_active_points_wcs()
        if len(active_pts_wcs) == 0:
            return []
        active_pts_local = applyRotation(active_pts_wcs, rotToToolFrame)
        tree = cKDTree(active_pts_local)
        valid_paths = []
        search_radius = tool_radius + step_over * 1.5
        for path in paths_local:
            dists, _ = tree.query(path, k=1, distance_upper_bound=search_radius)
            if np.any(dists != np.inf):
                valid_paths.append(path)
        return valid_paths

    def update_ipw_local(self, paths_local: List[np.ndarray], rotToToolFrame: np.ndarray, tool_radius: float):
        if not paths_local or not np.any(self.active_mask):
            return
        active_pts_wcs = self.get_active_points_wcs()
        active_indices = np.where(self.active_mask)[0]
        active_pts_local = applyRotation(active_pts_wcs, rotToToolFrame)
        tree = cKDTree(active_pts_local[:, :2])
        remove_local_idx = set()
        for path in paths_local:
            pts_idx = tree.query_ball_point(path[:, :2], r=tool_radius)
            for i, idx_list in enumerate(pts_idx):
                path_z = path[i, 2]
                for idx in idx_list:
                    if active_pts_local[idx, 2] >= path_z - tool_radius * 0.5:
                        remove_local_idx.add(idx)
        if remove_local_idx:
            global_remove_idx = active_indices[list(remove_local_idx)]
            self.active_mask[global_remove_idx] = False


class IToolpathStrategy(ABC):
    @abstractmethod
    def generate(self, targetMesh: trimesh.Trimesh, keepOutMesh: trimesh.Trimesh, toolRadius: float,
                 params: Dict[str, Any], safetyMargin: float) -> List[np.ndarray]:
        pass

    def cleanPolygon(self, polys: List[Any]) -> Any:
        cleaned = []
        for p in polys:
            if not p.is_valid:
                p = make_valid(p)
            p = p.buffer(0)
            if not p.is_empty:
                cleaned.append(p)
        return unary_union(cleaned)

    def robustSection(self, mesh: trimesh.Trimesh, z: float, tolerance: float = 0.05) -> Any:
        sliceResult = mesh.section(plane_origin=[0, 0, z], plane_normal=[0, 0, 1])
        if sliceResult is None:
            sliceResult = mesh.section(plane_origin=[0, 0, z - tolerance], plane_normal=[0, 0, 1])
        if sliceResult is None:
            sliceResult = mesh.section(plane_origin=[0, 0, z + tolerance], plane_normal=[0, 0, 1])
        if sliceResult is not None:
            slice2D, _ = sliceResult.to_2D()
            return slice2D.polygons_full
        return []


class ZLevelRoughingStrategy(IToolpathStrategy):
    def generate(self, targetMesh: trimesh.Trimesh, keepOutMesh: trimesh.Trimesh, toolRadius: float,
                 params: Dict[str, Any], safetyMargin: float) -> List[np.ndarray]:
        if targetMesh.is_empty:
            return []
        stepOver = float(params.get("stepOver", 1.0))
        layerStep = float(params.get("layerStep", 1.0))
        bounds = np.asarray(targetMesh.bounds, dtype=float)
        zMin, zMax = bounds[0][2], bounds[1][2]
        zLevels = np.arange(zMax - layerStep, zMin, -layerStep, dtype=float)
        paths = []

        for z in zLevels:
            targetPolys = self.robustSection(targetMesh, z)
            if not targetPolys:
                continue

            targetSliceResult = targetMesh.section(plane_origin=[0, 0, z], plane_normal=[0, 0, 1])
            if not targetSliceResult:
                continue
            _, to3DMatrix = targetSliceResult.to_2D()

            machinablePolyUnion = self.cleanPolygon(targetPolys)
            machinablePoly = machinablePolyUnion.buffer(-toolRadius)

            keepOutPoly = MultiPolygon()
            if not keepOutMesh.is_empty:
                keepOutPolys = self.robustSection(keepOutMesh, z)
                if keepOutPolys:
                    koPolyUnion = self.cleanPolygon(keepOutPolys)
                    keepOutPoly = koPolyUnion.buffer(toolRadius + safetyMargin + stepOver)

            if not machinablePoly.is_empty:
                safePoly = machinablePoly.difference(keepOutPoly).buffer(0)
                if safePoly.is_empty:
                    continue
                geoms = safePoly.geoms if safePoly.geom_type == 'MultiPolygon' else [safePoly]
                layerPaths2D = []
                for p in geoms:
                    if p.is_empty:
                        continue
                    minx, miny, maxx, maxy = p.bounds
                    yList = np.arange(miny, maxy, stepOver)
                    direction = 1
                    for y in yList:
                        scanLine = LineString([(minx - 1.0, y), (maxx + 1.0, y)])
                        intersection = scanLine.intersection(p)
                        if not intersection.is_empty:
                            lines = intersection.geoms if intersection.geom_type == 'MultiLineString' else [
                                intersection]
                            for line in lines:
                                if line.geom_type == 'LineString':
                                    coords = list(line.coords)
                                    if direction < 0:
                                        coords.reverse()
                                    layerPaths2D.append(coords)
                        direction *= -1
                for path2D in layerPaths2D:
                    if len(path2D) >= 2:
                        coordsHomo = np.column_stack((np.array(path2D), np.zeros(len(path2D)), np.ones(len(path2D))))
                        coords3D = (to3DMatrix @ coordsHomo.T).T[:, :3]
                        paths.append(coords3D)
        return paths


class WaterlineStrategy(IToolpathStrategy):
    def generate(self, targetMesh: trimesh.Trimesh, keepOutMesh: trimesh.Trimesh, toolRadius: float,
                 params: Dict[str, Any], safetyMargin: float) -> List[np.ndarray]:
        if targetMesh.is_empty:
            return []
        layerStep = float(params.get("layerStep", 0.5))
        bounds = np.asarray(targetMesh.bounds, dtype=float)
        zLevels = np.arange(bounds[0][2], bounds[1][2] + layerStep, layerStep, dtype=float)
        waterlines = []
        for z in zLevels:
            sliceResult = targetMesh.section(plane_origin=[0, 0, z], plane_normal=[0, 0, 1])
            if sliceResult:
                slice2D, to3DMatrix = sliceResult.to_2D()
                unionPoly = self.cleanPolygon(slice2D.polygons_full)
                offsetPoly = unionPoly.buffer(toolRadius)
                if not offsetPoly.is_empty:
                    geoms = offsetPoly.geoms if offsetPoly.geom_type == 'MultiPolygon' else [offsetPoly]
                    for p in geoms:
                        coords = np.array(p.exterior.coords)
                        coordsHomo = np.column_stack((coords, np.zeros(len(coords)), np.ones(len(coords))))
                        coords3D = (to3DMatrix @ coordsHomo.T).T[:, :3]
                        waterlines.append(coords3D)
        return waterlines


class DropRasterStrategy(IToolpathStrategy):
    def generate(self, targetMesh: trimesh.Trimesh, keepOutMesh: trimesh.Trimesh, toolRadius: float,
                 params: Dict[str, Any], safetyMargin: float) -> List[np.ndarray]:
        combinedMesh = trimesh.util.concatenate([targetMesh, keepOutMesh]) if not keepOutMesh.is_empty else targetMesh
        if combinedMesh.is_empty:
            return []
        bounds = np.asarray(targetMesh.bounds, dtype=float)
        stepOver = float(params.get("stepOver", 1.0))
        safeHeight = float(params.get("safeHeight", 5.0))
        angleThreshold = float(params.get("angleThreshold", 1.047))
        minZNormal = float(np.cos(angleThreshold))
        xMin, yMin, _ = bounds[0]
        xMax, yMax, zMax = bounds[1]
        xList = np.arange(xMin, xMax + stepOver, stepOver, dtype=float)
        yList = np.arange(yMin, yMax + stepOver, stepOver, dtype=float)
        zStart = float(zMax + safeHeight)
        paths = []
        for y in yList:
            rayOrigins = np.column_stack((xList, np.full(len(xList), y), np.full(len(xList), zStart)))
            rayDirections = np.tile([0, 0, -1], (len(xList), 1))
            locations, indexRay, indexTri = combinedMesh.ray.intersects_location(ray_origins=rayOrigins,
                                                                                 ray_directions=rayDirections)
            linePts = []
            for i in range(len(xList)):
                matchIndices = np.where(indexRay == i)[0]
                if len(matchIndices) > 0:
                    highestZIdx = matchIndices[np.argmax(locations[matchIndices, 2])]
                    triIdx = indexTri[highestZIdx]
                    normal = combinedMesh.face_normals[triIdx]
                    if normal[2] >= minZNormal:
                        linePts.append([xList[i], y, locations[highestZIdx, 2]])
            if linePts:
                paths.append(np.asarray(linePts, dtype=float))
        return paths


class ToolpathStrategyFactory:
    _strategies = {
        "zlevelroughing": ZLevelRoughingStrategy(),
        "zlr": ZLevelRoughingStrategy(),
        "waterline": WaterlineStrategy(),
        "wl": WaterlineStrategy(),
        "dropraster": DropRasterStrategy()
    }

    @classmethod
    def get_strategy(cls, mode: str) -> IToolpathStrategy:
        return cls._strategies.get(mode.lower(), DropRasterStrategy())


class TrimeshToolpathEngine:
    def optimizePathLinking(self, paths: List[np.ndarray], safeZ: float, stepOver: float) -> np.ndarray:
        if not paths:
            return np.array([])
        linkedPaths = []
        unvisited = list(range(len(paths)))
        currentPos = paths[0][0]
        while unvisited:
            starts = np.array([paths[i][0] for i in unvisited])
            ends = np.array([paths[i][-1] for i in unvisited])
            distStarts = cdist([currentPos], starts)[0]
            distEnds = cdist([currentPos], ends)[0]
            minStartIdx = int(np.argmin(distStarts))
            minEndIdx = int(np.argmin(distEnds))
            if distStarts[minStartIdx] <= distEnds[minEndIdx]:
                chosenListIdx, reverse, minDist = minStartIdx, False, distStarts[minStartIdx]
            else:
                chosenListIdx, reverse, minDist = minEndIdx, True, distEnds[minEndIdx]

            chosenPathIdx = unvisited[chosenListIdx]
            nextPath = paths[chosenPathIdx] if not reverse else paths[chosenPathIdx][::-1]
            if len(linkedPaths) > 0:
                if minDist <= stepOver * 2.1 and abs(currentPos[2] - nextPath[0][2]) < 1e-3:
                    linkedPaths.append(np.array([currentPos, nextPath[0]], dtype=float))
                else:
                    linkedPaths.append(np.vstack((np.array([currentPos[0], currentPos[1], safeZ], dtype=float),
                                                  np.array([nextPath[0][0], nextPath[0][1], safeZ], dtype=float))))
            linkedPaths.append(nextPath)
            currentPos = nextPath[-1]
            unvisited.pop(chosenListIdx)
        return np.vstack(linkedPaths) if linkedPaths else np.array([])

    def slicePathByPlatformZ(self, pathLocal: np.ndarray, rotBack: np.ndarray, platformSafeZ: float) -> List[
        np.ndarray]:
        pathWcs = applyRotation(pathLocal, rotBack)
        platformMask = pathWcs[:, 2] >= platformSafeZ
        validLocalPaths = []
        currentSubPath = []
        for i, isValid in enumerate(platformMask.tolist()):
            if isValid:
                currentSubPath.append(pathLocal[i])
            else:
                if len(currentSubPath) > 0:
                    validLocalPaths.append(np.asarray(currentSubPath, dtype=float))
                    currentSubPath = []
        if len(currentSubPath) > 0:
            validLocalPaths.append(np.asarray(currentSubPath, dtype=float))
        return validLocalPaths


class FiveAxisCncPathGenerator:
    def __init__(self, version: str = "1.0"):
        self.version = str(version)
        self.toolpathEngine = TrimeshToolpathEngine()

    def loadMesh(self, stlPath: str) -> trimesh.Trimesh:
        mesh = trimesh.load_mesh(stlPath)
        if isinstance(mesh, trimesh.Scene):
            mesh = trimesh.util.concatenate([g for g in mesh.geometry.values()])
        return mesh

    def generateJob(self, partStl: str, moldStl: str, gateStl: str, riserStl: str, toolParams: Dict[str, Any],
                    stepParams: List[Dict[str, Any]], axisStrategyParams: Dict[str, Any], wcsId: str = "WCS0",
                    jobId: Optional[str] = None) -> Dict[str, Any]:
        partMesh = self.loadMesh(partStl)
        moldMesh = self.loadMesh(moldStl)
        gateMesh = self.loadMesh(gateStl)
        riserMesh = self.loadMesh(riserStl)
        globalMinZ = min([float(m.bounds[0][2]) for m in [partMesh, moldMesh, gateMesh, riserMesh]])
        candidateAxesRaw = axisStrategyParams.get("candidateAxes", [[0.0, 0.0, 1.0]])
        candidateAxes = [ax for ax in [normalizeVector(np.array(a, dtype=float)) for a in candidateAxesRaw] if
                         ax[2] >= 0.0] or [np.array([0.0, 0.0, 1.0])]
        safetyMargin = float(toolParams.get("safetyMargin", 0.5))

        keepOutMeshStep1 = trimesh.util.concatenate([partMesh, gateMesh, riserMesh])
        step1 = self.generateStep(1, "shellRemoval", moldMesh, keepOutMeshStep1, toolParams, stepParams[0],
                                  candidateAxes, globalMinZ, safetyMargin)

        targetMeshStep2 = trimesh.util.concatenate([partMesh, riserMesh])
        step2 = self.generateStep(2, "partAndRiserFinishing", targetMeshStep2, gateMesh, toolParams, stepParams[1],
                                  candidateAxes, globalMinZ, safetyMargin)

        step3 = self.generateStep(3, "gateRemoval", gateMesh, partMesh, toolParams, stepParams[2], candidateAxes,
                                  globalMinZ, safetyMargin)

        out = {"version": self.version, "wcsId": str(wcsId), "steps": [step1, step2, step3]}
        if jobId is not None:
            out["jobId"] = str(jobId)
        return out

    def generateStep(self, stepId: int, stepType: str, targetMesh: trimesh.Trimesh, keepOutMesh: trimesh.Trimesh,
                     toolParams: Dict[str, Any], stepParam: Dict[str, Any], candidateAxes: List[np.ndarray],
                     globalMinZ: float, safetyMargin: float) -> Dict[str, Any]:
        mode = str(stepParam.get("mode", "dropRaster"))
        feedrate = float(stepParam.get("feedrate", 500.0))
        toolRadius = float(toolParams.get("diameter", 6.0)) * 0.5
        platformSafeZ = float(globalMinZ + toolRadius + safetyMargin)
        stepOver = float(stepParam.get("stepOver", 1.0))

        strategy = ToolpathStrategyFactory.get_strategy(mode)
        ipw = PointCloudIPW(targetMesh, 50000) if mode.lower() in ["zlevelroughing", "zlr", "dropraster"] else None

        segments = []
        allClPoints = []
        pointId = 0

        for segmentId, toolAxis in enumerate(candidateAxes):
            rotToToolFrame = buildRotationFromTo(toolAxis, np.array([0.0, 0.0, 1.0], dtype=float))
            rotBack = rotToToolFrame.T
            rotatedTarget = self.rotateMesh(targetMesh, rotToToolFrame)
            rotatedKeepOut = self.rotateMesh(keepOutMesh, rotToToolFrame)
            localSafeZ = float(rotatedTarget.bounds[1][2]) + float(stepParam.get("safeHeight", 5.0))

            rawPathsLocal = strategy.generate(rotatedTarget, rotatedKeepOut, toolRadius, stepParam, safetyMargin)

            validPathsLocal = []
            for pathLocal in rawPathsLocal:
                if len(pathLocal) == 0: continue
                validPathsLocal.extend(self.toolpathEngine.slicePathByPlatformZ(pathLocal, rotBack, platformSafeZ))

            if ipw is not None and validPathsLocal:
                validPathsLocal = ipw.filter_paths_local(validPathsLocal, rotToToolFrame, toolRadius, stepOver)
                ipw.update_ipw_local(validPathsLocal, rotToToolFrame, toolRadius)

            if validPathsLocal:
                optimizedLocalPath = self.toolpathEngine.optimizePathLinking(validPathsLocal, localSafeZ, stepOver)
                if len(optimizedLocalPath) > 0:
                    finalWcsPath = applyRotation(optimizedLocalPath, rotBack)
                    clPointsWcs = self.buildClPointDicts(finalWcsPath, toolAxis, feedrate, segmentId, pointId)
                    pointId += len(clPointsWcs)
                    segments.append({"segmentId": int(segmentId),
                                     "toolAxis": [float(toolAxis[0]), float(toolAxis[1]), float(toolAxis[2])],
                                     "pointCount": int(len(clPointsWcs))})
                    allClPoints.extend(clPointsWcs)

        return {"stepId": int(stepId), "stepType": str(stepType), "toolParams": toolParams, "segments": segments,
                "clPoints": allClPoints}

    def rotateMesh(self, mesh: trimesh.Trimesh, rotMat: np.ndarray) -> trimesh.Trimesh:
        if mesh.is_empty:
            return mesh
        return trimesh.Trimesh(vertices=applyRotation(np.asarray(mesh.vertices, dtype=float), rotMat),
                               faces=np.asarray(mesh.faces, dtype=int), process=False)

    def buildClPointDicts(self, positions: np.ndarray, toolAxis: np.ndarray, feedrate: float, segmentId: int,
                          startPointId: int) -> List[Dict[str, Any]]:
        toolAxis = normalizeVector(toolAxis)
        return [{"pointId": int(startPointId + i), "position": [float(p[0]), float(p[1]), float(p[2])],
                 "toolAxis": [float(toolAxis[0]), float(toolAxis[1]), float(toolAxis[2])], "feedrate": float(feedrate),
                 "segmentId": int(segmentId)} for i, p in enumerate(positions)]

    def exportClJson(self, clData: Dict[str, Any], outputPath: str) -> None:
        with open(outputPath, 'w', encoding='utf-8') as f:
            json.dump(clData, f, ensure_ascii=False, indent=2)


class VtkInteractorStyle(vtk.vtkInteractorStyleTrackballCamera):
    def __init__(self, visualizer):
        self.AddObserver("KeyPressEvent", self.keyPressEvent)
        self.visualizer = visualizer

    def keyPressEvent(self, obj, event):
        key = self.GetInteractor().GetKeySym()
        if key in ["1", "2", "3"]:
            stepIdx = int(key) - 1
            for i, actor in enumerate(self.visualizer.stepActors):
                actor.SetVisibility(1 if i == stepIdx else 0)
            print(f"Showing Step {key} only.")
        elif key == "0":
            for actor in self.visualizer.stepActors:
                actor.SetVisibility(1)
            print("Showing all Steps.")
        elif key == "c" or key == "C":
            isCollisionVisible = self.visualizer.collisionActor.GetVisibility()
            self.visualizer.collisionActor.SetVisibility(1 - isCollisionVisible)
            print("Toggled Collision Path Visibility.")
        self.GetInteractor().GetRenderWindow().Render()


class PathVisualizer:
    def __init__(self):
        self.windowSize = (1024, 768)
        self.stepActors = []
        self.collisionActor = None

    def createVTKMeshActor(self, mesh: trimesh.Trimesh, color: tuple, opacity: float) -> vtk.vtkActor:
        vtkPoints = vtk.vtkPoints()
        for v in mesh.vertices: vtkPoints.InsertNextPoint(v)
        vtkCells = vtk.vtkCellArray()
        for f in mesh.faces:
            tri = vtk.vtkTriangle()
            tri.GetPointIds().SetId(0, f[0])
            tri.GetPointIds().SetId(1, f[1])
            tri.GetPointIds().SetId(2, f[2])
            vtkCells.InsertNextCell(tri)
        polyData = vtk.vtkPolyData()
        polyData.SetPoints(vtkPoints)
        polyData.SetPolys(vtkCells)
        normalsFilter = vtk.vtkPolyDataNormals()
        normalsFilter.SetInputData(polyData)
        normalsFilter.ComputePointNormalsOn()
        normalsFilter.Update()
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(normalsFilter.GetOutputPort())
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(color)
        actor.GetProperty().SetOpacity(opacity)
        return actor

    def buildCollisionTree(self, mesh: trimesh.Trimesh) -> vtk.vtkOBBTree:
        vtkPoints = vtk.vtkPoints()
        for v in mesh.vertices: vtkPoints.InsertNextPoint(v)
        vtkCells = vtk.vtkCellArray()
        for f in mesh.faces:
            tri = vtk.vtkTriangle()
            tri.GetPointIds().SetId(0, f[0])
            tri.GetPointIds().SetId(1, f[1])
            tri.GetPointIds().SetId(2, f[2])
            vtkCells.InsertNextCell(tri)
        polyData = vtk.vtkPolyData()
        polyData.SetPoints(vtkPoints)
        polyData.SetPolys(vtkCells)
        obbTree = vtk.vtkOBBTree()
        obbTree.SetDataSet(polyData)
        obbTree.BuildLocator()
        return obbTree

    def evaluateCollisions(self, points: List[List[float]], obbTree: vtk.vtkOBBTree):
        normalLines, collisionLines = [], []
        if not points:
            return normalLines, collisionLines

        intersectPoints = vtk.vtkPoints()
        for i in range(1, len(points)):
            p1, p2 = points[i - 1], points[i]
            if obbTree.IntersectWithLine(p1, p2, intersectPoints, None) != 0:
                collisionLines.append((p1, p2))
            else:
                normalLines.append((p1, p2))
        return normalLines, collisionLines

    def createLinesActor(self, lines: List[tuple], color: tuple, lineWidth: float) -> vtk.vtkActor:
        vtkPoints = vtk.vtkPoints()
        vtkLines = vtk.vtkCellArray()
        pointIdx = 0
        for p1, p2 in lines:
            vtkPoints.InsertNextPoint(p1)
            vtkPoints.InsertNextPoint(p2)
            line = vtk.vtkLine()
            line.GetPointIds().SetId(0, pointIdx)
            line.GetPointIds().SetId(1, pointIdx + 1)
            vtkLines.InsertNextCell(line)
            pointIdx += 2

        polyData = vtk.vtkPolyData()
        polyData.SetPoints(vtkPoints)
        polyData.SetLines(vtkLines)
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputData(polyData)
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(color)
        actor.GetProperty().SetLineWidth(lineWidth)
        return actor

    def visualize(self, targetMesh: trimesh.Trimesh, clData: Dict[str, Any]) -> None:
        renderer = vtk.vtkRenderer()
        renderer.SetBackground(1.0, 1.0, 1.0)
        renderer.AddActor(self.createVTKMeshActor(targetMesh, (0.7, 0.7, 0.7), 0.5))

        print("Building OBBTree for collision detection...")
        obbTree = self.buildCollisionTree(targetMesh)
        colors = [(1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)]
        allCollisionLines = []

        print("Processing paths for step visualization and collisions...")
        for stepIndex, step in enumerate(clData.get("steps", [])):
            pts = [p["position"] for p in step.get("clPoints", [])]
            normalLines, collLines = self.evaluateCollisions(pts, obbTree)
            if normalLines:
                stepActor = self.createLinesActor(normalLines, colors[stepIndex % len(colors)], 2.0)
                self.stepActors.append(stepActor)
                renderer.AddActor(stepActor)
            allCollisionLines.extend(collLines)

        if allCollisionLines:
            self.collisionActor = self.createLinesActor(allCollisionLines, (1.0, 0.0, 1.0), 4.0)
            self.collisionActor.SetVisibility(0)
            renderer.AddActor(self.collisionActor)
            print(f"Found {len(allCollisionLines)} path segments intersecting with the target mesh.")

        renderWindow = vtk.vtkRenderWindow()
        renderWindow.AddRenderer(renderer)
        renderWindow.SetSize(self.windowSize[0], self.windowSize[1])
        renderWindow.SetWindowName("CNC Toolpath Visualization (1-3: Steps, 0: All, C: Collision)")

        interactor = vtk.vtkRenderWindowInteractor()
        interactor.SetRenderWindow(renderWindow)
        style = VtkInteractorStyle(self)
        style.SetDefaultRenderer(renderer)
        interactor.SetInteractorStyle(style)

        axesWidget = vtk.vtkOrientationMarkerWidget()
        axesWidget.SetOrientationMarker(vtk.vtkAxesActor())
        axesWidget.SetInteractor(interactor)
        axesWidget.SetEnabled(1)
        axesWidget.InteractiveOff()

        renderer.ResetCamera()
        renderer.GetActiveCamera().Azimuth(30)
        renderer.GetActiveCamera().Elevation(30)
        interactor.Initialize()
        renderWindow.Render()
        print("Controls: Keys '1', '2', '3' filter steps. '0' shows all. 'C' toggles collisions.")
        interactor.Start()


def generateCncJobInterface(partStl: str, moldStl: str, gateStl: str, riserStl: str, outputJsonPath: str,
                            processConfig: Dict[str, Any], jobId: str = "JOB_AUTO", visualize: bool = False) -> Dict[
    str, Any]:
    subtractiveConfig = processConfig.get("subtractive", {})
    toolParams = {"type": "ball", "diameter": float(subtractiveConfig.get("toolDiameter", 6.0)),
                  "safetyMargin": float(subtractiveConfig.get("toolSafetyMargin", 0.5))}
    feedrate = float(subtractiveConfig.get("feedRate", 500.0))
    stepOver = float(subtractiveConfig.get("stepOver", 1.5))
    layerStep = float(subtractiveConfig.get("layerStepDown", 1.0))
    safeHeight = float(subtractiveConfig.get("safeHeight", 5.0))
    waterlineStep = float(subtractiveConfig.get("waterlineStepDown", 0.5))
    angleThreshold = float(subtractiveConfig.get("angleThreshold", 1.047))
    axisMode = str(subtractiveConfig.get("axisMode", "hemisphere"))

    candidateAxes = generateHemisphereAxes(
        int(subtractiveConfig.get("axisCount", 9))) if axisMode == "hemisphere" else subtractiveConfig.get(
        "candidateAxes", [[0.0, 0.0, 1.0]])

    stepParams = [
        {"mode": "zLevelRoughing", "stepOver": stepOver, "layerStep": layerStep, "safeHeight": safeHeight,
         "feedrate": feedrate},
        {"mode": "waterline", "sampling": stepOver, "layerStep": waterlineStep, "safeHeight": safeHeight,
         "feedrate": feedrate},
        {"mode": "dropRaster", "stepOver": stepOver, "safeHeight": safeHeight, "feedrate": feedrate,
         "angleThreshold": angleThreshold}
    ]

    generator = FiveAxisCncPathGenerator(version="1.1")
    clData = generator.generateJob(partStl, moldStl, gateStl, riserStl, toolParams, stepParams,
                                   {"candidateAxes": candidateAxes}, "WCS_MAIN", jobId)
    generator.exportClJson(clData, outputJsonPath)

    if visualize:
        PathVisualizer().visualize(generator.loadMesh(partStl), clData)

    return clData


if __name__ == '__main__':
    tempCncDir = Path("../tempCncFiles")
    if tempCncDir.exists():
        generateCncJobInterface(
            partStl=str(tempCncDir / "part.stl"),
            moldStl=str(tempCncDir / "mold.stl"),
            gateStl=str(tempCncDir / "gate.stl"),
            riserStl=str(tempCncDir / "riser.stl"),
            outputJsonPath=str(tempCncDir / "cncToolpath.json"),
            processConfig={
                "subtractive": {
                    "toolDiameter": 6.0,
                    "toolSafetyMargin": 0.5,
                    "feedRate": 500.0,
                    "stepOver": 1.5,
                    "layerStepDown": 1.0,
                    "safeHeight": 5.0,
                    "waterlineStepDown": 0.5,
                    "axisMode": "hemisphere",
                    "axisCount": 9,
                    "angleThreshold": 1.047
                }
            },
            jobId="JOB_TEST",
            visualize=True
        )
