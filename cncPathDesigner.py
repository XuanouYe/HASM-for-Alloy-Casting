import json
from dataclasses import dataclass
from typing import List, Dict, Optional, Any
import numpy as np
import trimesh
import vtk
from scipy.spatial.distance import cdist


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


@dataclass
class SegmentOutput:
    segmentId: int
    toolAxis: List[float]
    pointCount: int


class KeepOutZoneManager:
    def __init__(self, targetMesh: trimesh.Trimesh, margin: float):
        self.margin = float(margin)
        self.keepOutMesh = self.buildExpandedMesh(targetMesh, self.margin)
        self.proxQuery = trimesh.proximity.ProximityQuery(self.keepOutMesh)

    def buildExpandedMesh(self, mesh: trimesh.Trimesh, margin: float) -> trimesh.Trimesh:
        v = np.asarray(mesh.vertices, dtype=float)
        n = np.asarray(mesh.vertex_normals, dtype=float)
        if n.shape[0] != v.shape[0]:
            mesh.rezero()
            n = np.asarray(mesh.vertex_normals, dtype=float)
        expandedV = v + n * float(margin)
        return trimesh.Trimesh(vertices=expandedV, faces=np.asarray(mesh.faces, dtype=int), process=False)

    def checkPointsSafe(self, points: np.ndarray, minDistance: float) -> np.ndarray:
        if len(points) == 0:
            return np.array([], dtype=bool)
        closestPts, distances, _ = trimesh.proximity.closest_point(self.keepOutMesh, points)
        distancesArray = np.asarray(distances, dtype=float)
        outsideDistanceSafe = distancesArray > float(minDistance)
        isInside = self.keepOutMesh.ray.contains_points(points)
        isInsideArray = np.asarray(isInside, dtype=bool)
        finalSafeMask = np.logical_and(outsideDistanceSafe, np.logical_not(isInsideArray))
        return finalSafeMask


class TrimeshToolpathEngine:
    def __init__(self):
        self.toolRadius = 0.0

    def buildCutter(self, toolParams: Dict[str, Any]) -> Any:
        self.toolRadius = float(toolParams.get("diameter", 6.0)) / 2.0
        return toolParams

    def generateDropCutterRasterLines(self, meshObject: trimesh.Trimesh, bounds: np.ndarray,
                                      rasterParams: Dict[str, Any]) -> List[np.ndarray]:
        stepOver = float(rasterParams.get("stepOver", 1.0))
        safeHeight = float(rasterParams.get("safeHeight", 5.0))
        xMin, yMin, zMin = bounds[0]
        xMax, yMax, zMax = bounds[1]
        xList = np.arange(xMin, xMax + stepOver, stepOver, dtype=float)
        yList = np.arange(yMin, yMax + stepOver, stepOver, dtype=float)
        zStart = float(zMax + safeHeight)
        paths = []
        for y in yList:
            rayOrigins = np.column_stack((xList, np.full(len(xList), y), np.full(len(xList), zStart)))
            rayDirections = np.tile([0, 0, -1], (len(xList), 1))
            locations, indexRay, _ = meshObject.ray.intersects_location(ray_origins=rayOrigins,
                                                                        ray_directions=rayDirections)
            linePts = []
            for i in range(len(xList)):
                matchIndices = np.where(indexRay == i)[0]
                if len(matchIndices) > 0:
                    highestZ = np.max(locations[matchIndices, 2])
                    linePts.append([xList[i], y, highestZ])
                else:
                    linePts.append([xList[i], y, zMin])
            if linePts:
                paths.append(np.asarray(linePts, dtype=float))
        return paths

    def generateWaterlineLoops(self, meshObject: trimesh.Trimesh, cutter: Any, zLevels: np.ndarray, sampling: float) -> \
    List[np.ndarray]:
        waterlines = []
        for z in zLevels:
            sliceResult = meshObject.section(plane_origin=[0, 0, z], plane_normal=[0, 0, 1])
            if sliceResult:
                slice2D, to3DMatrix = sliceResult.to_2D()
                for poly in slice2D.polygons_full:
                    offsetPoly = poly.buffer(self.toolRadius)
                    if not offsetPoly.is_empty:
                        if offsetPoly.geom_type == 'Polygon':
                            coords = np.array(offsetPoly.exterior.coords)
                            coordsHomo = np.column_stack((coords, np.zeros(len(coords)), np.ones(len(coords))))
                            coords3D = (to3DMatrix @ coordsHomo.T).T[:, :3]
                            waterlines.append(coords3D)
                        elif offsetPoly.geom_type == 'MultiPolygon':
                            for p in offsetPoly.geoms:
                                coords = np.array(p.exterior.coords)
                                coordsHomo = np.column_stack((coords, np.zeros(len(coords)), np.ones(len(coords))))
                                coords3D = (to3DMatrix @ coordsHomo.T).T[:, :3]
                                waterlines.append(coords3D)
        return waterlines

    def optimizePathLinking(self, paths: List[np.ndarray], safeZ: float) -> np.ndarray:
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
                chosenListIdx = minStartIdx
                reverse = False
            else:
                chosenListIdx = minEndIdx
                reverse = True
            chosenPathIdx = unvisited[chosenListIdx]
            nextPath = paths[chosenPathIdx]
            if reverse:
                nextPath = nextPath[::-1]
            if len(linkedPaths) > 0:
                retractUp = np.array([currentPos[0], currentPos[1], safeZ], dtype=float)
                moveOver = np.array([nextPath[0][0], nextPath[0][1], safeZ], dtype=float)
                linkedPaths.append(np.vstack((retractUp, moveOver)))
            linkedPaths.append(nextPath)
            currentPos = nextPath[-1]
            unvisited.pop(chosenListIdx)
        if len(linkedPaths) > 0:
            return np.vstack(linkedPaths)
        return np.array([])


class FiveAxisCncPathGenerator:
    def __init__(self, version: str = "1.0"):
        self.version = str(version)
        self.toolpathEngine = TrimeshToolpathEngine()

    def loadMesh(self, stlPath: str) -> trimesh.Trimesh:
        mesh = trimesh.load_mesh(stlPath)
        if isinstance(mesh, trimesh.Scene):
            mesh = trimesh.util.concatenate([g for g in mesh.geometry.values()])
        return mesh

    def generateJob(self, partStl: str, moldStl: str, gatingStl: str, toolParams: Dict[str, Any],
                    stepParams: List[Dict[str, Any]], axisStrategyParams: Dict[str, Any], wcsId: str = "WCS0",
                    jobId: Optional[str] = None) -> Dict[str, Any]:
        partMesh = self.loadMesh(partStl)
        moldMesh = self.loadMesh(moldStl)
        gatingMesh = self.loadMesh(gatingStl)
        globalMinZ = min([float(partMesh.bounds[0][2]), float(moldMesh.bounds[0][2]), float(gatingMesh.bounds[0][2])])
        candidateAxesRaw = axisStrategyParams.get("candidateAxes", [[0.0, 0.0, 1.0]])
        candidateAxesUnfiltered = [normalizeVector(np.array(a, dtype=float)) for a in candidateAxesRaw]
        candidateAxes = [ax for ax in candidateAxesUnfiltered if ax[2] >= 0.0]
        if not candidateAxes:
            candidateAxes = [np.array([0.0, 0.0, 1.0])]
        safetyMargin = float(toolParams.get("safetyMargin", 0.5))
        toolDiameter = float(toolParams.get("diameter", 6.0))
        toolRadius = toolDiameter * 0.5
        combinedObstacleMesh = trimesh.util.concatenate([partMesh, gatingMesh])
        step1KeepOut = KeepOutZoneManager(combinedObstacleMesh, safetyMargin)
        step1 = self.generateStep(1, "shellRemoval", moldMesh, step1KeepOut, toolParams, stepParams[0], candidateAxes,
                                  str(stepParams[0].get("mode", "dropRaster")), toolRadius, safetyMargin, globalMinZ)
        step2KeepOut = KeepOutZoneManager(gatingMesh, safetyMargin)
        step2 = self.generateStep(2, "partFinishing", partMesh, step2KeepOut, toolParams, stepParams[1], candidateAxes,
                                  str(stepParams[1].get("mode", "waterline")), toolRadius, safetyMargin, globalMinZ)
        step3KeepOut = KeepOutZoneManager(partMesh, safetyMargin)
        step3 = self.generateStep(3, "gatingRemoval", gatingMesh, step3KeepOut, toolParams, stepParams[2],
                                  candidateAxes, str(stepParams[2].get("mode", "dropRaster")), toolRadius, safetyMargin,
                                  globalMinZ)
        out = {"version": self.version, "wcsId": str(wcsId), "steps": [step1, step2, step3]}
        if jobId is not None:
            out["jobId"] = str(jobId)
        return out

    def generateStep(self, stepId: int, stepType: str, targetMesh: trimesh.Trimesh,
                     keepOutManager: Optional[KeepOutZoneManager], toolParams: Dict[str, Any],
                     stepParam: Dict[str, Any], candidateAxes: List[np.ndarray], mode: str, toolRadius: float,
                     safetyMargin: float, globalMinZ: float) -> Dict[str, Any]:
        feedrate = float(stepParam.get("feedrate", 500.0))
        minKeepOutDistance = float(stepParam.get("minKeepOutDistance", toolRadius + safetyMargin))
        platformSafeZ = float(globalMinZ + toolRadius + safetyMargin)
        segments = []
        allClPoints = []
        pointId = 0
        for segmentId, toolAxis in enumerate(candidateAxes):
            rotToToolFrame = buildRotationFromTo(toolAxis, np.array([0.0, 0.0, 1.0], dtype=float))
            rotBack = rotToToolFrame.T
            rotatedTarget = self.rotateMesh(targetMesh, rotToToolFrame)
            localSafeZ = float(rotatedTarget.bounds[1][2]) + float(stepParam.get("safeHeight", 5.0))
            cutter = self.toolpathEngine.buildCutter(toolParams)
            rawPathsLocal = []
            if mode.lower() in ["waterline", "wl"]:
                sampling = float(stepParam.get("sampling", stepParam.get("stepOver", 0.5)))
                layerStep = float(stepParam.get("layerStep", 0.5))
                zMin = float(rotatedTarget.bounds[0][2])
                zMax = float(rotatedTarget.bounds[1][2])
                zLevels = np.arange(zMin, zMax + layerStep, layerStep, dtype=float)
                rawPathsLocal = self.toolpathEngine.generateWaterlineLoops(rotatedTarget, cutter, zLevels, sampling)
            else:
                rasterParams = dict(stepParam)
                rawPathsLocal = self.toolpathEngine.generateDropCutterRasterLines(rotatedTarget,
                                                                                  np.asarray(rotatedTarget.bounds,
                                                                                             dtype=float), rasterParams)
            validPathsLocal = []
            for pathLocal in rawPathsLocal:
                if len(pathLocal) == 0:
                    continue
                pathWcs = applyRotation(pathLocal, rotBack)
                if keepOutManager is not None:
                    safeMask = keepOutManager.checkPointsSafe(pathWcs, minKeepOutDistance)
                else:
                    safeMask = np.ones(len(pathWcs), dtype=bool)
                platformMask = pathWcs[:, 2] >= platformSafeZ
                finalMask = np.logical_and(safeMask, platformMask)
                currentSubPath = []
                for i, isValid in enumerate(finalMask.tolist()):
                    if isValid:
                        currentSubPath.append(pathLocal[i])
                    else:
                        if len(currentSubPath) > 0:
                            validPathsLocal.append(np.asarray(currentSubPath, dtype=float))
                            currentSubPath = []
                if len(currentSubPath) > 0:
                    validPathsLocal.append(np.asarray(currentSubPath, dtype=float))
            validPathsLocal = [p for p in validPathsLocal if len(p) > 0]
            if len(validPathsLocal) > 0:
                optimizedLocalPath = self.toolpathEngine.optimizePathLinking(validPathsLocal, localSafeZ)
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
        v = np.asarray(mesh.vertices, dtype=float)
        f = np.asarray(mesh.faces, dtype=int)
        rv = applyRotation(v, rotMat)
        return trimesh.Trimesh(vertices=rv, faces=f, process=False)

    def buildClPointDicts(self, positions: np.ndarray, toolAxis: np.ndarray, feedrate: float, segmentId: int,
                          startPointId: int) -> List[Dict[str, Any]]:
        out = []
        toolAxis = normalizeVector(toolAxis)
        for i in range(positions.shape[0]):
            pid = int(startPointId + i)
            p = positions[i]
            d = {"pointId": pid, "position": [float(p[0]), float(p[1]), float(p[2])],
                 "toolAxis": [float(toolAxis[0]), float(toolAxis[1]), float(toolAxis[2])], "feedrate": float(feedrate),
                 "segmentId": int(segmentId)}
            out.append(d)
        return out

    def exportClJson(self, clData: Dict[str, Any], outputPath: str) -> None:
        with open(outputPath, 'w', encoding='utf-8') as f:
            json.dump(clData, f, ensure_ascii=False, indent=2)


class PathVisualizer:
    def __init__(self):
        self.windowSize = (1024, 768)

    def createVTKMeshActor(self, mesh: trimesh.Trimesh, color: tuple, opacity: float) -> vtk.vtkActor:
        vtkPoints = vtk.vtkPoints()
        for v in mesh.vertices:
            vtkPoints.InsertNextPoint(v)
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

    def createVTKPathActor(self, points: List[List[float]], color: tuple) -> vtk.vtkActor:
        vtkPoints = vtk.vtkPoints()
        vtkLines = vtk.vtkCellArray()
        for i, p in enumerate(points):
            vtkPoints.InsertNextPoint(p)
            if i > 0:
                line = vtk.vtkLine()
                line.GetPointIds().SetId(0, i - 1)
                line.GetPointIds().SetId(1, i)
                vtkLines.InsertNextCell(line)
        polyData = vtk.vtkPolyData()
        polyData.SetPoints(vtkPoints)
        polyData.SetLines(vtkLines)
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputData(polyData)
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(color)
        actor.GetProperty().SetLineWidth(2.0)
        return actor

    def visualize(self, targetMesh: trimesh.Trimesh, clData: Dict[str, Any]) -> None:
        renderer = vtk.vtkRenderer()
        renderer.SetBackground(1.0, 1.0, 1.0)
        meshActor = self.createVTKMeshActor(targetMesh, (0.7, 0.7, 0.7), 0.5)
        renderer.AddActor(meshActor)
        colors = [(1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0), (1.0, 0.5, 0.0), (0.5, 0.0, 1.0)]
        for stepIndex, step in enumerate(clData.get("steps", [])):
            clPoints = step.get("clPoints", [])
            points = []
            for p in clPoints:
                points.append(p["position"])
            if points:
                color = colors[stepIndex % len(colors)]
                pathActor = self.createVTKPathActor(points, color)
                renderer.AddActor(pathActor)
        renderWindow = vtk.vtkRenderWindow()
        renderWindow.AddRenderer(renderer)
        renderWindow.SetSize(self.windowSize[0], self.windowSize[1])
        renderWindow.SetWindowName("CNC Toolpath Visualization")
        interactor = vtk.vtkRenderWindowInteractor()
        interactor.SetRenderWindow(renderWindow)
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
        interactor.Start()
        renderWindow.Finalize()
        interactor.TerminateApp()
        del renderWindow, interactor, renderer


def generateCncJobInterface(
        partStl: str,
        moldStl: str,
        gatingStl: str,
        outputJsonPath: str,
        processConfig: Dict[str, Any],
        jobId: str = "JOB_AUTO",
        visualize: bool = False
) -> Dict[str, Any]:
    # Extract config from the flat subtractive config structure
    subtractiveConfig = processConfig.get("subtractive", {})

    # 1. Map Tool Params
    toolParams = {
        "type": "ball",  # Default to ball end mill
        "diameter": float(subtractiveConfig.get("toolDiameter", 6.0)),
        "safetyMargin": float(subtractiveConfig.get("toolSafetyMargin", 0.5))
    }

    # 2. Map Step Params (Common settings for all steps for now, can be expanded)
    feedrate = float(subtractiveConfig.get("feedRate", 500.0))
    stepOver = float(subtractiveConfig.get("stepOver", 1.5))
    safeHeight = float(subtractiveConfig.get("safeHeight", 5.0))
    waterlineStep = float(subtractiveConfig.get("waterlineStepDown", 0.5))

    stepParams = [
        # Step 1: Shell Removal (Roughing/Removal) - Drop Raster
        {
            "mode": "dropRaster",
            "stepOver": stepOver,
            "safeHeight": safeHeight,
            "feedrate": feedrate,
            "outputInvalidPoints": False
        },
        # Step 2: Part Finishing - Waterline
        {
            "mode": "waterline",
            "sampling": stepOver,  # For waterline, sampling along path often equals stepOver
            "layerStep": waterlineStep,
            "safeHeight": safeHeight,
            "feedrate": feedrate,
            "outputInvalidPoints": False
        },
        # Step 3: Gating Removal - Drop Raster
        {
            "mode": "dropRaster",
            "stepOver": stepOver,
            "safeHeight": safeHeight,
            "feedrate": feedrate,
            "outputInvalidPoints": False
        }
    ]

    # 3. Map Axis Strategy
    candidateAxes = subtractiveConfig.get("candidateAxes", [[0.0, 0.0, 1.0]])
    axisStrategyParams = {
        "candidateAxes": candidateAxes
    }

    # Execute Generator
    generator = FiveAxisCncPathGenerator(version="1.1")
    clData = generator.generateJob(
        partStl=partStl,
        moldStl=moldStl,
        gatingStl=gatingStl,
        toolParams=toolParams,
        stepParams=stepParams,
        axisStrategyParams=axisStrategyParams,
        wcsId="WCS_MAIN",
        jobId=jobId
    )

    # Export
    generator.exportClJson(clData, outputJsonPath)

    # Visualization (Optional)
    if visualize:
        partMesh = generator.loadMesh(partStl)
        visualizer = PathVisualizer()
        visualizer.visualize(partMesh, clData)

    return clData


def main():
    testConfig = {
        "subtractive": {
            "toolDiameter": 6.0,
            "toolSafetyMargin": 0.5,
            "feedRate": 1000,
            "stepOver": 1.2,
            "safeHeight": 10.0,
            "waterlineStepDown": 0.3,
            "candidateAxes": [[0.0, 0.0, 1.0], [0.5, 0.0, 0.866]]
        }
    }

    generateCncJobInterface(
        partStl="testModels/cylinder.down.stl",
        moldStl="testModels/cylinder.down.mold.stl",
        gatingStl="testModels/cylinder.down.gating.stl",
        outputJsonPath="cnc.cylinder.down.json",
        processConfig=testConfig,
        visualize=True
    )

if __name__ == "__main__":
    main()
