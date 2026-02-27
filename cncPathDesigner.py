import json
from dataclasses import dataclass
from typing import List, Dict, Optional, Any
import numpy as np
import trimesh
import vtk
from scipy.spatial.distance import cdist
from pathlib import Path


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


class TrimeshToolpathEngine:
    def __init__(self):
        self.toolRadius = 0.0

    def buildCutter(self, toolParams: Dict[str, Any]) -> Any:
        self.toolRadius = float(toolParams.get("diameter", 6.0)) / 2.0
        return toolParams

    def expandMesh(self, mesh: trimesh.Trimesh, margin: float) -> trimesh.Trimesh:
        v = np.asarray(mesh.vertices, dtype=float)
        n = np.asarray(mesh.vertex_normals, dtype=float)
        if n.shape[0] != v.shape[0]:
            mesh.rezero()
            n = np.asarray(mesh.vertex_normals, dtype=float)
        expandedV = v + n * float(margin)
        return trimesh.Trimesh(vertices=expandedV, faces=np.asarray(mesh.faces, dtype=int), process=False)

    def extractMachinableRegion(self, targetMesh: trimesh.Trimesh, keepOutMesh: trimesh.Trimesh,
                                margin: float) -> trimesh.Trimesh:
        expandedKeepOut = self.expandMesh(keepOutMesh, margin)
        try:
            return trimesh.boolean.difference([targetMesh, expandedKeepOut], engine="manifold")
        except:
            return targetMesh

    def generateDropCutterRasterLines(self, meshObject: trimesh.Trimesh, bounds: np.ndarray,
                                      rasterParams: Dict[str, Any]) -> List[np.ndarray]:
        if meshObject.is_empty:
            return []
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
            if linePts:
                paths.append(np.asarray(linePts, dtype=float))
        return paths

    def generateWaterlineLoops(self, meshObject: trimesh.Trimesh, cutter: Any, zLevels: np.ndarray, sampling: float) -> \
    List[np.ndarray]:
        if meshObject.is_empty:
            return []
        waterlines = []
        for z in zLevels:
            sliceResult = meshObject.section(plane_origin=[0, 0, z], plane_normal=[0, 0, 1])
            if sliceResult:
                slice2D, to3DMatrix = sliceResult.to_2D()
                for poly in slice2D.polygons_full:
                    offsetPoly = poly.buffer(self.toolRadius)
                    if not offsetPoly.is_empty:
                        geoms = offsetPoly.geoms if offsetPoly.geom_type == 'MultiPolygon' else [offsetPoly]
                        for p in geoms:
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
                chosenListIdx, reverse = minStartIdx, False
            else:
                chosenListIdx, reverse = minEndIdx, True
            chosenPathIdx = unvisited[chosenListIdx]
            nextPath = paths[chosenPathIdx] if not reverse else paths[chosenPathIdx][::-1]
            if len(linkedPaths) > 0:
                linkedPaths.append(np.vstack((np.array([currentPos[0], currentPos[1], safeZ], dtype=float),
                                              np.array([nextPath[0][0], nextPath[0][1], safeZ], dtype=float))))
            linkedPaths.append(nextPath)
            currentPos = nextPath[-1]
            unvisited.pop(chosenListIdx)
        return np.vstack(linkedPaths) if linkedPaths else np.array([])


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

        step1Region = self.toolpathEngine.extractMachinableRegion(moldMesh, trimesh.util.concatenate(
            [partMesh, gateMesh, riserMesh]), safetyMargin)
        step1 = self.generateStep(1, "shellRemoval", step1Region, toolParams, stepParams[0], candidateAxes,
                                  str(stepParams[0].get("mode", "dropRaster")), globalMinZ)

        step2Region = self.toolpathEngine.extractMachinableRegion(trimesh.util.concatenate([partMesh, riserMesh]),
                                                                  gateMesh, safetyMargin)
        step2 = self.generateStep(2, "partAndRiserFinishing", step2Region, toolParams, stepParams[1], candidateAxes,
                                  str(stepParams[1].get("mode", "waterline")), globalMinZ)

        step3Region = self.toolpathEngine.extractMachinableRegion(gateMesh, partMesh, safetyMargin)
        step3 = self.generateStep(3, "gateRemoval", step3Region, toolParams, stepParams[2],
                                  candidateAxes, str(stepParams[2].get("mode", "dropRaster")), globalMinZ)

        out = {"version": self.version, "wcsId": str(wcsId), "steps": [step1, step2, step3]}
        if jobId is not None:
            out["jobId"] = str(jobId)
        return out

    def generateStep(self, stepId: int, stepType: str, targetMesh: trimesh.Trimesh,
                     toolParams: Dict[str, Any], stepParam: Dict[str, Any], candidateAxes: List[np.ndarray],
                     mode: str, globalMinZ: float) -> Dict[str, Any]:
        feedrate = float(stepParam.get("feedrate", 500.0))
        toolRadius = float(toolParams.get("diameter", 6.0)) * 0.5
        platformSafeZ = float(globalMinZ + toolRadius + float(toolParams.get("safetyMargin", 0.5)))
        segments = []
        allClPoints = []
        pointId = 0

        for segmentId, toolAxis in enumerate(candidateAxes):
            rotToToolFrame = buildRotationFromTo(toolAxis, np.array([0.0, 0.0, 1.0], dtype=float))
            rotBack = rotToToolFrame.T
            rotatedTarget = self.rotateMesh(targetMesh, rotToToolFrame)
            localSafeZ = float(rotatedTarget.bounds[1][2]) + float(stepParam.get("safeHeight", 5.0))
            cutter = self.toolpathEngine.buildCutter(toolParams)

            if mode.lower() in ["waterline", "wl"]:
                sampling = float(stepParam.get("sampling", stepParam.get("stepOver", 0.5)))
                layerStep = float(stepParam.get("layerStep", 0.5))
                zLevels = np.arange(float(rotatedTarget.bounds[0][2]), float(rotatedTarget.bounds[1][2]) + layerStep,
                                    layerStep, dtype=float)
                rawPathsLocal = self.toolpathEngine.generateWaterlineLoops(rotatedTarget, cutter, zLevels, sampling)
            else:
                rawPathsLocal = self.toolpathEngine.generateDropCutterRasterLines(rotatedTarget,
                                                                                  np.asarray(rotatedTarget.bounds,
                                                                                             dtype=float),
                                                                                  dict(stepParam))

            validPathsLocal = []
            for pathLocal in rawPathsLocal:
                if len(pathLocal) == 0: continue
                pathWcs = applyRotation(pathLocal, rotBack)
                platformMask = pathWcs[:, 2] >= platformSafeZ

                currentSubPath = []
                for i, isValid in enumerate(platformMask.tolist()):
                    if isValid:
                        currentSubPath.append(pathLocal[i])
                    else:
                        if len(currentSubPath) > 0:
                            validPathsLocal.append(np.asarray(currentSubPath, dtype=float))
                            currentSubPath = []
                if len(currentSubPath) > 0:
                    validPathsLocal.append(np.asarray(currentSubPath, dtype=float))

            if validPathsLocal:
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


class PathVisualizer:
    def __init__(self):
        self.windowSize = (1024, 768)

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
        renderer.AddActor(self.createVTKMeshActor(targetMesh, (0.7, 0.7, 0.7), 0.5))
        colors = [(1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0), (1.0, 0.5, 0.0), (0.5, 0.0, 1.0)]
        for stepIndex, step in enumerate(clData.get("steps", [])):
            points = [p["position"] for p in step.get("clPoints", [])]
            if points:
                renderer.AddActor(self.createVTKPathActor(points, colors[stepIndex % len(colors)]))
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


def generateCncJobInterface(
        partStl: str,
        moldStl: str,
        gateStl: str,
        riserStl: str,
        outputJsonPath: str,
        processConfig: Dict[str, Any],
        jobId: str = "JOB_AUTO",
        visualize: bool = False
) -> Dict[str, Any]:
    subtractiveConfig = processConfig.get("subtractive", {})

    toolParams = {
        "type": "ball",
        "diameter": float(subtractiveConfig.get("toolDiameter", 6.0)),
        "safetyMargin": float(subtractiveConfig.get("toolSafetyMargin", 0.5))
    }

    feedrate = float(subtractiveConfig.get("feedRate", 500.0))
    stepOver = float(subtractiveConfig.get("stepOver", 1.5))
    safeHeight = float(subtractiveConfig.get("safeHeight", 5.0))
    waterlineStep = float(subtractiveConfig.get("waterlineStepDown", 0.5))

    stepParams = [
        {"mode": "dropRaster", "stepOver": stepOver, "safeHeight": safeHeight, "feedrate": feedrate},
        {"mode": "waterline", "sampling": stepOver, "layerStep": waterlineStep, "safeHeight": safeHeight,
         "feedrate": feedrate},
        {"mode": "dropRaster", "stepOver": stepOver, "safeHeight": safeHeight, "feedrate": feedrate}
    ]

    axisStrategyParams = {"candidateAxes": subtractiveConfig.get("candidateAxes", [[0.0, 0.0, 1.0]])}

    generator = FiveAxisCncPathGenerator(version="1.1")
    clData = generator.generateJob(
        partStl=partStl, moldStl=moldStl, gateStl=gateStl, riserStl=riserStl,
        toolParams=toolParams, stepParams=stepParams, axisStrategyParams=axisStrategyParams,
        wcsId="WCS_MAIN", jobId=jobId
    )

    generator.exportClJson(clData, outputJsonPath)

    if visualize:
        PathVisualizer().visualize(generator.loadMesh(partStl), clData)

    return clData


if __name__ == '__main__':
    tempCncDir = Path("tempCncFiles")
    if tempCncDir.exists():
        generateCncJobInterface(
            partStl=str(tempCncDir / "part.stl"),
            moldStl=str(tempCncDir / "mold.stl"),
            gateStl=str(tempCncDir / "gate.stl"),
            riserStl=str(tempCncDir / "riser.stl"),
            outputJsonPath=str(tempCncDir / "cncToolpath.json"),
            processConfig={
                "subtractive": {"toolDiameter": 6.0, "toolSafetyMargin": 0.5, "feedRate": 500.0, "stepOver": 1.5,
                                "safeHeight": 5.0, "waterlineStepDown": 0.5,
                                "candidateAxes": [[0.0, 0.0, 1.0], [1.0, 0.0, 0.0]]}},
            jobId="JOB_TEST",
            visualize=True
        )
