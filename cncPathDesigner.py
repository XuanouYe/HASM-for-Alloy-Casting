import json
from dataclasses import dataclass
from typing import List, Dict, Optional, Any
import numpy as np
import trimesh
import vtk

def normalizeVector(vec: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(vec))
    if norm == 0.0:
        return vec
    return vec / norm

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
    return (rotMat @ points.T).T

@dataclass
class SegmentOutput:
    segmentId: int
    toolAxis: List[float]
    pointCount: int

class KeepOutZoneManager:
    def __init__(self, gatingMesh: trimesh.Trimesh, margin: float):
        self.margin = float(margin)
        self.keepOutMesh = self.buildExpandedMesh(gatingMesh, self.margin)
        self.proxQuery = trimesh.proximity.ProximityQuery(self.keepOutMesh)

    def buildExpandedMesh(self, mesh: trimesh.Trimesh, margin: float) -> trimesh.Trimesh:
        v = np.asarray(mesh.vertices, dtype=float)
        n = np.asarray(mesh.vertex_normals, dtype=float)
        if n.shape[0] != v.shape[0]:
            mesh.rezero()
            n = np.asarray(mesh.vertex_normals, dtype=float)
        expandedV = v + n * float(margin)
        return trimesh.Trimesh(vertices=expandedV, faces=np.asarray(mesh.faces, dtype=int), process=False)

    def computeDistanceToKeepOut(self, points: np.ndarray) -> np.ndarray:
        closestPts, distances, _ = trimesh.proximity.closest_point(self.keepOutMesh, points)
        return np.asarray(distances, dtype=float)

    def filterByToolCenterDistance(self, points: np.ndarray, minDistance: float) -> np.ndarray:
        distances = self.computeDistanceToKeepOut(points)
        return distances >= float(minDistance)

class TrimeshToolpathEngine:
    def __init__(self):
        self.toolRadius = 0.0

    def buildCutter(self, toolParams: Dict[str, Any]) -> Any:
        self.toolRadius = float(toolParams.get("diameter", 6.0)) / 2.0
        return toolParams

    def generateDropCutterRaster(self, meshObject: trimesh.Trimesh, cutter: Any, bounds: np.ndarray, rasterParams: Dict[str, Any]) -> np.ndarray:
        stepOver = float(rasterParams.get("stepOver", 1.0))
        safeHeight = float(rasterParams.get("safeHeight", 5.0))
        xMin, yMin, zMin = bounds[0]
        xMax, yMax, zMax = bounds[1]
        xList = np.arange(xMin, xMax + stepOver, stepOver, dtype=float)
        yList = np.arange(yMin, yMax + stepOver, stepOver, dtype=float)
        zStart = float(zMax + safeHeight)
        gridX, gridY = np.meshgrid(xList, yList)
        flatX = gridX.flatten()
        flatY = gridY.flatten()
        rayOrigins = np.column_stack((flatX, flatY, np.full(len(flatX), zStart)))
        rayDirections = np.tile([0, 0, -1], (len(flatX), 1))
        locations, indexRay, _ = meshObject.ray.intersects_location(ray_origins=rayOrigins, ray_directions=rayDirections)
        points = []
        for i in range(len(flatX)):
            matchIndices = np.where(indexRay == i)[0]
            if len(matchIndices) > 0:
                highestZ = np.max(locations[matchIndices, 2])
                points.append([flatX[i], flatY[i], highestZ])
            else:
                points.append([flatX[i], flatY[i], zMin])
        return np.asarray(points, dtype=float)

    def generateWaterlineLoops(self, meshObject: trimesh.Trimesh, cutter: Any, zLevels: np.ndarray, sampling: float) -> List[np.ndarray]:
        waterlines = []
        for z in zLevels:
            sliceResult = meshObject.section(plane_origin=[0, 0, z], plane_normal=[0, 0, 1])
            if sliceResult:
                slice2D, _ = sliceResult.to_2D()
                for poly in slice2D.polygons_full:
                    offsetPoly = poly.buffer(self.toolRadius)
                    if not offsetPoly.is_empty:
                        if offsetPoly.geom_type == 'Polygon':
                            coords = np.array(offsetPoly.exterior.coords)
                            coords3d = np.column_stack((coords, np.full(len(coords), z)))
                            waterlines.append(coords3d)
                        elif offsetPoly.geom_type == 'MultiPolygon':
                            for p in offsetPoly.geoms:
                                coords = np.array(p.exterior.coords)
                                coords3d = np.column_stack((coords, np.full(len(coords), z)))
                                waterlines.append(coords3d)
        return waterlines

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
                    stepParams: List[Dict[str, Any]], axisStrategyParams: Dict[str, Any],
                    wcsId: str = "WCS_0", jobId: Optional[str] = None) -> Dict[str, Any]:
        partMesh = self.loadMesh(partStl)
        moldMesh = self.loadMesh(moldStl)
        gatingMesh = self.loadMesh(gatingStl)
        candidateAxesRaw = axisStrategyParams.get("candidateAxes", [[0.0, 0.0, 1.0]])
        candidateAxes = [normalizeVector(np.array(a, dtype=float)) for a in candidateAxesRaw]
        safetyMargin = float(toolParams.get("safetyMargin", 0.5))
        toolDiameter = float(toolParams.get("diameter", 6.0))
        toolRadius = toolDiameter * 0.5
        keepOutManager = KeepOutZoneManager(gatingMesh, safetyMargin)
        step1 = self.generateStep(stepId=1, stepType="shellRemoval", targetMesh=moldMesh, keepOutManager=keepOutManager,
                                  toolParams=toolParams, stepParam=stepParams[0], candidateAxes=candidateAxes,
                                  mode=str(stepParams[0].get("mode", "dropRaster")), toolRadius=toolRadius, safetyMargin=safetyMargin)
        step2 = self.generateStep(stepId=2, stepType="partFinishing", targetMesh=partMesh, keepOutManager=keepOutManager,
                                  toolParams=toolParams, stepParam=stepParams[1], candidateAxes=candidateAxes,
                                  mode=str(stepParams[1].get("mode", "waterline")), toolRadius=toolRadius, safetyMargin=safetyMargin)
        step3 = self.generateStep(stepId=3, stepType="gatingRemoval", targetMesh=gatingMesh, keepOutManager=None,
                                  toolParams=toolParams, stepParam=stepParams[2], candidateAxes=candidateAxes,
                                  mode=str(stepParams[2].get("mode", "dropRaster")), toolRadius=toolRadius, safetyMargin=safetyMargin)
        out = {"version": self.version, "wcsId": str(wcsId), "steps": [step1, step2, step3]}
        if jobId is not None:
            out["jobId"] = str(jobId)
        return out

    def generateStep(self, stepId: int, stepType: str, targetMesh: trimesh.Trimesh, keepOutManager: Optional[KeepOutZoneManager],
                     toolParams: Dict[str, Any], stepParam: Dict[str, Any], candidateAxes: List[np.ndarray],
                     mode: str, toolRadius: float, safetyMargin: float) -> Dict[str, Any]:
        feedrate = float(stepParam.get("feedrate", 500.0))
        outputInvalidPoints = bool(stepParam.get("outputInvalidPoints", False))
        minKeepOutDistance = float(stepParam.get("minKeepOutDistance", toolRadius + safetyMargin))
        bounds = np.asarray(targetMesh.bounds, dtype=float)
        segments = []
        allClPoints = []
        pointId = 0
        for segmentId, toolAxis in enumerate(candidateAxes):
            rotToToolFrame = buildRotationFromTo(toolAxis, np.array([0.0, 0.0, 1.0], dtype=float))
            rotBack = rotToToolFrame.T
            rotatedTarget = self.rotateMesh(targetMesh, rotToToolFrame)
            cutter = self.toolpathEngine.buildCutter(toolParams)
            clPointsWcs = []
            if mode.lower() in ["waterline", "wl"]:
                sampling = float(stepParam.get("sampling", stepParam.get("stepOver", 0.5)))
                layerStep = float(stepParam.get("layerStep", 0.5))
                zMin = float(rotatedTarget.bounds[0][2])
                zMax = float(rotatedTarget.bounds[1][2])
                zLevels = np.arange(zMin, zMax + layerStep, layerStep, dtype=float)
                loops = self.toolpathEngine.generateWaterlineLoops(rotatedTarget, cutter, zLevels, sampling)
                for loopId, loopPts in enumerate(loops):
                    loopPtsWcs = applyRotation(loopPts, rotBack)
                    clPointsWcs.extend(self.buildClPointDicts(loopPtsWcs, toolAxis, feedrate, segmentId, loopId, pointId))
                    pointId += len(loopPtsWcs)
            else:
                rasterParams = dict(stepParam)
                rasterPointsToolFrame = self.toolpathEngine.generateDropCutterRaster(rotatedTarget, cutter, np.asarray(rotatedTarget.bounds, dtype=float), rasterParams)
                if rasterPointsToolFrame.shape[0] > 0:
                    rasterPointsWcs = applyRotation(rasterPointsToolFrame, rotBack)
                    clPointsWcs.extend(self.buildClPointDicts(rasterPointsWcs, toolAxis, feedrate, segmentId, None, pointId))
                    pointId += len(rasterPointsWcs)
            if keepOutManager is not None and len(clPointsWcs) > 0:
                positions = np.asarray([p["position"] for p in clPointsWcs], dtype=float)
                safeMask = keepOutManager.filterByToolCenterDistance(positions, minKeepOutDistance)
                if outputInvalidPoints:
                    for i, ok in enumerate(safeMask.tolist()):
                        clPointsWcs[i]["validFlag"] = bool(ok)
                    clPointsWcs = clPointsWcs
                else:
                    clPointsWcs = [p for i, p in enumerate(clPointsWcs) if bool(safeMask[i])]
            segments.append({"segmentId": int(segmentId), "toolAxis": [float(toolAxis[0]), float(toolAxis[1]), float(toolAxis[2])], "pointCount": int(len(clPointsWcs))})
            allClPoints.extend(clPointsWcs)
        return {"stepId": int(stepId), "stepType": str(stepType), "toolParams": toolParams, "segments": segments, "clPoints": allClPoints}

    def rotateMesh(self, mesh: trimesh.Trimesh, rotMat: np.ndarray) -> trimesh.Trimesh:
        v = np.asarray(mesh.vertices, dtype=float)
        f = np.asarray(mesh.faces, dtype=int)
        rv = applyRotation(v, rotMat)
        return trimesh.Trimesh(vertices=rv, faces=f, process=False)

    def buildClPointDicts(self, positions: np.ndarray, toolAxis: np.ndarray, feedrate: float, segmentId: int,
                          loopId: Optional[int], startPointId: int) -> List[Dict[str, Any]]:
        out = []
        toolAxis = normalizeVector(toolAxis)
        for i in range(positions.shape[0]):
            pid = int(startPointId + i)
            p = positions[i]
            d = {"pointId": pid,
                 "position": [float(p[0]), float(p[1]), float(p[2])],
                 "toolAxis": [float(toolAxis[0]), float(toolAxis[1]), float(toolAxis[2])],
                 "feedrate": float(feedrate),
                 "segmentId": int(segmentId)}
            if loopId is not None:
                d["metadata"] = {"loopId": int(loopId)}
            out.append(d)
        return out

    def exportClJson(self, clData: Dict[str, Any], outputPath: str) -> None:
        with open(outputPath, "w", encoding="utf-8") as f:
            json.dump(clData, f, ensure_ascii=False, indent=2)

class PathVisualizer:
    def __init__(self):
        self.windowSize = (1024, 768)

    def _createVTKMeshActor(self, mesh: trimesh.Trimesh, color: tuple, opacity: float) -> vtk.vtkActor:
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

    def _createVTKPathActor(self, points: List[List[float]], color: tuple) -> vtk.vtkActor:
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
        meshActor = self._createVTKMeshActor(targetMesh, (0.7, 0.7, 0.7), 0.5)
        renderer.AddActor(meshActor)
        colors = [(1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0), (1.0, 0.5, 0.0), (0.5, 0.0, 1.0)]
        for stepIndex, step in enumerate(clData.get("steps", [])):
            clPoints = step.get("clPoints", [])
            points = []
            for p in clPoints:
                points.append(p["position"])
            if points:
                color = colors[stepIndex % len(colors)]
                pathActor = self._createVTKPathActor(points, color)
                renderer.AddActor(pathActor)
        renderWindow = vtk.vtkRenderWindow()
        renderWindow.AddRenderer(renderer)
        renderWindow.SetSize(self.windowSize[0], self.windowSize[1])
        renderWindow.SetWindowName('CNC Toolpath Visualization')
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

def buildSampleConfig() -> Dict[str, Any]:
    toolParams = {"type": "ball", "diameter": 6.0, "length": 50.0, "safetyMargin": 0.6}
    stepParams = [
        {"mode": "dropRaster", "stepOver": 1.5, "safeHeight": 5.0, "feedrate": 800.0, "outputInvalidPoints": False},
        {"mode": "waterline", "sampling": 0.5, "layerStep": 0.5, "feedrate": 600.0, "outputInvalidPoints": False},
        {"mode": "dropRaster", "stepOver": 1.5, "safeHeight": 5.0, "feedrate": 1000.0, "outputInvalidPoints": False}
    ]
    axisStrategyParams = {"candidateAxes": [[0.0, 0.0, 1.0], [0.70710678, 0.0, 0.70710678], [0.0, 0.70710678, 0.70710678]]}
    return {"toolParams": toolParams, "stepParams": stepParams, "axisStrategyParams": axisStrategyParams}

def main():
    config = buildSampleConfig()
    generator = FiveAxisCncPathGenerator(version="1.0")
    partMesh = generator.loadMesh("testModels/hollow.cylinder.left.with.riser.stl")
    clData = generator.generateJob(
        partStl             = "testModels/hollow.cylinder.left.with.riser.stl",
        moldStl             = "testModels/hollow.cylinder.left.mold.stl",
        gatingStl           = "testModels/hollow.cylinder.left.gating.stl",
        toolParams          = config["toolParams"],
        stepParams          = config["stepParams"],
        axisStrategyParams  = config["axisStrategyParams"],
        wcsId               = "WCS_MAIN",
        jobId               = "JOB_001"
    )
    generator.exportClJson(clData, "output_cl_data.json")
    visualizer = PathVisualizer()
    visualizer.visualize(partMesh, clData)

if __name__ == "__main__":
    main()
