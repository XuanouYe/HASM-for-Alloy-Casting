import numpy as np
import trimesh


def trimeshToPyVista(triMesh: trimesh.Trimesh):
    import pyvista as pv
    vertices = triMesh.vertices
    faces = triMesh.faces
    pvFaces = np.column_stack([np.full(len(faces), 3), faces]).flatten()
    return pv.PolyData(vertices, pvFaces)


def trimeshToVtkPolyData(triMesh: trimesh.Trimesh):
    import vtk
    vtkPoints = vtk.vtkPoints()
    for v in triMesh.vertices:
        vtkPoints.InsertNextPoint(float(v[0]), float(v[1]), float(v[2]))
    vtkCells = vtk.vtkCellArray()
    for f in triMesh.faces:
        tri = vtk.vtkTriangle()
        tri.GetPointIds().SetId(0, int(f[0]))
        tri.GetPointIds().SetId(1, int(f[1]))
        tri.GetPointIds().SetId(2, int(f[2]))
        vtkCells.InsertNextCell(tri)
    polyData = vtk.vtkPolyData()
    polyData.SetPoints(vtkPoints)
    polyData.SetPolys(vtkCells)
    normalsFilter = vtk.vtkPolyDataNormals()
    normalsFilter.SetInputData(polyData)
    normalsFilter.ComputePointNormalsOn()
    normalsFilter.Update()
    return normalsFilter.GetOutput()


def loadMeshFromFile(filePath: str) -> trimesh.Trimesh:
    loaded = trimesh.load(filePath)
    if isinstance(loaded, trimesh.Scene):
        loaded = loaded.dump(concatenate=True)
    loaded.fix_normals()
    return loaded


def exportMeshToStl(triMesh: trimesh.Trimesh, filePath: str) -> None:
    triMesh.export(filePath)
