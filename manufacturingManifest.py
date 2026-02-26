import json
import numpy as np
import trimesh
from typing import Dict, Any, Tuple, Optional
from datetime import datetime


class ManufacturingManifest:
    def __init__(self, projectId: str):
        self.projectId = projectId
        self.timestamp = datetime.now().isoformat()
        self.wcsTransform = np.eye(4).tolist()
        self.files = {}
        self.parameters = {}

    def setWcsTransform(self, transformMatrix: np.ndarray):
        self.wcsTransform = transformMatrix.tolist()

    def addFile(self, key: str, path: str):
        self.files[key] = path

    def addParameters(self, key: str, params: Dict[str, Any]):
        self.parameters[key] = params

    def save(self, outputPath: str):
        data = {
            "projectId": self.projectId,
            "timestamp": self.timestamp,
            "wcsTransform": self.wcsTransform,
            "files": self.files,
            "parameters": self.parameters
        }
        with open(outputPath, 'w', encoding='utf-8') as f:
            json.dump(data, f, indent=2, ensure_ascii=False)


def normalizeMeshesToWcs(
        partMesh: trimesh.Trimesh,
        moldMesh: trimesh.Trimesh,
        gatingMesh: Optional[trimesh.Trimesh]
) -> Tuple[trimesh.Trimesh, trimesh.Trimesh, Optional[trimesh.Trimesh], np.ndarray]:
    meshesToBound = [partMesh, moldMesh]
    if gatingMesh is not None:
        meshesToBound.append(gatingMesh)

    combinedBounds = np.array([mesh.bounds for mesh in meshesToBound])
    globalMin = np.min(combinedBounds[:, 0, :], axis=0)
    globalMax = np.max(combinedBounds[:, 1, :], axis=0)

    centerX = (globalMin[0] + globalMax[0]) / 2.0
    centerY = (globalMin[1] + globalMax[1]) / 2.0
    minZ = globalMin[2]

    translation = np.array([-centerX, -centerY, -minZ])
    transformMatrix = trimesh.transformations.translation_matrix(translation)

    partNormalized = partMesh.copy()
    partNormalized.apply_transform(transformMatrix)

    moldNormalized = moldMesh.copy()
    moldNormalized.apply_transform(transformMatrix)

    gatingNormalized = None
    if gatingMesh is not None:
        gatingNormalized = gatingMesh.copy()
        gatingNormalized.apply_transform(transformMatrix)

    return partNormalized, moldNormalized, gatingNormalized, transformMatrix
