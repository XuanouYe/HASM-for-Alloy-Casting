import json
import os
import time
import numpy as np
import trimesh
from dataclasses import dataclass, field
from typing import Dict, Any, List


@dataclass
class SupportRegionResult:
    layerGeoms: List
    sliceHeights: np.ndarray
    totalSupportArea: float


@dataclass
class MachiningRegionResult:
    points: np.ndarray
    unmachinableMask: np.ndarray
    unmachinablePoints: np.ndarray
    unmachinablePercent: float
    totalPoints: int
    unmachinablePointsCount: int


@dataclass
class GatingComponents:
    gateMesh: trimesh.Trimesh
    riserMesh: trimesh.Trimesh
    castingWithRiserMesh: trimesh.Trimesh
    systemMesh: trimesh.Trimesh
    gateSurface: np.ndarray
    runnerPath: List[np.ndarray]
    runnerRadius: float


@dataclass
class MoldDesignContext:
    partMesh: trimesh.Trimesh = None
    moldMesh: trimesh.Trimesh = None
    innerCavityMesh: trimesh.Trimesh = None
    gatingMesh: trimesh.Trimesh = None
    supportRegionResult: SupportRegionResult = None
    machiningRegionResult: MachiningRegionResult = None
    gatingComponents: GatingComponents = None
    wcsTransform: np.ndarray = field(default_factory=lambda: np.eye(4))


class ManifestManager:
    def __init__(self, workspaceDir: str):
        self.workspaceDir = workspaceDir
        if not os.path.exists(self.workspaceDir):
            os.makedirs(self.workspaceDir, exist_ok=True)

        self.manifest = {
            "timestamp": time.time(),
            "wcsTransform": {"translation": [0.0, 0.0, 0.0], "scale": 1.0},
            "files": {
                "part": os.path.join(self.workspaceDir, "part.normalized.stl"),
                "moldShell": os.path.join(self.workspaceDir, "moldShell.normalized.stl"),
                "gating": os.path.join(self.workspaceDir, "gating.normalized.stl"),
                "fdmGcode": os.path.join(self.workspaceDir, "fdm.gcode"),
                "cncClJson": os.path.join(self.workspaceDir, "cnc.cl.json"),
                "manifest": os.path.join(self.workspaceDir, "manifest.json")
            },
            "configSnapshot": {}
        }

    def getFilePaths(self) -> Dict[str, str]:
        return self.manifest["files"]

    def setWcsTransform(self, translation: List[float], scale: float = 1.0):
        self.manifest["wcsTransform"] = {"translation": translation, "scale": scale}

    def setConfigSnapshot(self, config: Dict[str, Any]):
        self.manifest["configSnapshot"] = config

    def save(self):
        manifestPath = self.manifest["files"]["manifest"]
        with open(manifestPath, 'w', encoding='utf-8') as f:
            json.dump(self.manifest, f, ensure_ascii=False, indent=2)
        return manifestPath

    @classmethod
    def load(cls, manifestPath: str) -> Dict[str, Any]:
        with open(manifestPath, 'r', encoding='utf-8') as f:
            return json.load(f)
