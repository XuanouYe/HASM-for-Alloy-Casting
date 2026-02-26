import json
import os
import time
import uuid
import numpy as np
import trimesh
from dataclasses import dataclass, field
from typing import Dict, Any, List, Optional


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


@dataclass
class ArtifactRecord:
    artifactKey: str
    filePath: str
    sourceStep: str
    versionTag: str = ""
    contentHash: str = ""
    metrics: Dict[str, Any] = field(default_factory=dict)
    createdAt: float = field(default_factory=time.time)

    def toDict(self) -> Dict[str, Any]:
        return {
            "artifactKey": self.artifactKey,
            "filePath": self.filePath,
            "sourceStep": self.sourceStep,
            "versionTag": self.versionTag,
            "contentHash": self.contentHash,
            "metrics": self.metrics,
            "createdAt": self.createdAt,
        }

    @classmethod
    def fromDict(cls, d: Dict[str, Any]) -> "ArtifactRecord":
        return cls(
            artifactKey=d["artifactKey"],
            filePath=d["filePath"],
            sourceStep=d["sourceStep"],
            versionTag=d.get("versionTag", ""),
            contentHash=d.get("contentHash", ""),
            metrics=d.get("metrics", {}),
            createdAt=d.get("createdAt", time.time()),
        )


@dataclass
class PathsSnapshot:
    workspaceDir: str
    partStl: str = ""
    moldShellStl: str = ""
    gatingStl: str = ""
    fdmGcode: str = ""
    cncClJson: str = ""
    jobJson: str = ""

    def toDict(self) -> Dict[str, Any]:
        return self.__dict__.copy()


@dataclass
class ConfigSnapshot:
    version: str
    profileName: str
    mold: Dict[str, Any] = field(default_factory=dict)
    additive: Dict[str, Any] = field(default_factory=dict)
    casting: Dict[str, Any] = field(default_factory=dict)
    subtractive: Dict[str, Any] = field(default_factory=dict)
    fdm: Dict[str, Any] = field(default_factory=dict)
    paths: PathsSnapshot = field(default_factory=lambda: PathsSnapshot(workspaceDir=""))

    def toDict(self) -> Dict[str, Any]:
        d = {
            "version": self.version,
            "profileName": self.profileName,
            "mold": self.mold,
            "additive": self.additive,
            "casting": self.casting,
            "subtractive": self.subtractive,
            "fdm": self.fdm,
            "paths": self.paths.toDict(),
        }
        return d

    @classmethod
    def fromDict(cls, d: Dict[str, Any]) -> "ConfigSnapshot":
        pathsData = d.get("paths", {})
        paths = PathsSnapshot(workspaceDir=pathsData.get("workspaceDir", ""))
        paths.__dict__.update(pathsData)
        return cls(
            version=d.get("version", ""),
            profileName=d.get("profileName", "default"),
            mold=d.get("mold", {}),
            additive=d.get("additive", {}),
            casting=d.get("casting", {}),
            subtractive=d.get("subtractive", {}),
            fdm=d.get("fdm", {}),
            paths=paths,
        )


@dataclass
class MoldPlan:
    partMeshPath: str = ""
    moldShellPath: str = ""
    gatingPath: str = ""
    wcsTranslation: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    wcsScale: float = 1.0
    boundingBoxOffset: float = 2.0
    orientationMatrix: List[List[float]] = field(default_factory=list)

    def toDict(self) -> Dict[str, Any]:
        return self.__dict__.copy()


@dataclass
class ProcessPlan:
    sliceLayerCount: int = 0
    gcodeEstimatedTime: float = 0.0
    castingVolumeCm3: float = 0.0
    solidificationTimeSec: int = 1800

    def toDict(self) -> Dict[str, Any]:
        return self.__dict__.copy()


@dataclass
class NcProgramSet:
    shellRemovalNcPath: str = ""
    finishingNcPath: str = ""
    clJsonPath: str = ""
    toolpathCount: int = 0

    def toDict(self) -> Dict[str, Any]:
        return self.__dict__.copy()


@dataclass
class JobContext:
    jobId: str = field(default_factory=lambda: str(uuid.uuid4())[:8])
    workspaceDir: str = ""
    inputGeometryRef: str = ""
    configSnapshot: Optional[ConfigSnapshot] = None
    artifactsIndex: Dict[str, Dict[str, Any]] = field(default_factory=dict)
    runtimeStatus: Dict[str, Any] = field(default_factory=dict)
    currentState: str = "Idle"
    moldPlan: Optional[MoldPlan] = None
    processPlan: Optional[ProcessPlan] = None
    ncProgramSet: Optional[NcProgramSet] = None
    moldDesignContext: Optional[MoldDesignContext] = None

    def registerArtifact(self, artifactKey: str, filePath: str, sourceStep: str,
                         versionTag: str = "", contentHash: str = "",
                         metrics: Optional[Dict[str, Any]] = None):
        record = ArtifactRecord(
            artifactKey=artifactKey,
            filePath=filePath,
            sourceStep=sourceStep,
            versionTag=versionTag,
            contentHash=contentHash,
            metrics=metrics or {},
        )
        self.artifactsIndex[artifactKey] = record.toDict()

    def getArtifactPath(self, artifactKey: str) -> str:
        entry = self.artifactsIndex.get(artifactKey, {})
        return entry.get("filePath", "")

    def toSerializableDict(self) -> Dict[str, Any]:
        return {
            "jobId": self.jobId,
            "workspaceDir": self.workspaceDir,
            "inputGeometryRef": self.inputGeometryRef,
            "configSnapshot": self.configSnapshot.toDict() if self.configSnapshot else {},
            "artifactsIndex": self.artifactsIndex,
            "runtimeStatus": self.runtimeStatus,
            "currentState": self.currentState,
            "moldPlan": self.moldPlan.toDict() if self.moldPlan else {},
            "processPlan": self.processPlan.toDict() if self.processPlan else {},
            "ncProgramSet": self.ncProgramSet.toDict() if self.ncProgramSet else {},
        }

    def saveToJson(self):
        jobJsonPath = os.path.join(self.workspaceDir, "job.json")
        with open(jobJsonPath, 'w', encoding='utf-8') as f:
            json.dump(self.toSerializableDict(), f, ensure_ascii=False, indent=2)
        return jobJsonPath

    @classmethod
    def loadFromJson(cls, jobJsonPath: str) -> "JobContext":
        with open(jobJsonPath, 'r', encoding='utf-8') as f:
            d = json.load(f)
        ctx = cls(
            jobId=d.get("jobId", ""),
            workspaceDir=d.get("workspaceDir", ""),
            inputGeometryRef=d.get("inputGeometryRef", ""),
            artifactsIndex=d.get("artifactsIndex", {}),
            runtimeStatus=d.get("runtimeStatus", {}),
            currentState=d.get("currentState", "Idle"),
        )
        snapshotData = d.get("configSnapshot", {})
        if snapshotData:
            ctx.configSnapshot = ConfigSnapshot.fromDict(snapshotData)
        moldPlanData = d.get("moldPlan", {})
        if moldPlanData:
            ctx.moldPlan = MoldPlan(**moldPlanData)
        processPlanData = d.get("processPlan", {})
        if processPlanData:
            ctx.processPlan = ProcessPlan(**processPlanData)
        ncProgramSetData = d.get("ncProgramSet", {})
        if ncProgramSetData:
            ctx.ncProgramSet = NcProgramSet(**ncProgramSetData)
        return ctx


class ManifestManager:
    def __init__(self, workspaceDir: str):
        self.workspaceDir = workspaceDir
        if not os.path.exists(self.workspaceDir):
            os.makedirs(self.workspaceDir, exist_ok=True)

    @classmethod
    def fromJobContext(cls, jobContext: JobContext) -> "ManifestManager":
        mgr = cls(jobContext.workspaceDir)
        mgr._jobContext = jobContext
        return mgr

    def save(self) -> str:
        return self._jobContext.saveToJson()
