import os
from enum import Enum
from typing import Any, Dict, Optional, Callable
from dataModel import JobContext, MoldPlan, ProcessPlan, NcProgramSet
import configEngine


class WorkflowState(str, Enum):
    Idle = "Idle"
    JobCreated = "JobCreated"
    MoldGenerated = "MoldGenerated"
    Sliced = "Sliced"
    ToolpathPlanned = "ToolpathPlanned"
    NcGenerated = "NcGenerated"
    PrintingMold = "PrintingMold"
    Casting = "Casting"
    Cooling = "Cooling"
    ShellRemovalMilling = "ShellRemovalMilling"
    FinishingMilling = "FinishingMilling"
    Completed = "Completed"
    Failed = "Failed"
    Canceled = "Canceled"


VALID_TRANSITIONS: Dict[str, Dict[str, str]] = {
    "CreateJob":         {WorkflowState.Idle: WorkflowState.JobCreated},
    "GenerateMold":      {WorkflowState.JobCreated: WorkflowState.MoldGenerated},
    "Slice":             {WorkflowState.MoldGenerated: WorkflowState.Sliced},
    "PlanToolpath":      {WorkflowState.Sliced: WorkflowState.ToolpathPlanned},
    "GenerateNc":        {WorkflowState.ToolpathPlanned: WorkflowState.NcGenerated},
    "StartPrint":        {WorkflowState.NcGenerated: WorkflowState.PrintingMold},
    "StartCast":         {WorkflowState.PrintingMold: WorkflowState.Casting},
    "StartCool":         {WorkflowState.Casting: WorkflowState.Cooling},
    "StartShellRemoval": {WorkflowState.Cooling: WorkflowState.ShellRemovalMilling},
    "StartFinish":       {WorkflowState.ShellRemovalMilling: WorkflowState.FinishingMilling},
    "CompleteFinish":    {WorkflowState.FinishingMilling: WorkflowState.Completed},
    "Abort":             {s: WorkflowState.Canceled for s in WorkflowState if s not in (WorkflowState.Completed, WorkflowState.Canceled)},
    "Retry":             {WorkflowState.Failed: WorkflowState.Idle},
}


class WorkflowManager:
    def __init__(self):
        self.jobContext: Optional[JobContext] = None
        self._handlers: Dict[str, Callable] = {}
        self._registerBuiltinHandlers()

    def _registerBuiltinHandlers(self):
        self._handlers["CreateJob"] = self._handleCreateJob
        self._handlers["GenerateMold"] = self._handleGenerateMold

    def _handleGenerateMold(self, payload: Dict[str, Any]):
        import moldGenerator
        if not self.jobContext:
            raise ValueError("Job context is not initialized")

        jobOverrides = payload.get("jobOverrides", {})
        if "mold" in jobOverrides:
            self.jobContext.configSnapshot.mold.update(jobOverrides["mold"])

        return moldGenerator.generateMold(self.jobContext)

    def registerHandler(self, eventName: str, handler: Callable):
        self._handlers[eventName] = handler

    def dispatch(self, eventName: str, payload: Optional[Dict[str, Any]] = None) -> Any:
        payload = payload or {}
        currentState = self.jobContext.currentState if self.jobContext else WorkflowState.Idle

        transitionMap = VALID_TRANSITIONS.get(eventName, {})
        nextState = transitionMap.get(currentState)
        if nextState is None:
            raise ValueError(f"Invalid transition: event={eventName}, state={currentState}")

        handler = self._handlers.get(eventName)
        result = None
        if handler:
            result = handler(payload)

        if self.jobContext:
            self.jobContext.currentState = nextState
            self.jobContext.runtimeStatus[eventName] = {"completedAt": __import__("time").time()}
            self.jobContext.saveToJson()

        return result

    def _handleCreateJob(self, payload: Dict[str, Any]) -> JobContext:
        workspaceDir = payload.get("workspaceDir", "./workspace")
        inputGeometryRef = payload.get("inputGeometryRef", "")
        jobOverrides = payload.get("jobOverrides", {})
        profileName = payload.get("profileName", "default")
        jobId = payload.get("jobId", None)

        os.makedirs(workspaceDir, exist_ok=True)

        snapshot = configEngine.buildSnapshot(
            jobOverrides=jobOverrides,
            profileName=profileName,
            workspaceDir=workspaceDir,
            jobId=jobId or "",
        )

        ctx = JobContext(
            workspaceDir=workspaceDir,
            inputGeometryRef=inputGeometryRef,
            configSnapshot=snapshot,
            currentState=WorkflowState.Idle,
        )
        if jobId:
            ctx.jobId = jobId

        snapshot.paths.workspaceDir = workspaceDir
        snapshot.paths.partStl = os.path.join(workspaceDir, "part.normalized.stl")
        snapshot.paths.moldShellStl = os.path.join(workspaceDir, "moldShell.normalized.stl")
        snapshot.paths.gatingStl = os.path.join(workspaceDir, "gating.normalized.stl")
        snapshot.paths.fdmGcode = os.path.join(workspaceDir, "fdm.gcode")
        snapshot.paths.cncClJson = os.path.join(workspaceDir, "cnc.cl.json")
        snapshot.paths.jobJson = os.path.join(workspaceDir, "job.json")

        ctx.moldPlan = MoldPlan()
        ctx.processPlan = ProcessPlan()
        ctx.ncProgramSet = NcProgramSet()

        self.jobContext = ctx
        if inputGeometryRef and os.path.exists(inputGeometryRef):
            ctx.registerArtifact("partMesh", inputGeometryRef, "CreateJob")

        ctx.saveToJson()
        return ctx
