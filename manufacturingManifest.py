import os
import json
from typing import Dict, Any, Optional
from dataModel import JobContext


class ManifestManager:
    """
    [Task A Refactoring]
    Simplified ManifestManager acting as a compatibility layer and persistence handler
    for JobContext. It no longer maintains independent state.
    """

    def __init__(self, jobContext: JobContext):
        self.jobContext = jobContext
        self.workspaceDir = jobContext.workspaceDir

        # Ensure workspace exists
        if not os.path.exists(self.workspaceDir):
            os.makedirs(self.workspaceDir, exist_ok=True)

    def registerArtifact(self, key: str, filePath: str, sourceStep: str,
                         versionTag: str = "", metadata: Optional[Dict] = None):
        """
        Registers a production artifact into the JobContext.
        """
        self.jobContext.registerArtifact(
            artifactKey=key,
            filePath=filePath,
            sourceStep=sourceStep,
            versionTag=versionTag,
            metrics=metadata
        )

    def getArtifactPath(self, key: str) -> str:
        """
        Retrieves the file path of a registered artifact from JobContext.
        """
        return self.jobContext.getArtifactPath(key)

    def setConfigSnapshot(self, config: Dict[str, Any]):
        """
        Legacy compatibility: Updates the config snapshot in JobContext.
        Ideally, config should be set during JobContext initialization via ConfigEngine.
        """
        # In the new architecture, configSnapshot is usually immutable after creation.
        # This method is kept only if strictly necessary for legacy mutable flows,
        # but normally we rely on the one already in jobContext.
        pass

    def save(self) -> str:
        """
        Persists the entire JobContext (including artifactsIndex and runtimeStatus) to disk.
        Returns the path to the saved job file (job.json).
        """
        return self.jobContext.saveToJson()

    @classmethod
    def load(cls, jobJsonPath: str) -> "ManifestManager":
        """
        Factory method to restore a manager from a serialized JobContext file.
        """
        if not os.path.exists(jobJsonPath):
            raise FileNotFoundError(f"Job file not found: {jobJsonPath}")

        ctx = JobContext.loadFromJson(jobJsonPath)
        return cls(ctx)
