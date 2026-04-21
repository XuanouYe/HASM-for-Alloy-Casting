from typing import Dict, Any, Optional, Callable
from datetime import datetime
import logging
from enum import Enum
from controlConfig import ConfigManager

logger = logging.getLogger(__name__)


class WorkflowState(Enum):
    IDLE        = "idle"
    INITIALIZED = "initialized"
    RUNNING     = "running"
    PAUSED      = "paused"
    COMPLETED   = "completed"
    FAILED      = "failed"


class ExecutionStep:
    def __init__(self, stepName: str, stepDisplay: str):
        self.stepName    = stepName
        self.stepDisplay = stepDisplay
        self.status      = "pending"
        self.output      = None
        self.error       = None
        self.startTime   = None
        self.endTime     = None

    def toDict(self) -> Dict[str, Any]:
        return {
            "step":      self.stepName,
            "display":   self.stepDisplay,
            "status":    self.status,
            "output":    self.output,
            "error":     self.error,
            "startTime": self.startTime,
            "endTime":   self.endTime,
        }


class WorkflowManager:
    """
    工作流管理器，串联增材 → 铸造 → 减材三个步骤。

    moduleRegistry 约定：
      每个模块函数签名为 func(projectId: str, fullConfig: Dict[str, Any]) -> Any
      其中 fullConfig 为完整的配置字典（含 "additive"、"casting"、"subtractive" 等顶层键）。

    FIX: execute() 原来传 stepConfig（已展开的单段参数），
         导致 generateGcodeInterface 等函数无法通过 config.get("additive") 取到参数，
         回抽等配置全部 fallback 到默认值。
         现改为传完整 self.config，各模块函数自行按需取用所需段落。
    """

    def __init__(self, configManager: Optional[ConfigManager] = None):
        self.state              = WorkflowState.IDLE
        self.projectId          = None
        self.config             = None
        self.executionHistory   = []
        self.currentStepIndex   = -1
        self.configManager      = configManager or ConfigManager()

        self.steps = [
            ("additive",    "Additive Process - Mold Generation"),
            ("casting",     "Casting Process - Part Casting"),
            ("subtractive", "Subtractive Process - Mold Removal"),
        ]

        logger.info("Workflow manager initialized")

    def initialize(self, projectId: str, config: Optional[Dict[str, Any]] = None) -> bool:
        if self.state != WorkflowState.IDLE:
            logger.error(f"Workflow in wrong state for initialization: {self.state.value}")
            return False

        self.projectId = projectId
        self.config    = config if config is not None else self.configManager.loadConfig(projectId)

        self.state            = WorkflowState.INITIALIZED
        self.executionHistory = []
        self.currentStepIndex = -1

        logger.info(f"Workflow initialized: {projectId}")
        return True

    def execute(self, moduleRegistry: Dict[str, Callable]) -> bool:
        """
        顺序执行三个步骤。

        FIX: 每步调用时传入完整 self.config（而非 self.config.get(stepKey)），
             确保各模块函数能访问所有配置段（如 fdmExecutor 需要 additive + fdm 两段）。
        """
        if self.state != WorkflowState.INITIALIZED:
            logger.error(f"Workflow not properly initialized. Current state: {self.state.value}")
            return False

        self.state = WorkflowState.RUNNING
        logger.info(f"Starting workflow execution: {self.projectId}")

        try:
            for index, (stepKey, stepDisplay) in enumerate(self.steps):
                self.currentStepIndex = index

                stepRecord = ExecutionStep(stepKey, stepDisplay)
                self.executionHistory.append(stepRecord)

                logger.info(f"Executing step {index + 1}/3: {stepDisplay}")

                if stepKey not in moduleRegistry:
                    raise KeyError(f"Module not found in registry: {stepKey}")

                moduleFunc = moduleRegistry[stepKey]

                try:
                    stepRecord.status    = "running"
                    stepRecord.startTime = datetime.now().isoformat()

                    if stepKey == "additive":
                        gatingResult = self.config.get("gatingResult") if isinstance(self.config, dict) else None
                        if gatingResult is not None and getattr(gatingResult, "runnerPath", None):
                            sprueInlet = gatingResult.runnerPath[0]
                            self.config["sprueInletPos"] = [float(v) for v in sprueInlet]

                    output = moduleFunc(self.projectId, self.config)

                    stepRecord.status  = "completed"
                    stepRecord.output  = output
                    if isinstance(output, dict):
                        gatingComponents = output.get("gatingComponents")
                        if gatingComponents is not None:
                            self.config["gatingResult"] = gatingComponents
                    stepRecord.endTime = datetime.now().isoformat()
                    logger.info(f"Step completed successfully: {stepDisplay}")

                except Exception as e:
                    stepRecord.status  = "failed"
                    stepRecord.error   = str(e)
                    stepRecord.endTime = datetime.now().isoformat()
                    logger.error(f"Step failed: {stepDisplay} - {e}")
                    self.state = WorkflowState.FAILED
                    return False

            self.state = WorkflowState.COMPLETED
            logger.info(f"Workflow completed successfully: {self.projectId}")
            return True

        except Exception as e:
            logger.error(f"Workflow execution error: {e}")
            self.state = WorkflowState.FAILED
            return False

    def getStatus(self) -> Dict[str, Any]:
        return {
            "state":       self.state.value,
            "projectId":   self.projectId,
            "currentStep": self.currentStepIndex,
            "totalSteps":  len(self.steps),
            "progress":    f"{self.currentStepIndex + 1}/{len(self.steps)}" if self.currentStepIndex >= 0 else "0/3",
            "history":     [step.toDict() for step in self.executionHistory],
        }

    def reset(self):
        self.state            = WorkflowState.IDLE
        self.projectId        = None
        self.config           = None
        self.executionHistory = []
        self.currentStepIndex = -1
        logger.info("Workflow reset")
