from typing import Any, Dict
from PyQt5.QtCore import QThread, pyqtSignal

class WorkerThread(QThread):
    taskStarted = pyqtSignal()
    taskCompleted = pyqtSignal(object)
    taskError = pyqtSignal(str)

    def __init__(self, workflowManager, eventName: str, payload: Dict[str, Any], parent=None):
        super().__init__(parent)
        self.workflowManager = workflowManager
        self.eventName = eventName
        self.payload = payload
        self.shouldStop = False

    def run(self):
        try:
            self.taskStarted.emit()
            result = self.workflowManager.dispatch(self.eventName, self.payload)
            self.taskCompleted.emit(result)
        except Exception as e:
            self.taskError.emit(str(e))

    def stop(self):
        self.shouldStop = True
        self.wait()
