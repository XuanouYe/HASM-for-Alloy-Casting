from typing import Callable
from PyQt5.QtCore import QThread, pyqtSignal

class WorkerThread(QThread):
    taskStarted = pyqtSignal()
    taskProgress = pyqtSignal(int)
    taskCompleted = pyqtSignal(object)
    taskError = pyqtSignal(str)

    def __init__(self, task: Callable, *args, **kwargs):
        super().__init__()
        self.task = task
        self.args = args
        self.kwargs = kwargs
        self.shouldStop = False

    def run(self):
        try:
            self.taskStarted.emit()
            result = self.task(*self.args, **self.kwargs)
            self.taskCompleted.emit(result)
        except Exception as e:
            self.taskError.emit(str(e))

    def stop(self):
        self.shouldStop = True
        self.wait()
