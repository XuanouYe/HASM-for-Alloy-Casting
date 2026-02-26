import sys
from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QApplication, QMessageBox, QVBoxLayout,
    QSplitter, QTabWidget, QProgressBar, QHBoxLayout, QPushButton, QLabel
)
from PyQt5.QtCore import Qt
from guiModelViewer import ModelViewerWidget
from guiMoldProcessPanel import MoldProcessPanel
from guiParameterPanel import ProcessParameterPanel
from guiWorkerThread import WorkerThread
from controlWorkflow import WorkflowManager


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("镓合金3D打印控制系统")
        self.setGeometry(100, 100, 1400, 800)

        self.workflowManager = WorkflowManager()

        self.initUI()
        self.initConnections()

    def initUI(self):
        centralWidget = QWidget()
        mainLayout = QVBoxLayout()

        self.horizontalSplitter = QSplitter(Qt.Horizontal)

        self.modelViewer = ModelViewerWidget()
        self.horizontalSplitter.addWidget(self.modelViewer)

        self.rightPanel = QWidget()
        rightLayout = QVBoxLayout()

        self.tabWidget = QTabWidget()
        self.moldProcessPanel = MoldProcessPanel()
        self.parameterPanel = ProcessParameterPanel()

        self.tabWidget.addTab(self.moldProcessPanel, "模具生成")
        self.tabWidget.addTab(self.parameterPanel, "工艺参数")
        rightLayout.addWidget(self.tabWidget)

        btnLayout = QHBoxLayout()
        self.btnGenerateGcode = QPushButton("生成 FDM G代码")
        self.btnGenerateGcode.setEnabled(False)
        self.btnGenerateCnc = QPushButton("生成 CNC 路径")
        self.btnGenerateCnc.setEnabled(False)

        btnLayout.addWidget(self.btnGenerateGcode)
        btnLayout.addWidget(self.btnGenerateCnc)
        rightLayout.addLayout(btnLayout)

        self.rightPanel.setLayout(rightLayout)
        self.horizontalSplitter.addWidget(self.rightPanel)
        self.horizontalSplitter.setSizes([900, 500])

        mainLayout.addWidget(self.horizontalSplitter)

        self.statusBar = self.statusBar()
        self.statusLabel = QLabel("就绪")
        self.statusBar.addWidget(self.statusLabel)

        centralWidget.setLayout(mainLayout)
        self.setCentralWidget(centralWidget)

    def initConnections(self):
        self.moldProcessPanel.requestLoadModel.connect(self.dispatchCreateJob)
        self.moldProcessPanel.requestMoldGenerate.connect(self.dispatchGenerateMold)

        self.moldProcessPanel.requestAddGating.connect(self.dispatchGenerateMold)
        self.moldProcessPanel.requestOrientationOptimize.connect(self.dispatchGenerateMold)
        self.moldProcessPanel.requestAdjustStructure.connect(self.dispatchGenerateMold)

        self.btnGenerateGcode.clicked.connect(self.dispatchStartPrint)
        self.btnGenerateCnc.clicked.connect(self.dispatchPlanToolpath)

    def _runWorker(self, eventName: str, payload: dict, onSuccess):
        self.statusLabel.setText(f"正在执行: {eventName}...")
        self.worker = WorkerThread(self.workflowManager, eventName, payload)
        self.worker.taskCompleted.connect(onSuccess)
        self.worker.taskError.connect(self.onTaskError)
        self.worker.start()

    def onTaskError(self, err: str):
        QMessageBox.critical(self, "错误", f"执行失败:\n{err}")
        self.statusLabel.setText("执行出错")

    def dispatchCreateJob(self, filePath: str):
        self.workflowManager.jobContext = None
        jobOverrides = self.parameterPanel.getConfiguration()
        payload = {
            "workspaceDir": "./workspace_gui",
            "inputGeometryRef": filePath,
            "jobOverrides": jobOverrides
        }
        self._runWorker("CreateJob", payload, self.onCreateJobDone)

    def onCreateJobDone(self, result):
        self.modelViewer.loadFromJobContext(self.workflowManager.jobContext)
        self.moldProcessPanel.updateStatus("项目已创建，请生成模具。")
        self.statusLabel.setText("项目就绪")

    def dispatchGenerateMold(self, moldOverrides: dict):
        fullOverrides = self.parameterPanel.getConfiguration()
        fullOverrides["mold"] = moldOverrides
        payload = {"jobOverrides": fullOverrides}
        self._runWorker("GenerateMold", payload, self.onGenerateMoldDone)

    def onGenerateMoldDone(self, result):
        self.modelViewer.loadFromJobContext(self.workflowManager.jobContext)
        self.moldProcessPanel.updateStatus("模具生成完毕。", enableNext=True)
        self.btnGenerateGcode.setEnabled(True)
        self.btnGenerateCnc.setEnabled(True)
        self.statusLabel.setText("模具就绪")

    def dispatchStartPrint(self):
        self.btnGenerateGcode.setEnabled(False)
        self._runWorker("StartPrint", {}, self.onPrintDone)

    def onPrintDone(self, result):
        self.btnGenerateGcode.setEnabled(True)
        QMessageBox.information(self, "成功", "FDM G代码已成功生成登记！")

    def dispatchPlanToolpath(self):
        self.btnGenerateCnc.setEnabled(False)
        self._runWorker("PlanToolpath", {}, self.onToolpathDone)

    def onToolpathDone(self, result):
        self.btnGenerateCnc.setEnabled(True)
        QMessageBox.information(self, "成功", "CNC 刀轨文件已生成！")


def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
