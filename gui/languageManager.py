import json
from pathlib import Path
from PyQt5.QtCore import QObject, pyqtSignal

class LanguageManager(QObject):
    languageChanged = pyqtSignal()

    def __init__(self):
        super().__init__()
        self.settingsPath = Path("configs") / "appSettings.json"
        self.currentLanguage = "zh"
        self.loadLanguage()

    def loadLanguage(self):
        if self.settingsPath.exists():
            data = json.loads(self.settingsPath.read_text(encoding="utf-8"))
            self.currentLanguage = data.get("language", "zh")

    def setLanguage(self, language):
        if language == self.currentLanguage:
            return
        self.currentLanguage = language
        self.saveLanguage()
        self.languageChanged.emit()

    def saveLanguage(self):
        self.settingsPath.parent.mkdir(exist_ok=True, parents=True)
        data = {}
        if self.settingsPath.exists():
            data = json.loads(self.settingsPath.read_text(encoding="utf-8"))
        data["language"] = self.currentLanguage
        self.settingsPath.write_text(json.dumps(data, indent=2, ensure_ascii=False), encoding="utf-8")

    def text(self, key, **kwargs):
        value = texts.get(key, {}).get(self.currentLanguage, key)
        return value.format(**kwargs) if kwargs else value

languageManager = LanguageManager()

def tr(key, **kwargs):
    return languageManager.text(key, **kwargs)

texts = {
    "appTitle": {"zh": "增减材复合制造系统控制平台", "en": "Hybrid Additive-Subtractive Manufacturing Control Platform"},
    "menuFile": {"zh": "文件(&F)", "en": "File(&F)"}, "menuView": {"zh": "视图(&V)", "en": "View(&V)"}, "menuLanguage": {"zh": "语言", "en": "Language"},
    "menuChinese": {"zh": "中文", "en": "Chinese"}, "menuEnglish": {"zh": "English", "en": "English"},
    "actionNewProject": {"zh": "新建项目", "en": "New Project"}, "actionSaveProject": {"zh": "保存项目", "en": "Save Project"}, "actionLoadManifest": {"zh": "加载制造清单(Manifest)", "en": "Load Manufacturing Manifest"}, "actionExit": {"zh": "退出", "en": "Exit"}, "actionResetView": {"zh": "重置视角", "en": "Reset View"},
    "tabMoldProcess": {"zh": "模具生成", "en": "Mold Generation"}, "tabProcessParameters": {"zh": "工艺参数", "en": "Process Parameters"},
    "groupConfigFile": {"zh": "配置文件", "en": "Configuration File"}, "buttonLoadConfig": {"zh": "加载配置", "en": "Load Configuration"}, "buttonSaveConfig": {"zh": "保存配置", "en": "Save Configuration"}, "buttonResetDefault": {"zh": "重置为默认", "en": "Reset to Default"},
    "groupManufacturingFile": {"zh": "制造文件", "en": "Manufacturing Files"}, "buttonGenerateFdmGcode": {"zh": "生成FDM G代码", "en": "Generate FDM G-code"}, "buttonGenerateCncGcode": {"zh": "生成CNC G代码", "en": "Generate CNC G-code"}, "statusReady": {"zh": "就绪", "en": "Ready"}, "tooltipTogglePanel": {"zh": "折叠/展开右侧面板", "en": "Collapse/expand the right panel"},
    "dialogLoadManifest": {"zh": "加载制造清单", "en": "Load Manufacturing Manifest"}, "dialogLoadConfig": {"zh": "加载配置", "en": "Load Configuration"}, "dialogSaveConfig": {"zh": "保存配置", "en": "Save Configuration"}, "dialogSaveGcode": {"zh": "保存G代码", "en": "Save G-code"}, "dialogSaveCncGcode": {"zh": "保存CNC G代码", "en": "Save CNC G-code"}, "dialogVisualization": {"zh": "可视化", "en": "Visualization"}, "confirmCncVisualization": {"zh": "是否在生成后启动VTK可视化窗口检查刀路？", "en": "Open the VTK visualization window to inspect the toolpath after generation?"}, "confirmResetConfig": {"zh": "确定要重置为默认配置吗?", "en": "Reset the configuration to defaults?"},
    "titleMoldProcess": {"zh": "模具生成与处理", "en": "Mold Generation and Processing"}, "groupModelLoading": {"zh": "模型加载", "en": "Model Loading"}, "buttonLoadStl": {"zh": "加载STL文件", "en": "Load STL File"}, "groupAddGating": {"zh": "添加浇道", "en": "Add Gating"}, "labelRunnerDiameter": {"zh": "浇道直径:", "en": "Runner diameter:"}, "labelSprueOffset": {"zh": "浇口偏移:", "en": "Sprue offset:"}, "buttonAddGating": {"zh": "添加浇道", "en": "Add Gating"}, "groupCavityVolume": {"zh": "模腔体积估算", "en": "Cavity Volume Estimate"}, "labelCavityVolume": {"zh": "模腔体积:", "en": "Cavity volume:"}, "labelEstimatedMass": {"zh": "预估质量:", "en": "Estimated mass:"}, "groupMoldGeneration": {"zh": "模具生成", "en": "Mold Generation"}, "labelBoundingBoxOffset": {"zh": "包围盒偏移:", "en": "Bounding box offset:"}, "buttonGenerateMold": {"zh": "生成模具", "en": "Generate Mold"}, "buttonExportMold": {"zh": "导出模具 STL", "en": "Export Mold STL"}, "groupMoldBounds": {"zh": "模具包围盒规模", "en": "Mold Bounding Box Size"}, "groupPrintOrientation": {"zh": "打印方向调整", "en": "Print Orientation Adjustment"}, "buttonOptimizeOrientation": {"zh": "调整打印方向", "en": "Adjust Print Orientation"}, "groupSurfaceOffset": {"zh": "表面偏移", "en": "Surface Offset"}, "labelOffsetValue": {"zh": "偏移值:", "en": "Offset value:"}, "buttonExecuteOffset": {"zh": "执行偏移", "en": "Execute Offset"},
    "titleParameterConfig": {"zh": "工艺参数配置", "en": "Process Parameter Configuration"}, "tabAdditive": {"zh": "增材工艺", "en": "Additive Process"}, "tabCasting": {"zh": "铸造工艺", "en": "Casting Process"}, "tabSubtractive": {"zh": "减材加工", "en": "Subtractive Machining"}, "groupStepSelection": {"zh": "加工步骤选择", "en": "Machining Step Selection"}, "groupPerformanceSafety": {"zh": "性能与安全设置", "en": "Performance and Safety Settings"}, "groupAxisLimits": {"zh": "Axis Limits", "en": "Axis Limits"}, "labelMin": {"zh": "Min:", "en": "Min:"}, "labelMax": {"zh": "Max:", "en": "Max:"},
    "success": {"zh": "成功", "en": "Success"}, "warning": {"zh": "警告", "en": "Warning"}, "error": {"zh": "错误", "en": "Error"}, "complete": {"zh": "完成", "en": "Complete"}, "prompt": {"zh": "提示", "en": "Information"}, "exportSuccess": {"zh": "导出成功", "en": "Export Successful"},
    "statusNewProject": {"zh": "新项目已创建", "en": "New project created"}, "statusSavingProject": {"zh": "项目正在保存...", "en": "Saving project..."}, "messageProjectSaved": {"zh": "项目已保存", "en": "Project saved"}, "messageManifestLoaded": {"zh": "制造清单加载成功，可以生成CNC路径", "en": "Manufacturing manifest loaded. CNC toolpaths can now be generated."}, "messageManifestMissing": {"zh": "制造清单缺少必要的部件模型或模具模型路径", "en": "The manufacturing manifest is missing required part or mold model paths."}, "statusCastingLoaded": {"zh": "铸件模型已加载", "en": "Casting model loaded"}, "messageMoldExported": {"zh": "模具STL已保存至:\n{filePath}", "en": "Mold STL saved to:\n{filePath}"}, "statusMoldExported": {"zh": "模具已导出: {fileName}", "en": "Mold exported: {fileName}"}, "messageConfigLoaded": {"zh": "配置已加载: {filePath}", "en": "Configuration loaded: {filePath}"}, "messageConfigSaved": {"zh": "配置已保存: {filePath}", "en": "Configuration saved: {filePath}"}, "messageConfigReset": {"zh": "已重置为默认配置", "en": "Configuration reset to defaults"}, "messageGcodeGenerated": {"zh": "G代码已生成:\n{gcodePath}", "en": "G-code generated:\n{gcodePath}"}, "messageGenerationComplete": {"zh": "生成完成", "en": "Generation complete"}, "messageMissingModels": {"zh": "缺少必要的模型数据。请先加载制造清单或生成模具。", "en": "Required model data is missing. Load a manufacturing manifest or generate a mold first."}, "statusCncCalculating": {"zh": "正在计算CNC G代码...", "en": "Calculating CNC G-code..."}, "statusCncComplete": {"zh": "CNC G代码生成完成", "en": "CNC G-code generation complete"}, "messageCncGenerated": {"zh": "CNC G代码已成功生成。", "en": "CNC G-code generated successfully."},
    "dialogOpenStl": {"zh": "打开STL文件", "en": "Open STL File"}, "dialogExportMoldStl": {"zh": "导出模具STL", "en": "Export Mold STL"}, "statusLoadingFile": {"zh": "加载中: {fileName}...", "en": "Loading: {fileName}..."}, "statusGeneratingMold": {"zh": "正在生成模具...", "en": "Generating mold..."}, "statusAddingGating": {"zh": "正在添加浇道...", "en": "Adding gating..."}, "statusOrientationCalculating": {"zh": "打印方向调整计算中{dots}", "en": "Calculating print orientation{dots}"}, "statusExecutingOffset": {"zh": "正在执行表面偏移({offsetValue} mm)...", "en": "Executing surface offset ({offsetValue} mm)..."}, "statusModelLoadedReady": {"zh": "模型已加载，可调整打印方向或生成模具", "en": "Model loaded. Print orientation can be adjusted or the mold can be generated."}, "messageMoldGenerated": {"zh": "模具生成完成", "en": "Mold generation complete"}, "messageGatingAdded": {"zh": "浇道添加完成，请继续生成模具", "en": "Gating added. Continue to generate the mold."}, "messageOrientationDone": {"zh": "打印方向调整已完成\n最优旋转角: {summaryMsg}", "en": "Print orientation adjustment complete\nOptimal rotation angle: {summaryMsg}"}, "messageOffsetDone": {"zh": "表面偏移已完成 ({offsetValue} mm)", "en": "Surface offset complete ({offsetValue} mm)"}, "statusMolded": {"zh": "✓ 模具已生成", "en": "✓ Mold generated"}, "statusGated": {"zh": "✓ 浇道已添加", "en": "✓ Gating added"}, "statusOriented": {"zh": "✓ 方向已调整", "en": "✓ Orientation adjusted"}, "statusAdjusted": {"zh": "✓ 表面已偏移", "en": "✓ Surface offset applied"},
    "dialogCncVisualization": {"zh": "CNC 刀轨可视化", "en": "CNC Toolpath Visualization"}, "buttonShowAll": {"zh": "显示全部({stepCount})", "en": "Show All ({stepCount})"}, "checkShowLinks": {"zh": "显示抬刀/连接", "en": "Show retract/link moves"}, "checkShowCollision": {"zh": "显示碰撞路径", "en": "Show collision paths"}
}
