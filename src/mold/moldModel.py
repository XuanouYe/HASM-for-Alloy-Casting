import trimesh
import numpy as np
from datetime import datetime
from pathlib import Path

from src.control.controlConfigManager import ConfigManager, DirectoryManager
from src.control.controlConfigValidation import InfillPattern
from src.control.controlDataExchangeInterface import MessageRouter
from moldGenerator import generateMoldWithManifold, MoldConfig  # 导入模具生成模块

"""
@FUNCTION
    1. Input a product model file
    2. Generate the STL file of corresponding mold model with overhang structure
    3. Detect and remove the overhang structure of the mold model file
    4. Obtain the STL file of restructured product model
@FILE INPUT
@FILE OUTPUT
    1. 02-mold/mold_adjusted_yyyymmdd_v001.stl
    2. 02-mold/product_adjusted_yyyymmdd_v001.stl
"""


class FDMmodel:
    def __init__(self):
        self.configManager = ConfigManager(configDir="./config")
        self.directoryManager = DirectoryManager(baseDir="./projects")
        self.messageRouter = MessageRouter()
        self.messageRouter.registerModule("moldGenerationModule")
        self.messageRouter.registerModule("fdmModule")

        # 初始化模具配置
        self.defaultMoldConfig = MoldConfig(
            wallThickness=2.0,  # 默认壁厚2mm
            fixWatertight=True,
            checkVolume=False
        )

    def generateMoldModel(self, projectId: str = "LiquidMetalProject001"):
        try:
            projectDir = self.directoryManager.getFilePath(projectId)
            if not projectDir.exists():
                raise FileNotFoundError(f"Project directory not found: {projectDir}")

            modelDir = projectDir / "01-product"
            stlFiles = list(modelDir.glob("*.stl")) + list(modelDir.glob("*.STL"))

            if not stlFiles:
                raise FileNotFoundError(f"No STL files found in {modelDir}")

            productPath = stlFiles[0]
            print(f"Loading product model: {productPath}")

            productMesh = trimesh.load_mesh(str(productPath))

            moldDir = projectDir / "02-mold"
            moldDir.mkdir(parents=True, exist_ok=True)

            timestamp = datetime.now().strftime("%Y%m%d")
            version = "v001"

            moldOutputPath = moldDir / f"mold.adjusted.{timestamp}.{version}.stl"
            moldConfig = MoldConfig(
                wallThickness=self.defaultMoldConfig.wallThickness,
                fixWatertight=self.defaultMoldConfig.fixWatertight,
                checkVolume=self.defaultMoldConfig.checkVolume,
                outputPath=moldOutputPath
            )

            print(f"Generating mold model...")
            moldMesh = generateMoldWithManifold(productPath, moldConfig)
            print(f"Mold model saved to: {moldOutputPath}")

            productMesh.export(str(moldOutputPath))
            print(f"Adjusted product model saved to: {moldOutputPath}")

            return {
                "success": True,
                "moldPath": str(moldOutputPath),
                "productAdjustedPath": str(moldOutputPath),
                "message": "Mold generation completed successfully"
            }

        except Exception as e:
            errorMsg = f"Error generating mold model: {str(e)}"
            print(errorMsg)

            return {
                "success": False,
                "error": errorMsg
            }

    def optimizePrintDirection(self, mesh: trimesh.Trimesh) -> np.ndarray:
        pass

    def detectOverhang(self, mesh: trimesh.Trimesh, angle_threshold: float = 45.0) -> np.ndarray:
        pass

    def removeOverhang(self, mesh: trimesh.Trimesh) -> trimesh.Trimesh:
        pass

    def setMoldConfig(self, wallThickness: float = None,
                      fixWatertight: bool = None,
                      checkVolume: bool = None):
        pass