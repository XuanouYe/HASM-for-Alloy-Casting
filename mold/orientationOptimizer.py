import csv
import math
from dataclasses import dataclass
from typing import Dict, Tuple, Optional, List, Union
import numpy as np
import trimesh
from moldGenerator import MoldGenerator
from supportRegionDetector import SupportRegionDetector
from machiningRegionDetector import AccessibilityAnalyzer


@dataclass
class OptimizerConfig:
    seed: Optional[int] = 0
    s1Tolerance: float = 0.01
    s2Tolerance: float = 0.01
    numSamples: int = 2000
    raysPerPoint: int = 64
    boundingBoxOffset: float = 2.0
    booleanEngine: Optional[str] = None
    supportAngle: float = 45.0
    layerHeight: float = 0.5
    smoothHeight: float = 1.0
    areaEps: float = 1e-6
    angleQuantizationDeg: float = 0.05
    logCsvPath: Optional[str] = "orientationEvalLog.csv"
    saveBestMoldPath: Optional[str] = None
    gaPopulationSize: int = 30
    gaGenerations: int = 25
    gaMutationRate: float = 0.1
    gaCrossoverRate: float = 0.8
    gaTournamentSize: int = 3
    gaEliteRatio: float = 0.1
    gaMutationStep: float = 0.5
    gaSeed: Optional[int] = None
    optimizationPriority: str = "machining"


@dataclass
class _GAIndividual:
    alpha: float
    beta: float
    s1: float = None
    s2: float = None


class MoldOrientationOptimizer:
    ANGLE_MIN = -180.0
    ANGLE_MAX = 180.0

    def __init__(self, config: OptimizerConfig):
        self.config = config
        self.bestAlpha: Optional[float] = None
        self.bestBeta: Optional[float] = None
        self.bestS1: Optional[float] = None
        self.bestS2: Optional[float] = None
        self.evalCache: Dict[Tuple[float, float], Tuple[float, float]] = {}
        self.evalRows: List[Tuple[float, float, float, float]] = []
        self.moldConfig = {
            "boundingBoxOffset": float(self.config.boundingBoxOffset),
            "booleanEngine": self.config.booleanEngine,
        }
        self.supportConfig = {
            "supportAngle": float(self.config.supportAngle),
            "layerHeight": float(self.config.layerHeight),
            "smoothHeight": float(self.config.smoothHeight),
            "areaEps": float(self.config.areaEps),
        }
        gaSeed = self.config.gaSeed if self.config.gaSeed is not None else self.config.seed
        if gaSeed is not None:
            np.random.seed(gaSeed)

    def createRotationMatrix(self, alphaDeg: float, betaDeg: float) -> np.ndarray:
        alphaRad = math.radians(alphaDeg)
        betaRad = math.radians(betaDeg)
        rotX = np.array(
            [[1.0, 0.0, 0.0, 0.0],
             [0.0, math.cos(alphaRad), -math.sin(alphaRad), 0.0],
             [0.0, math.sin(alphaRad), math.cos(alphaRad), 0.0],
             [0.0, 0.0, 0.0, 1.0]], dtype=float)
        rotY = np.array(
            [[math.cos(betaRad), 0.0, math.sin(betaRad), 0.0],
             [0.0, 1.0, 0.0, 0.0],
             [-math.sin(betaRad), 0.0, math.cos(betaRad), 0.0],
             [0.0, 0.0, 0.0, 1.0]], dtype=float)
        return rotY @ rotX

    def placeOnBuildPlate(self, mesh: trimesh.Trimesh) -> trimesh.Trimesh:
        placedMesh = mesh.copy()
        zMin = float(placedMesh.bounds[0, 2])
        placedMesh.apply_translation([0.0, 0.0, -zMin])
        return placedMesh

    def rotateMesh(self, mesh: trimesh.Trimesh, alphaDeg: float, betaDeg: float) -> trimesh.Trimesh:
        rotatedMesh = mesh.copy()
        centroid = rotatedMesh.centroid
        rotatedMesh.apply_translation(-centroid)
        rotatedMesh.apply_transform(self.createRotationMatrix(alphaDeg, betaDeg))
        rotatedMesh.apply_translation(centroid)
        rotatedMesh = self.placeOnBuildPlate(rotatedMesh)
        rotatedMesh.fix_normals()
        return rotatedMesh

    def quantizeAngles(self, alphaDeg: float, betaDeg: float) -> Tuple[float, float]:
        q = float(self.config.angleQuantizationDeg)
        if q <= 0:
            return float(alphaDeg), float(betaDeg)
        alphaQ = round(alphaDeg / q) * q
        betaQ = round(betaDeg / q) * q
        return float(alphaQ), float(betaQ)

    def calculateS1Area(self, castingMesh: trimesh.Trimesh) -> float:
        analyzer = AccessibilityAnalyzer(castingMesh)
        result = analyzer.analyze(
            numSamples=int(self.config.numSamples),
            raysPerPoint=int(self.config.raysPerPoint),
            exportCsv=False,
        )
        unmachinablePercent = float(result.unmachinablePercent)
        return float((unmachinablePercent / 100.0) * float(castingMesh.area))

    def calculateS2Area(self, moldMesh: trimesh.Trimesh) -> float:
        detector = SupportRegionDetector(self.supportConfig)
        supportResult = detector.calculateSupportRegions(moldMesh)
        return float(supportResult.totalSupportArea)

    def updateBestLexicographic(self, alphaDeg: float, betaDeg: float, s1: float, s2: float) -> None:
        if self.bestS1 is None:
            self.bestAlpha, self.bestBeta, self.bestS1, self.bestS2 = alphaDeg, betaDeg, s1, s2
            return
        bestS1 = float(self.bestS1)
        bestS2 = float(self.bestS2)
        if self.config.optimizationPriority == "support":
            if s2 < bestS2 * (1.0 - 1e-12):
                self.bestAlpha, self.bestBeta, self.bestS1, self.bestS2 = alphaDeg, betaDeg, s1, s2
                return
            s2Close = abs(s2 - bestS2) <= bestS2 * float(self.config.s2Tolerance)
            if s2Close and (s1 < bestS1 * (1.0 - 1e-12)):
                self.bestAlpha, self.bestBeta, self.bestS1, self.bestS2 = alphaDeg, betaDeg, s1, s2
        else:
            if s1 < bestS1 * (1.0 - 1e-12):
                self.bestAlpha, self.bestBeta, self.bestS1, self.bestS2 = alphaDeg, betaDeg, s1, s2
                return
            s1Close = abs(s1 - bestS1) <= bestS1 * float(self.config.s1Tolerance)
            if s1Close and (s2 < bestS2 * (1.0 - 1e-12)):
                self.bestAlpha, self.bestBeta, self.bestS1, self.bestS2 = alphaDeg, betaDeg, s1, s2

    def evaluateAngles(self, originalCasting: trimesh.Trimesh, alphaDeg: float, betaDeg: float) -> Tuple[float, float]:
        alphaQ, betaQ = self.quantizeAngles(alphaDeg, betaDeg)
        cacheKey = (alphaQ, betaQ)
        if cacheKey in self.evalCache:
            return self.evalCache[cacheKey]
        try:
            rotatedCasting = self.rotateMesh(originalCasting, alphaQ, betaQ)
            s1 = self.calculateS1Area(rotatedCasting)
            moldGen = MoldGenerator(config=self.moldConfig)
            moldMesh = moldGen.generateMoldShell(rotatedCasting)
            s2 = self.calculateS2Area(moldMesh)
        except Exception:
            s1 = 1e30
            s2 = 1e30
        self.evalCache[cacheKey] = (float(s1), float(s2))
        self.evalRows.append((alphaQ, betaQ, float(s1), float(s2)))
        self.updateBestLexicographic(alphaQ, betaQ, float(s1), float(s2))
        return float(s1), float(s2)

    def _initializePopulation(self, originalMesh: trimesh.Trimesh) -> List[_GAIndividual]:
        population = []
        for _ in range(self.config.gaPopulationSize):
            alpha = np.random.uniform(self.ANGLE_MIN, self.ANGLE_MAX)
            beta = np.random.uniform(self.ANGLE_MIN, self.ANGLE_MAX)
            population.append(_GAIndividual(alpha=float(alpha), beta=float(beta)))
        return population

    def _evaluatePopulation(self, originalMesh: trimesh.Trimesh, population: List[_GAIndividual]) -> None:
        for ind in population:
            if ind.s1 is None or ind.s2 is None:
                s1, s2 = self.evaluateAngles(originalMesh, ind.alpha, ind.beta)
                ind.s1 = float(s1)
                ind.s2 = float(s2)

    def _lexicographicCompare(self, ind1: _GAIndividual, ind2: _GAIndividual) -> bool:
        if ind1.s1 is None or ind2.s1 is None:
            return False
        s1_1, s2_1 = float(ind1.s1), float(ind1.s2)
        s1_2, s2_2 = float(ind2.s1), float(ind2.s2)
        if self.config.optimizationPriority == "support":
            if s2_1 < s2_2 * (1.0 - 1e-12):
                return True
            if s2_2 < s2_1 * (1.0 - 1e-12):
                return False
            s2Close = abs(s2_1 - s2_2) <= max(s2_1, s2_2) * float(self.config.s2Tolerance)
            if s2Close:
                return s1_1 < s1_2 * (1.0 - 1e-12)
            return False
        else:
            if s1_1 < s1_2 * (1.0 - 1e-12):
                return True
            if s1_2 < s1_1 * (1.0 - 1e-12):
                return False
            s1Close = abs(s1_1 - s1_2) <= max(s1_1, s1_2) * float(self.config.s1Tolerance)
            if s1Close:
                return s2_1 < s2_2 * (1.0 - 1e-12)
            return False

    def _getFitnessTuple(self, ind: _GAIndividual) -> Tuple[float, float]:
        if self.config.optimizationPriority == "support":
            return (float(ind.s2) if ind.s2 is not None else 1e30,
                    float(ind.s1) if ind.s1 is not None else 1e30)
        else:
            return (float(ind.s1) if ind.s1 is not None else 1e30,
                    float(ind.s2) if ind.s2 is not None else 1e30)

    def _tournamentSelection(self, population: List[_GAIndividual]) -> _GAIndividual:
        tournamentSize = min(self.config.gaTournamentSize, len(population))
        candidates = np.random.choice(population, size=tournamentSize, replace=False)
        best = candidates[0]
        for candidate in candidates[1:]:
            if self._lexicographicCompare(candidate, best):
                best = candidate
        return best

    def _crossover(self, parent1: _GAIndividual, parent2: _GAIndividual) -> Tuple[_GAIndividual, _GAIndividual]:
        eta = 20.0
        if np.random.random() < self.config.gaCrossoverRate:
            p1_alpha, p2_alpha = parent1.alpha, parent2.alpha
            u = np.random.random()
            beta_q = (2.0 * u) ** (1.0 / (eta + 1.0)) if u <= 0.5 else (1.0 / (2.0 * (1.0 - u))) ** (1.0 / (eta + 1.0))
            c1_alpha = np.clip(0.5 * ((1.0 + beta_q) * p1_alpha + (1.0 - beta_q) * p2_alpha), self.ANGLE_MIN, self.ANGLE_MAX)
            c2_alpha = np.clip(0.5 * ((1.0 - beta_q) * p1_alpha + (1.0 + beta_q) * p2_alpha), self.ANGLE_MIN, self.ANGLE_MAX)
            p1_beta, p2_beta = parent1.beta, parent2.beta
            u = np.random.random()
            beta_q = (2.0 * u) ** (1.0 / (eta + 1.0)) if u <= 0.5 else (1.0 / (2.0 * (1.0 - u))) ** (1.0 / (eta + 1.0))
            c1_beta = np.clip(0.5 * ((1.0 + beta_q) * p1_beta + (1.0 - beta_q) * p2_beta), self.ANGLE_MIN, self.ANGLE_MAX)
            c2_beta = np.clip(0.5 * ((1.0 - beta_q) * p1_beta + (1.0 + beta_q) * p2_beta), self.ANGLE_MIN, self.ANGLE_MAX)
        else:
            c1_alpha, c1_beta = parent1.alpha, parent1.beta
            c2_alpha, c2_beta = parent2.alpha, parent2.beta
        return _GAIndividual(alpha=float(c1_alpha), beta=float(c1_beta)), _GAIndividual(alpha=float(c2_alpha), beta=float(c2_beta))

    def _mutate(self, individual: _GAIndividual) -> _GAIndividual:
        eta = 20.0
        mutationStep = float(self.config.gaMutationStep)
        mutated = _GAIndividual(alpha=individual.alpha, beta=individual.beta)
        if np.random.random() < self.config.gaMutationRate:
            u = np.random.random()
            delta = (2.0 * u) ** (1.0 / (eta + 1.0)) - 1.0 if u < 0.5 else 1.0 - (2.0 * (1.0 - u)) ** (1.0 / (eta + 1.0))
            mutated.alpha = float(np.clip(mutated.alpha + delta * mutationStep, self.ANGLE_MIN, self.ANGLE_MAX))
        if np.random.random() < self.config.gaMutationRate:
            u = np.random.random()
            delta = (2.0 * u) ** (1.0 / (eta + 1.0)) - 1.0 if u < 0.5 else 1.0 - (2.0 * (1.0 - u)) ** (1.0 / (eta + 1.0))
            mutated.beta = float(np.clip(mutated.beta + delta * mutationStep, self.ANGLE_MIN, self.ANGLE_MAX))
        return mutated

    def _elitePreserve(self, oldPop: List[_GAIndividual], newPop: List[_GAIndividual]) -> List[_GAIndividual]:
        eliteSize = max(1, int(self.config.gaPopulationSize * self.config.gaEliteRatio))
        elites = sorted(oldPop, key=self._getFitnessTuple)[:eliteSize]
        combined = elites + newPop
        if len(combined) > self.config.gaPopulationSize:
            combined = sorted(combined, key=self._getFitnessTuple)[:self.config.gaPopulationSize]
        return combined

    def _gaOptimize(self, originalMesh: trimesh.Trimesh) -> Tuple[float, float, float, float]:
        print(f"种群大小: {self.config.gaPopulationSize}")
        print(f"进化代数: {self.config.gaGenerations}")
        print(f"优化模式: {self.config.optimizationPriority} 优先")
        population = self._initializePopulation(originalMesh)
        self._evaluatePopulation(originalMesh, population)
        bestIndividual = min(population, key=self._getFitnessTuple)
        print(f"\n初代最优: alpha={bestIndividual.alpha:.2f}°, beta={bestIndividual.beta:.2f}°, "
              f"S1={bestIndividual.s1:.4f}, S2={bestIndividual.s2:.4f}")
        for generation in range(self.config.gaGenerations):
            offspring = []
            while len(offspring) < self.config.gaPopulationSize:
                parent1 = self._tournamentSelection(population)
                parent2 = self._tournamentSelection(population)
                child1, child2 = self._crossover(parent1, parent2)
                offspring.extend([self._mutate(child1), self._mutate(child2)])
            offspring = offspring[:self.config.gaPopulationSize]
            self._evaluatePopulation(originalMesh, offspring)
            population = self._elitePreserve(population, offspring)
            currentBest = min(population, key=self._getFitnessTuple)
            if self._lexicographicCompare(currentBest, bestIndividual):
                bestIndividual = currentBest
        print(f"\n总评估次数: {len(self.evalRows)}")
        print(f"最优解: alpha={bestIndividual.alpha:.2f}°, beta={bestIndividual.beta:.2f}°")
        print(f"最优目标值: S1={bestIndividual.s1:.4f}, S2={bestIndividual.s2:.4f}\n")
        return (float(bestIndividual.alpha), float(bestIndividual.beta),
                float(bestIndividual.s1), float(bestIndividual.s2))

    def writeCsvLog(self) -> None:
        if not self.config.logCsvPath:
            return
        with open(self.config.logCsvPath, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["alphaDeg", "betaDeg", "s1Area", "s2Area"])
            writer.writerows(self.evalRows)

    def optimize(self,
                 inputCasting: Union[str, trimesh.Trimesh],
                 outputCastingPath: Optional[str] = None) -> Dict:
        if isinstance(inputCasting, str):
            from geometryAdapters import loadMeshFromFile
            originalMesh = loadMeshFromFile(inputCasting)
        else:
            originalMesh = inputCasting.copy()
            originalMesh.fix_normals()
        originalMesh = self.placeOnBuildPlate(originalMesh)
        self.evaluateAngles(originalMesh, 0.0, 0.0)
        bestAlpha, bestBeta, bestS1, bestS2 = self._gaOptimize(originalMesh)
        self.bestAlpha, self.bestBeta, self.bestS1, self.bestS2 = bestAlpha, bestBeta, bestS1, bestS2
        optimizedCasting = self.rotateMesh(originalMesh, bestAlpha, bestBeta)
        moldGen = MoldGenerator(config=self.moldConfig)
        optimizedMold = moldGen.generateMoldShell(optimizedCasting)
        if outputCastingPath:
            from geometryAdapters import exportMeshToStl
            exportMeshToStl(optimizedCasting, outputCastingPath)
        if self.config.saveBestMoldPath:
            from geometryAdapters import exportMeshToStl
            exportMeshToStl(optimizedMold, self.config.saveBestMoldPath)
        self.writeCsvLog()
        return {
            "bestAlpha": bestAlpha,
            "bestBeta": bestBeta,
            "bestS1": float(self.bestS1) if self.bestS1 is not None else None,
            "bestS2": float(self.bestS2) if self.bestS2 is not None else None,
            "bestMesh": optimizedCasting,
            "bestMold": optimizedMold,
            "gaFinalAlpha": bestAlpha,
            "gaFinalBeta": bestBeta,
            "gaFinalS1": bestS1,
            "gaFinalS2": bestS2,
            "nEvaluations": len(self.evalRows),
            "logCsvPath": self.config.logCsvPath,
            "outputCastingPath": outputCastingPath,
            "optimizationPriority": self.config.optimizationPriority,
        }


def optimizeMoldOrientation(
        inputCasting: Union[str, trimesh.Trimesh],
        outputCastingPath: Optional[str] = None,
        seed: Optional[int] = 0,
        numSamples: int = 2000,
        raysPerPoint: int = 64,
        supportAngle: float = 45.0,
        layerHeight: float = 0.5,
        smoothHeight: float = 1.0,
        boundingBoxOffset: float = 5.0,
        booleanEngine: Optional[str] = None,
        s1Tolerance: float = 0.01,
        s2Tolerance: float = 0.01,
        angleQuantizationDeg: float = 0.05,
        logCsvPath: Optional[str] = "orientationEvalLog.csv",
        saveBestMoldPath: Optional[str] = None,
        gaPopulationSize: int = 30,
        gaGenerations: int = 25,
        gaMutationRate: float = 0.1,
        gaCrossoverRate: float = 0.8,
        gaTournamentSize: int = 3,
        gaEliteRatio: float = 0.1,
        gaMutationStep: float = 0.5,
        gaSeed: Optional[int] = None,
        optimizationPriority: str = "machining",
) -> Dict:
    config = OptimizerConfig(
        seed=seed, s1Tolerance=s1Tolerance, s2Tolerance=s2Tolerance,
        numSamples=numSamples, raysPerPoint=raysPerPoint, layerHeight=layerHeight,
        smoothHeight=smoothHeight, supportAngle=supportAngle, boundingBoxOffset=boundingBoxOffset,
        booleanEngine=booleanEngine, angleQuantizationDeg=angleQuantizationDeg,
        logCsvPath=logCsvPath, saveBestMoldPath=saveBestMoldPath,
        gaPopulationSize=gaPopulationSize, gaGenerations=gaGenerations,
        gaMutationRate=gaMutationRate, gaCrossoverRate=gaCrossoverRate,
        gaTournamentSize=gaTournamentSize, gaEliteRatio=gaEliteRatio,
        gaMutationStep=gaMutationStep, gaSeed=gaSeed, optimizationPriority=optimizationPriority,
    )
    optimizer = MoldOrientationOptimizer(config)
    return optimizer.optimize(inputCasting, outputCastingPath)


if __name__ == "__main__":
    results = optimizeMoldOrientation(
        inputCasting="testModels/hollow.cylinder.down.stl",
        outputCastingPath="cylinder.down.optimized.stl",
        saveBestMoldPath="cylinder.down.mold.optimized.stl",
        gaPopulationSize=15,
        gaGenerations=10,
        gaSeed=42,
        optimizationPriority="support"
    )
    print(f"\n最优角度: alpha={results['bestAlpha']:.2f}°, beta={results['bestBeta']:.2f}°")
    print(f"最优 S1: {results['bestS1']:.4f}")
    print(f"最优 S2: {results['bestS2']:.4f}")
    print(f"评估次数: {results['nEvaluations']}")
