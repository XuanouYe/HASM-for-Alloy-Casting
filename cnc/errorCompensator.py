from typing import Any, Dict, List, Tuple


def linearInterpolate(table: List[List[float]], queryVal: float) -> float:
    if not table:
        return 0.0
    if queryVal <= table[0][0]:
        return table[0][1]
    if queryVal >= table[-1][0]:
        return table[-1][1]
    for idx in range(len(table) - 1):
        x0, y0 = table[idx]
        x1, y1 = table[idx + 1]
        if x0 <= queryVal <= x1:
            ratio = (queryVal - x0) / (x1 - x0)
            return y0 + ratio * (y1 - y0)
    return table[-1][1]


class GeometricErrorCompensator:
    def __init__(self, errorCfg: Dict[str, Any]):
        self.enabled = bool(errorCfg.get("enabled", False))

        linearAxisErrors = errorCfg.get("linearAxisErrors", {})
        self.xTable: List[List[float]] = linearAxisErrors.get("xTable", [])
        self.yTable: List[List[float]] = linearAxisErrors.get("yTable", [])
        self.zTable: List[List[float]] = linearAxisErrors.get("zTable", [])

        rotaryAxisErrors = errorCfg.get("rotaryAxisErrors", {})
        self.cEpsilonXTable: List[List[float]] = rotaryAxisErrors.get("cEpsilonXTable", [])
        self.cEpsilonYTable: List[List[float]] = rotaryAxisErrors.get("cEpsilonYTable", [])
        self.cEpsilonZTable: List[List[float]] = rotaryAxisErrors.get("cEpsilonZTable", [])
        self.aEpsilonXTable: List[List[float]] = rotaryAxisErrors.get("aEpsilonXTable", [])
        self.aEpsilonYTable: List[List[float]] = rotaryAxisErrors.get("aEpsilonYTable", [])
        self.aEpsilonZTable: List[List[float]] = rotaryAxisErrors.get("aEpsilonZTable", [])

        positionErrors = errorCfg.get("positionErrors", {})
        self.ocx: float = float(positionErrors.get("ocx", 0.0))
        self.ocy: float = float(positionErrors.get("ocy", 0.0))
        self.oax: float = float(positionErrors.get("oax", 0.0))
        self.oay: float = float(positionErrors.get("oay", 0.0))
        self.oza: float = float(positionErrors.get("oza", 0.0))

        squarenessErrors = errorCfg.get("squarenessErrors", {})
        self.scx: float = float(squarenessErrors.get("scx", 0.0))
        self.scy: float = float(squarenessErrors.get("scy", 0.0))
        self.sax: float = float(squarenessErrors.get("sax", 0.0))
        self.say: float = float(squarenessErrors.get("say", 0.0))

    def computeLinearAxisErrors(self, px: float, py: float, pz: float) -> Tuple[float, float, float]:
        deltaX = linearInterpolate(self.xTable, px) if self.xTable else 0.0
        deltaY = linearInterpolate(self.yTable, py) if self.yTable else 0.0
        deltaZ = linearInterpolate(self.zTable, pz) if self.zTable else 0.0
        return deltaX, deltaY, deltaZ

    def computeRotaryAxisErrors(self, aDeg: float, cDeg: float) -> Tuple[float, float, float, float, float, float]:
        epsilonXc = linearInterpolate(self.cEpsilonXTable, cDeg) if self.cEpsilonXTable else 0.0
        epsilonYc = linearInterpolate(self.cEpsilonYTable, cDeg) if self.cEpsilonYTable else 0.0
        epsilonZc = linearInterpolate(self.cEpsilonZTable, cDeg) if self.cEpsilonZTable else 0.0
        epsilonXa = linearInterpolate(self.aEpsilonXTable, aDeg) if self.aEpsilonXTable else 0.0
        epsilonYa = linearInterpolate(self.aEpsilonYTable, aDeg) if self.aEpsilonYTable else 0.0
        epsilonZa = linearInterpolate(self.aEpsilonZTable, aDeg) if self.aEpsilonZTable else 0.0
        return epsilonXc, epsilonYc, epsilonZc, epsilonXa, epsilonYa, epsilonZa

    def applyCompensation(self, px: float, py: float, pz: float, aDeg: float, cDeg: float) -> Tuple[float, float, float]:
        if not self.enabled:
            return px, py, pz

        deltaXlin, deltaYlin, deltaZlin = self.computeLinearAxisErrors(px, py, pz)
        epsilonXc, epsilonYc, epsilonZc, epsilonXa, epsilonYa, epsilonZa = self.computeRotaryAxisErrors(aDeg, cDeg)

        x0, y0, z0 = px, py, pz

        compensationX = -(deltaXlin + self.ocx) + y0 * (epsilonZc + self.say) - z0 * (epsilonYc + self.scx + epsilonZa)
        compensationY = -(deltaYlin + self.ocy + self.oax) + z0 * (epsilonXc + self.scy) - x0 * (epsilonZc + self.say)
        compensationZ = -(deltaZlin + self.oza) + x0 * (epsilonYc + self.scx + epsilonZa) - y0 * (epsilonXc + self.scy)

        return px + compensationX, py + compensationY, pz + compensationZ
