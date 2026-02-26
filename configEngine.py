import copy
import time
from typing import Any, Dict, Optional
from dataModel import ConfigSnapshot, PathsSnapshot
from controlConfig import parameterSchema


_BUILTIN_DEFAULTS: Dict[str, Any] = {}
for _section, _params in parameterSchema.items():
    _BUILTIN_DEFAULTS[_section] = {k: v["default"] for k, v in _params.items() if "default" in v}

_BUILTIN_DEFAULTS.setdefault("paths", {
    "partStlTemplate": "{workspaceDir}/part.normalized.stl",
    "moldShellStlTemplate": "{workspaceDir}/moldShell.normalized.stl",
    "gatingStlTemplate": "{workspaceDir}/gating.normalized.stl",
    "fdmGcodeTemplate": "{workspaceDir}/fdm.gcode",
    "cncClJsonTemplate": "{workspaceDir}/cnc.cl.json",
    "jobJsonTemplate": "{workspaceDir}/job.json",
})

_PROFILES: Dict[str, Dict[str, Any]] = {
    "default": {},
    "fast": {
        "additive": {"layerHeight": 0.3, "printSpeed": 80},
    },
    "quality": {
        "additive": {"layerHeight": 0.1, "printSpeed": 30},
    },
}


def _deepMerge(base: Dict, override: Dict) -> Dict:
    result = copy.deepcopy(base)
    for k, v in override.items():
        if isinstance(v, dict) and isinstance(result.get(k), dict):
            result[k] = _deepMerge(result[k], v)
        else:
            result[k] = copy.deepcopy(v)
    return result


def buildSnapshot(jobOverrides: Dict[str, Any], profileName: str = "default",
                  workspaceDir: str = "", jobId: str = "") -> ConfigSnapshot:
    profile = _PROFILES.get(profileName, {})
    merged = _deepMerge(_BUILTIN_DEFAULTS, profile)
    merged = _deepMerge(merged, jobOverrides)

    pathsSection = merged.get("paths", {})

    def resolvePath(template: str) -> str:
        return template.replace("{workspaceDir}", workspaceDir).replace("{jobId}", jobId)

    paths = PathsSnapshot(
        workspaceDir=workspaceDir,
        partStl=resolvePath(pathsSection.get("partStlTemplate", "")),
        moldShellStl=resolvePath(pathsSection.get("moldShellStlTemplate", "")),
        gatingStl=resolvePath(pathsSection.get("gatingStlTemplate", "")),
        fdmGcode=resolvePath(pathsSection.get("fdmGcodeTemplate", "")),
        cncClJson=resolvePath(pathsSection.get("cncClJsonTemplate", "")),
        jobJson=resolvePath(pathsSection.get("jobJsonTemplate", "")),
    )

    snapshot = ConfigSnapshot(
        version=str(int(time.time())),
        profileName=profileName,
        mold=merged.get("mold", {}),
        additive=merged.get("additive", {}),
        casting=merged.get("casting", {}),
        subtractive=merged.get("subtractive", {}),
        fdm=merged.get("fdm", {}),
        paths=paths,
    )
    return snapshot


def get(key: str, jobContext) -> Any:
    snapshot = jobContext.configSnapshot
    for section in ["mold", "additive", "casting", "subtractive", "fdm"]:
        sectionData = getattr(snapshot, section, {})
        if key in sectionData:
            return sectionData[key]
    return None


def getSection(sectionName: str, jobContext) -> Dict[str, Any]:
    snapshot = jobContext.configSnapshot
    if sectionName == "paths":
        return snapshot.paths.toDict()
    return getattr(snapshot, sectionName, {})
