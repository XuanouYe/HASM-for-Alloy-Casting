# CNC 五轴加工模块 v4.0

面向铟镓合金铸件后处理的五轴 CNC 路径规划模块，基于严格 swept volume 碰撞检测，实现数学上保证无干涉的刀具路径。

## 核心架构

```
cnc/
├── toolModel.py          # 平面铣刀（FlatEndMillTool）几何建模
├── implicitGeometry.py   # SDF 体积（SdfVolume）隐式几何层
├── sweptCollision.py     # Swept Volume 连续碰撞检测引擎
├── toolpathEngine.py     # 路径引擎总线（IPW、旋转变换、SDF 工厂）
├── geometryUtils.py      # 几何工具函数
├── pathGenerator.py      # 五轴路径生成器（四工步主逻辑）
├── toolpathStrategies.py # ZLevelRoughing / DropRaster / IsoPlanar 策略
├── coveragePlanner.py    # 壳体轴覆盖规划
├── interface.py          # 统一调用入口
├── pathLinker.py         # CL 段链接与退刀插入
├── machineKinematics.py  # XYZAC-TRT 运动学正反解
├── gcodeProcessor.py     # G-code 后处理
└── visualization.py      # VTK 可视化
main.py                   # 入口脚本
```

## 重构要点（v4.0 vs v3.x）

### 1. 刀具建模替换

旧版（v3.x）使用球头刀假设，含 `finishContactPatchRadius`、`finishLocalAllowance`、`finishCollisionClearance` 等参数，已全部移除。

新版引入 `FlatEndMillTool`（`toolModel.py`），描述平面铣刀几何：

| 参数 | 说明 |
|------|------|
| `toolDiameter` | 刀具直径（mm） |
| `toolLength` | 刀刃有效长度（mm） |
| `shankDiameter` | 刀柄直径（mm） |
| `toolSafetyMargin` | 额外安全裕度（mm） |
| `sdfVoxelSize` | SDF 体素尺寸，控制碰撞精度（mm） |

### 2. 安全机制升级

旧版（v3.x）三层近似安全：`SafeEnvelope`（采样 KD-Tree）+ 障碍高度图 + `SolidKeepOutClipper`（点包含测试）。

新版统一为严格 SDF + Swept Volume 连续碰撞检测：

```
SdfVolume（implicitGeometry.py）
  └── 基于 trimesh 表面采样 + KD-Tree 近邻 + contains() 有符号距离

SweptVolumeCollisionEngine（sweptCollision.py）
  └── 对 CL 段 (p0,a0)→(p1,a1) 自适应细分
  └── 每子段对 FlatEndMillTool 表面采样点查询 SDF
  └── SDF < -clearance 即判定碰撞
```

误差上界：\`O(h + 刀具表面采样间距)\`，其中 h 为 SDF 体素尺寸（由 `sdfVoxelSize` 控制）。

已移除：`SafeEnvelope`、`buildObstacleHeightMapLocal`、`clipPathsByObstacleLocal`、`clipPathsByCollisionChecker`、`MeshCollisionChecker`。

### 3. 四工步安全配置

| 工步 | 目标 | 保护实体 |
|------|------|---------|
| Step1 shellRemoval | 去除塑料模壳 | SDF(part ∪ gate ∪ riser) |
| Step2 riserRemoval | 去除冒口 | SDF(part ∪ gate) |
| Step3 partFinishing | 铸件精加工 | SDF(offset(part, -finishStock)) |
| Step4 gateRemoval | 去除浇口 | SDF(part) |

Step3 使用 `buildOffsetSdf` 构造铸件内缩 SDF，平面铣刀不得穿入内缩偏置面。

### 4. 路径生成流程

```
strategy.generate() → 候选路径（局部坐标）
    ↓
SweptVolumeCollisionEngine.filterPaths()  ← 唯一安全决策点
    ↓
slicePathByPlatformZ()（非精加工）
    ↓
IPW 过滤 & 更新
    ↓
_emitSegments() → CL 点输出
```

## 快速使用

```python
from pathlib import Path
from cnc.interface import generateCncJobInterface

generateCncJobInterface(
    partStl="tempCncFiles/part.stl",
    moldStl="tempCncFiles/mold.stl",
    gateStl="tempCncFiles/gate.stl",
    riserStl="tempCncFiles/riser.stl",
    outputJsonPath="tempCncFiles/cncToolpath.json",
    processConfig={
        "subtractive": {
            "toolDiameter": 6.0,
            "toolLength": 24.0,
            "shankDiameter": 6.0,
            "toolSafetyMargin": 0.5,
            "sdfVoxelSize": 0.3,        # SDF 精度，越小越精确但越慢
            "feedRate": 500.0,
            "stepOver": 1.2,
            "layerStepDown": 1.0,
            "safeHeight": 5.0,
            "axisMode": "hemisphere",
            "axisCount": 48,
            "finishStepOver": 0.45,
            "finishStock": 0.03,
            "step3AxisCount": 16,
        }
    },
    jobId="JOB_001",
    visualize=True
)
```

随后调用 G-code 后处理：

```python
from cnc.gcodeProcessor import generateGcodeFromClJson

generateGcodeFromClJson(
    inputJsonPath="tempCncFiles/cncToolpath.json",
    processConfig={...},
    outputGcodePath="tempCncFiles/cnc.gcode"
)
```

## 参数调优指南

### 精度与性能权衡

| 参数 | 减小 | 增大 |
|------|------|------|
| `sdfVoxelSize` | 碰撞检测更精确，速度更慢 | 速度更快，可能漏检薄特征 |
| `sweepTol`（自动 = sdfVoxelSize） | 细分更密，安全性↑，速度↓ | 细分更稀，速度↑ |
| `toolSafetyMargin` | 允许路径更靠近工件 | 路径更保守，覆盖率可能下降 |

推荐初始值：`sdfVoxelSize = toolDiameter * 0.05`（直径 6mm 刀具 → 0.3mm）。

### 依赖项

```
numpy scipy trimesh shapely scipy.spatial.transform
```

可选（VTK 可视化）：
```
vtk
```

## 文件变更记录（v3.x → v4.0）

| 操作 | 文件 | 说明 |
|------|------|------|
| 新增 | `toolModel.py` | FlatEndMillTool |
| 新增 | `implicitGeometry.py` | SdfVolume, buildSdfVolume, buildOffsetSdf |
| 新增 | `sweptCollision.py` | SweptVolumeCollisionEngine |
| 重构 | `toolpathEngine.py` | 移除 SafeEnvelope/HeightMap/SolidClipper，新增 SDF 工厂方法 |
| 重构 | `pathGenerator.py` | generateStepWithAxes 改为 SweptVolumeCollisionEngine 过滤 |
| 重构 | `interface.py` | 参数替换为平面铣刀参数，移除球头刀参数 |
| 重构 | `geometryUtils.py` | 新增 generateHemisphereAxes/generateSphereAxes/deduplicateAxes |
| 不变 | `toolpathStrategies.py` | 策略层无刀具类型依赖，不变 |
| 不变 | `pathLinker.py` | 链接逻辑不变 |
| 不变 | `machineKinematics.py` | 运动学不变 |
| 不变 | `gcodeProcessor.py` | 后处理不变 |
| 不变 | `visualization.py` | 可视化不变 |
| 不变 | `coveragePlanner.py` | 壳体规划不变 |
