# CNC 模块 README

## 模块概述

`cnc/` 模块是面向铟镓合金铸造工件的**五轴数控铣削后处理系统**，负责从 STL 几何模型出发，自动生成多工步刀路、连接段、五轴运动学变换，并最终输出标准 G-code 文件。整个流程分为路径规划、碰撞过滤、路径连接、运动学后置四个核心阶段。

## 文件结构

```
cnc/
├── __main__.py            # 模块命令行入口
├── interface.py           # 对外统一调用接口
├── pathGenerator.py       # 五轴刀路主生成器
├── toolpathStrategies.py  # 刀路策略（粗/精加工）
├── toolpathEngine.py      # 刀路引擎（IPW、裁剪）
├── coveragePlanner.py     # 遮挡可达性覆盖规划
├── pathLinker.py          # 刀路段间连接生成
├── gcodeProcessor.py      # G-code 后置处理器
├── machineKinematics.py   # XYZAC 五轴运动学解算
├── sweptCollision.py      # 扫掠体碰撞检测引擎
├── implicitGeometry.py    # SDF 隐式几何体封装
├── sdfBackend.py          # SDF 计算后端（voxel）
├── geometryUtils.py       # 通用几何工具函数
└── toolModel.py           # 平底立铣刀几何模型
```

***

## 各模块详解

### `pathGenerator.py` — 五轴刀路主生成器

**核心类：** `FiveAxisCncPathGenerator`

整个 CNC 加工任务的顶层调度器，接受零件、模具壳体、浇口、冒口四组 STL 文件，按顺序生成四个工步的刀位点数据（CL Data），并输出为 JSON。

**四个工步：**

| 工步 ID | 类型 | 目标 | 碰撞保护对象 |
|---|---|---|---|
| 1 | `shellRemoval` | 铣削模具外壳 | 零件 + 浇口 + 冒口 |
| 2 | `riserRemoval` | 去除冒口 | 零件 + 浇口 |
| 3 | `partFinishing` | 零件表面精修 | 零件偏置体（留量） |
| 4 | `gateRemoval` | 去除浇口 | 零件 |

**关键算法：贪心覆盖轴选（`selectAxesGreedyCoverage`）**

用于工步 3 精加工的轴方向自动选取：
1. 在目标网格上采样 `sampleCount` 个点及其法向量；
2. 对每个候选加工轴，统计法向量点积 ≥ `minNormalDot` 的采样点集合（即该轴能覆盖的面积）；
3. 采用**贪心集合覆盖**迭代：每轮选出能新增覆盖最多未覆盖点的轴，加入轴集合；
4. 同时通过 `diversityDot` 约束确保相邻轴方向足够多样；
5. 当累计覆盖率 ≥ `targetCoverage`（默认 99.5%）或达到 `maxAxisCount` 时终止。

**CL 点结构（`buildClPointDicts`）：**
每个刀位点包含 `position`（三维坐标）、`toolAxis`（单位刀轴方向）、`feedrate`（进给速度）、`segmentId`（所属段 ID）。

**SDF 过滤（`_filterClPointsByPartSdf`）：**
加工完成后对所有 CL 点查询零件 SDF，将刀具中心落入零件体内（SDF < `-safeClearance`）的点及对应段移除，防止过切。

***

### `toolpathStrategies.py` — 刀路策略

**抽象接口：** `IToolpathStrategy.generate()`

通过工厂类 `ToolpathStrategyFactory.getStrategy(mode)` 按模式名返回对应策略实例。

#### `ZLevelRoughingStrategy`（水平层切粗加工，`zlr`）

1. 在目标网格 Z 方向范围内，按 `layerStep` 步长生成水平切平面序列；
2. 对每层截面用 Shapely 做布尔运算：可加工区域 = 目标截面向内偏置 `toolRadius + roughStock`，再减去 keepOut 区域的 `toolRadius + safetyMargin` 膨胀体；
3. 在安全多边形内按 `stepOver` 步距扫线（zigzag 双向），提取线段交集作为刀路。

#### `DropRasterStrategy` / `SurfaceProjectionFinishingStrategy`（投影光栅，`dropraster` / `spf`）

共用核心函数 `generateDropCutterPaths`，实现**Drop Cutter（最高点投影）** 算法：

1. 建立目标网格和 keepOut 网格的**离散 Z 高度图**（`buildZMap`）：在 XY 平面均匀栅格上记录各列最大 Z 值；
2. 对目标高度图用**半球形卷积核**（`nd.grey_dilation`）模拟球头/平底刀接触，得到刀具中心高度图 `clTarget`；
3. 对 keepOut 高度图用更大半径核得到 `clKeepOut`，取二者最大值 `clFinal` 作为最终刀具 Z 坐标；
4. 沿 X 或 Y 方向按 `stepOver` 扫描线提取 3D 刀路点，仅保留目标足迹范围内且刀具高度 ≥ keepOut 约束的点；
5. `SurfaceProjectionFinishingStrategy` 与 `IsoPlanarPatchFinishingStrategy` 均调用同一函数，仅 `finishStock` 参数不同。

***

### `toolpathEngine.py` — 刀路引擎

**类：** `TrimeshToolpathEngine`、`PointCloudIPW`

#### `PointCloudIPW`（在制品点云，In-Process Workpiece）

追踪当前剩余材料状态，避免精加工重复切削已去除区域：

1. 初始化时对目标网格采样点云；
2. `filterPathsLocal`：用 KD 树查询，只保留距未去除材料点 ≤ `toolRadius + stepOver * 1.5` 的刀路段；
3. `updateIpwLocal`：刀路执行后，将刀具半径内 XY 投影覆盖、Z 高度满足的点标记为已去除（`activeMask = False`）。

#### `TrimeshToolpathEngine`

提供 `clipWcsPathsByZ`（Z 平面裁剪）和 `slicePathByPlatformZ`（平台 Z 过滤）两个辅助方法，确保刀具不低于工作台安全面。

***

### `coveragePlanner.py` — 遮挡可达性覆盖规划

**类：** `ShellCoveragePlanner`

专用于模具外壳铣削的**射线可达性分析**，动态推荐加工轴方向：

1. 在壳体网格上均匀采样点及法向，维护 `coveredMask` 追踪覆盖状态；
2. **`evaluateAxis`**：对每个候选轴方向，从采样点沿轴反向发射射线，判断最近碰撞点是否为目标面本身（即可直接到达），统计可访问点数量及侧面权重作为评分；
3. **`buildLocalAxesFromUncovered`**：对未覆盖点中侧面朝向强（Z 分量小）的法向量聚类，生成局部候选轴；
4. **`suggestAxes`**：综合全局半球轴 + 局部轴 + 已有轴，排除与已选轴过近的候选，返回评分最高的 TopK 方向；
5. **`updateCoverageByPath`**：路径执行后，将路径点的 `toolRadius` 邻域内满足轴向可达的采样点标记为已覆盖。

***

### `pathLinker.py` — 刀路段间连接生成

**类：** `ClPathLinker`

在相邻刀路段之间插入安全连接运动点，分三级策略：

**连接分类（`classifyLink`）：**

| 级别 | 触发条件 | 连接方式 |
|---|---|---|
| 0（直连） | 距离 ≤ `directLinkThreshold` 且轴角变化 ≤ `rotationChangeThreshold` | 不插入额外点 |
| 1（局部抬刀） | 距离 ≤ `maxRetractOffset` 且轴角变化小 | 抬至局部 safeZ 后平移 |
| 2（全局抬刀） | 超出 Level 1 条件 | 抬至全局 clearanceZ 后平移 |
| 3（旋转换轴） | 轴向变化 ≥ `rotationRetractAngle` | 双倍距离抬刀 + 换轴 + 接近 |

**回退方向解算（`resolveOutwardAxisMultiRay`）：**

当 `retractAlongAxis = True` 时，沿刀轴方向抬刀需确定是正向还是反向：
1. 从当前点向轴正方向和反方向各发射一条射线，统计与工件网格的交点奇偶性（**奇偶规则**：奇数次 = 在体内）；
2. 若正向在体外（偶数），则正向为抬刀方向；若无法判断则退化到 SDF 梯度方向；
3. 结果缓存至 `_retractCache`，避免重复计算。

**安全回退长度（`_safeRetractLength`）：**
向抬刀方向发射射线，若遇到网格遮挡则回退长度缩短为交点距离减 0.5 mm，防止穿出另一侧。

***

### `gcodeProcessor.py` — G-code 后置处理器

**主函数：** `generateCncGcodeInterface`、`generateGcodeFromClJson`

将 CL Data JSON 转换为标准 G-code 文件的完整流水线：

```
CL JSON → ClPathLinker（插入连接点）→ XyzacTrtKinematics（运动学转换）→ G-code 输出
```

**`generateGcode` 逻辑：**
1. 按工步 ID 排序遍历所有 CL 点；
2. 对每个点调用运动学解算得到 `(X, Y, Z, A, C)` 五轴坐标；
3. 与前一点比较，仅输出有变化的轴地址（`axisOutputEps` 阈值去重）；
4. 运动类型映射：`cut` → `G1`，`rapid/retract/approach` → `G0`；
5. 进给速度仅在变化时输出（`feedrateMergeStrategy`）；
6. 程序头包含单位、绝对坐标、WCS、主轴转速、冷却液代码；程序尾含 `M5 M9 M30`。

***

### `machineKinematics.py` — XYZAC 五轴运动学解算

**类：** `XyzacTrtKinematics`（工作台旋转 + 刀具倾斜型，Table-Rotary-Tilt 结构）

**正运动学解（`solveRotaryAngles`）：**

给定工件坐标系中刀轴方向向量 \((d_x, d_y, d_z)\)，求旋转轴角度：

\[A = \arccos(d_z), \quad C = \text{atan2}(d_x, -d_y)
\]

奇异点（`sinA ≈ 0`，刀轴垂直）时 C 轴保持前值不变。

**线性轴解算（`solveLinearAxes`）：**

构建绕 A 轴再绕 C 轴的齐次变换矩阵链（含工件偏置 `workOffset` 和 A 轴偏置 `aAxisOffset`），对刀尖 WCS 坐标施加逆变换，得到机床 XYZ 坐标。

**最小角跳动优化（`minimizeAngularJump`）：**

枚举 A 轴正反解和 C 轴 ±360°/±720° 候选组合，选择同时满足：
- 与前一点 A、C 变化量最小；
- 刀轴方向误差（转为角度惩罚项 × 1000）最小；

的最优解，消除机床奇异翻转和多圈绕行。

***

### `sweptCollision.py` — 扫掠体碰撞检测引擎

**类：** `SweptVolumeCollisionEngine`

使用 SDF 体素场对刀具沿路径段的扫掠体进行碰撞检测：

1. 初始化时对 `FlatEndMillTool` 几何体在本地坐标系采样离散点云（`diskCount × ringCount`）；
2. **`_checkPoseBatch`**：批量计算多个刀具姿态（位置 + 旋转矩阵）下，所有采样点的 SDF 值，返回每个姿态的最小 SDF（+ clearance）；
3. **`checkSegment`（自适应细分）**：对路径段 `[p0→p1]` 采用递归二分法：
   - 端点 SDF > `safeBuffer` 则跳过；
   - 否则沿段插值采样，若最小 SDF < 0 则碰撞；
   - 若段长 ≤ `tol` 且轴角变化 ≤ 3° 则终止细分；
   - 最大递归深度 `maxSubdivDepth = 8`；
4. **`filterPaths`**：逐段检查，碰撞点处截断路径，返回安全子段列表。

刀轴插值使用**球面线性插值 SLERP**（`_slerpAxis`），保证轴向过渡平滑。

***

### `implicitGeometry.py` & `sdfBackend.py` — SDF 隐式几何

**`SdfVolume`** 封装网格的有符号距离场（Signed Distance Field）：
- 内部点返回负值，外部点返回正值；
- 通过 `createSdfBackend` 选择计算后端（`auto` 模式自动选择 pysdf / trimesh proximity）；
- `buildOffsetSdf`：将网格顶点沿法向偏移 `-offsetDist` 生成偏置体的 SDF，用于精加工留量控制。

***

### `geometryUtils.py` — 通用几何工具

| 函数 | 功能 |
|---|---|
| `applyRotation` | 齐次变换矩阵作用于点云 |
| `buildRotationFromTo` | 构造从向量 A 到向量 B 的最短旋转矩阵（4×4） |
| `generateHemisphereAxes` | 黄金角螺旋采样半球均匀轴向 |
| `generateSphereAxes` | 全球面均匀轴向采样 |
| `deduplicateAxes` | 按角度阈值去除近似重复轴向 |
| `densifyPolyline` | 对折线插值加密至指定步长 |
| `splitPolylineByGap` | 按最大间距将折线分段 |
| `nearestDistanceToPath` | KD 树加速的点到路径最近距离 |

半球轴向采样使用**黄金角螺旋**（`φ = π(3 - √5)`）保证均匀分布，同时保证 `[0,0,1]` 轴一定存在。

***

### `toolModel.py` — 立铣刀几何模型

**类：** `FlatEndMillTool`

平底立铣刀几何定义，提供 `sampleToolSurfaceLocal` 方法：在刀具本地坐标系（刀轴沿 +Z）上，按 `diskCount`（轴向层数）× `ringCount`（径向环数）均匀采样刀刃面及刀柄的离散点集，供碰撞检测使用。

***

## 整体数据流

```
STL 文件（零件 / 模具壳 / 浇口 / 冒口）
        │
        ▼
FiveAxisCncPathGenerator.generateJob()
        │  ① 贪心覆盖轴选（selectAxesGreedyCoverage）
        │  ② 各轴方向刀路策略生成（ZLevelRoughing / DropRaster / SPF）
        │  ③ SweptVolumeCollisionEngine 碰撞过滤
        │  ④ PointCloudIPW 在制品过滤
        │  ⑤ SDF 过切过滤（_filterClPointsByPartSdf）
        ▼
CL Data JSON（刀位点 + 工步结构）
        │
        ▼
ClPathLinker.processClData()
        │  ① 三级连接分类（直连 / 局部抬刀 / 全局抬刀 / 旋转换轴）
        │  ② 安全回退方向解算（射线奇偶 + SDF 梯度）
        ▼
Linked CL Data
        │
        ▼
generateGcode()
        │  XyzacTrtKinematics.convertPoint()
        │  → 运动学解算 (A, C) + 最小角跳动优化
        │  → 线性轴 (X, Y, Z) 求解
        ▼
G-code 文件（.gcode）
```