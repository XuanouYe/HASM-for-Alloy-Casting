面向常温固化液态铟镓合金铸件后处理场景的五轴 CNC 刀具路径规划模块，支持从 STL 模型自动生成多工步 CL 点数据，并进一步后处理为 G-code。

项目当前围绕四个加工工步组织流程：壳体铣削、冒口去除、铸件精加工、浇口去除，并在统一的数据结构下输出带刀轴信息的五轴路径结果。

## 功能概述

- 支持读取 `part`、`mold`、`gate`、`riser` 四类 STL 模型并自动完成整套减材路径规划。
- 支持三类基础刀路策略：`zLevelRoughing`、`dropRaster`、`surfaceFinishing`。
- 支持离散候选刀轴生成与贪心覆盖选轴，用较少姿态覆盖复杂曲面。
- 支持局部坐标系加工，将任意刀轴方向统一转换为局部 Z 方向处理。
- 支持三层安全机制，包括碰撞检查、高度图限制和实体硬裁剪。
- 支持 VTK 可视化显示刀路，并高亮显示与指定碰撞模型相交的线段。
- 支持将 CL 点结果后处理为五轴 G-code，并根据刀轴向量计算回转轴角度。

## 适用场景

本项目适用于“3D 打印模具 + 液态金属浇铸 + 五轴 CNC 去壳/修整”的增减材复合制造流程，重点解决铸件外壳去除、冒口浇口切除和铸件表面精加工问题。

如果目标是快速验证五轴路径规划逻辑、工步组织方式、碰撞规避框架和后处理链路，这套代码已经具备完整原型能力。

## 项目结构

`textcnc/
├── interface.py            # 顶层接口，组织配置、调用主流程、可视化入口
├── pathGenerator.py        # 总控模块，工步装配、选轴、局部坐标变换、CL输出
├── toolpathStrategies.py   # 刀路策略实现
├── toolpathEngine.py       # 碰撞、避障、高度图、路径链接、IPW等公共能力
├── geometryUtils.py        # 几何工具函数
├── visualization.py        # VTK 可视化
└── gcodeProcessor.py       # CL JSON 到 G-code 的后处理`

上面的模块划分与当前代码职责是一一对应的，其中 `pathGenerator.py` 是核心协调器，`toolpathStrategies.py` 负责“怎么生成路径”，`toolpathEngine.py` 负责“怎么保证路径安全”。

## 整体流程

系统入口为 `generateCncJobInterface(...)`，它会从 `processConfig["subtractive"]` 中读取刀具直径、安全边距、步距、层深、候选刀轴数量和精加工参数等配置。

接口层随后构造 4 个工步参数和 Step3 的刀轴覆盖参数，并调用 `FiveAxisCncPathGenerator.generateJob(...)` 生成整套 CL 数据。

生成后的 CL 数据会被导出为 JSON；如果启用了 `visualize=True`，则还会调用 `PathVisualizer` 载入路径和碰撞检测模型进行显示。

如果继续调用后处理模块，`gcodeProcessor.py` 会读取 CL JSON，结合机床运动学配置，把每个 CL 点转换为 XYZ 直线轴和 A/B 回转轴指令，最终写出 G-code 文件。

## 四个工步

## Step1：壳体铣削

Step1 的目标网格是模具壳体 `moldMesh`，禁入和避让对象都是“铸件 + 浇口 + 冒口”的组合体，并启用了实体硬裁剪。

这一工步默认采用 `zLevelRoughing`，适合做外壳材料去除，并通过 `solidClipMesh` 保证刀路不会直接进入被保护实体内部。

## Step2：冒口去除

Step2 的目标网格是 `riserMesh`，保护对象是“铸件 + 浇口”，刀轴固定为 Z 方向。

接口默认把这一工步配置为 `dropRaster`，用于在自上而下投影条件下快速完成冒口区域去除。

## Step3：铸件精加工

Step3 的目标网格是 `partMesh`，`keepOutMesh` 为 `gateMesh`，同时分别沿 `x` 和 `y` 两个扫描方向各执行一次 `surfaceFinishing`，再把结果合并为最终精加工路径。

这一工步默认关闭 `enablePathLinking`，说明当前设计倾向于保留独立表面条带，而不是强行把所有条带连成一整条连续路径。

Step3 不使用 `solidClipMesh`，这是因为精加工需要允许刀具贴近甚至接触零件表面，不能像粗加工那样直接用“实体内部剔除”做硬裁剪。

## Step4：浇口去除

Step4 的目标网格是 `gateMesh`，保护对象和实体裁剪对象都是 `partMesh`。

该步默认仍采用 `dropRaster`，刀轴固定为 Z 方向，目的是切除浇口同时避免伤及成品铸件主体。

## 核心算法设计

## 1. 刀轴选择

候选刀轴默认由 `generateHemisphereAxes` 在半球上生成，并可通过 `axisCount` 和 `minAxisZ` 控制数量与姿态范围。

Step3 的选轴由 `selectAxesGreedyCoverage` 完成，它会先对零件表面采样法向，再用法向与候选刀轴的点积关系评估“该刀轴能覆盖哪些表面区域”，最后用贪心方式选出覆盖增量最大的几个姿态。

这种方法的目标是用尽量少的离散刀轴覆盖尽量多的有效表面区域。

## 2. 局部坐标系加工

对每个刀轴，系统都会构造一个旋转矩阵，把该刀轴对齐到局部坐标系的 Z 轴方向。

随后，目标网格、避让网格和裁剪网格都会一起旋转到该局部坐标系中，路径策略、碰撞检测和抬刀逻辑也都在这个坐标系下执行，最后再旋转回世界坐标系输出。

这种设计把五轴问题统一成了“局部 Z 向加工”问题，极大简化了策略复用和安全逻辑实现。

## 3. 路径生成策略

`ZLevelRoughingStrategy` 通过水平分层截面、轮廓偏置和扫描线填充生成粗加工路径。

`DropRasterStrategy` 在规则 XY 栅格上向下发射射线，取最高交点作为局部刀位基础，并通过法向过滤避免在不适合投影加工的面上生成路径。

`SurfaceProjectionFinishingStrategy` 先对目标表面采样接触点和法向，再沿法向偏置得到候选刀具中心点，随后进行可达性筛查、条带聚类、路径重建和必要的碰撞裁剪，最终形成精加工条带路径。

## 4. 三层安全机制

当前代码中的避障逻辑主要由三层组成：`SafeEnvelope`、障碍高度图和 `SolidKeepOutClipper`。

`SafeEnvelope` 通过障碍表面采样点构建 KDTree，并结合 `mesh.contains(...)` 完成带符号安全间隙判断，可用于点和线段级安全检查。

障碍高度图由 `buildObstacleHeightMapLocal(...)` 生成，它在局部坐标系下为每个 XY 位置记录一个“至少要高于多少 Z 才安全”的限制值。

`SolidKeepOutClipper` 则对路径进行密化采样并用实体包含判断做最终硬裁剪，适合处理“绝对不可进入”的禁区实体。

## 5. 路径链接

当工步启用 `enablePathLinking` 时，系统会先用最近邻贪心方式对多条子路径排序，再尝试构造安全的过渡段。

过渡段优先尝试直接连线，若不安全则自动切换为“抬刀 - 平移 - 下刀”的门形路径，并逐步提高抬刀高度直到通过安全验证。

## 输出数据

CL 输出采用 JSON 结构，每个点包含 `pointId`、`position`、`toolAxis`、`feedrate` 和 `segmentId`。

每个工步同时包含 `segments` 和 `clPoints` 两个字段，其中 `segments` 描述路径段元信息，`clPoints` 保存实际刀位点序列。

G-code 后处理阶段会按 `stepId` 和 `segmentId` 重组路径，并把 `toolAxis` 转换为机床回转角度后输出五轴联动指令。

## 快速开始

## 依赖

当前代码直接使用了 `numpy`、`scipy`、`trimesh`、`shapely`、`vtk`，并可选使用 `fcl` 作为更底层的碰撞相关依赖入口。

## 生成 CL JSON

下面是一个最小调用示例，接口与当前代码中的顶层调用方式一致。

`pythonfrom cnc.interface import generateCncJobInterface

processConfig = {
    "subtractive": {
        "toolDiameter": 6.0,
        "toolSafetyMargin": 0.5,
        "feedRate": 500.0,
        "stepOver": 1.2,
        "layerStepDown": 1.0,
        "safeHeight": 5.0,
        "axisMode": "hemisphere",
        "axisCount": 48,
        "minAxisZ": 0.02,
        "finishStepOver": 0.45,
        "finishProjectionStep": 0.25,
        "finishNormalAngleDeg": 95.0
    }
}

generateCncJobInterface(
    partStl="tempCncFiles/part.stl",
    moldStl="tempCncFiles/mold.stl",
    gateStl="tempCncFiles/gate.stl",
    riserStl="tempCncFiles/riser.stl",
    outputJsonPath="tempCncFiles/cncToolpath.json",
    processConfig=processConfig,
    jobId="JOB_TEST",
    visualize=True
)`

## 生成 G-code

如果需要从 CL JSON 进一步生成机床代码，可以调用后处理接口。

`pythonfrom cnc.gcodeProcessor import generateCncGcodeInterface

generateCncGcodeInterface(
    partStl="tempCncFiles/part.stl",
    moldStl="tempCncFiles/mold.stl",
    gateStl="tempCncFiles/gate.stl",
    riserStl="tempCncFiles/riser.stl",
    outputGcodePath="tempCncFiles/cnc.gcode",
    processConfig=processConfig,
    visualize=False
)`

## 可视化说明

调用顶层接口时传入 `visualize=True` 后，系统会使用 `PathVisualizer` 启动 VTK 窗口进行交互式查看。

可视化器支持按数字键切换工步显示，也支持按 `C` 键切换碰撞线段显示模式。

碰撞检测模式会对相邻 CL 点形成的线段逐段采样，只要采样点落入指定碰撞模型内部，就会把该线段单独高亮出来。

## 关键参数

## 通用参数

- `toolDiameter`：刀具直径，系统内部会使用其一半作为 `toolRadius`。
- `toolSafetyMargin`：安全边距，参与碰撞和高度图限制计算。
- `stepOver`：扫描步距，影响光栅路径和粗加工填充密度。
- `layerStepDown`：Z 层粗加工层深。
- `safeHeight`：安全抬刀高度基准。

## Step3 精加工相关参数

- `finishStepOver`：精加工条带间距。
- `finishProjectionStep`：条带内部点间距及密化步长参考值。
- `finishNormalAngleDeg`：判定表面是否对当前刀轴“可见”的法向角阈值。
- `finishSurfaceSampleCount`：目标表面采样点数量，影响精加工可达中心构建质量。
- `finishKeepOutSampleCount`：禁入区域采样点数量，影响近邻避障精度。
- `finishCollisionClearance`、`finishContactPatchRadius`、`finishLocalAllowance`：共同控制球头刀中心的局部碰撞判定和接触区域容差。
- `finishLineGapTolerance`：条带内部断裂与重新分段阈值。

## 刀轴选择参数

- `axisMode`：默认使用 `hemisphere` 生成候选刀轴。
- `axisCount`：候选刀轴数量。
- `step3AxisCount`：Step3 最多保留的刀轴数。
- `step3AxisSampleCount`：用于表面法向覆盖评估的采样点数。
- `step3MinNormalDot`：法向与刀轴点积下限，用来判断该姿态是否可加工该表面。
- `step3TargetCoverage`：贪心选轴目标覆盖率。
- `step3AxisDiversityDot`：刀轴姿态多样性阈值，避免姿态冗余。

## 当前实现特点

当前实现已经具备从 STL 到 CL JSON 再到 G-code 的完整链路，适合作为五轴刀路研究、工步验证和可视化调试的工程原型。

系统的优势在于模块划分清晰、参数化程度高、可视化与后处理链路完整，并且核心逻辑全部围绕三角网格展开，便于和增材制造及铸造几何数据直接对接。

## 已知局限

当前 Step3 精加工仍然属于“基于表面采样点 + 条带聚类 + 刀具中心重建”的近似方法，而不是成熟工业 CAM 中基于曲面参数空间或精确刀具包络的精加工内核。

碰撞与避障逻辑目前仍以采样判定、高度图和实体包含测试为主，虽然工程上已经具备较强约束能力，但还没有形成严格意义上的刀具 swept volume 连续碰撞求解链路。

后处理部分当前按 CL 点直接输出 G1 联动，不包含姿态平滑、回转轴运动代价优化、进给前瞻和 RTCP 强相关功能。

## 后续开发方向

- 引入更稳定的 Step3 曲面精加工方案，例如基于曲面分区、主方向场或参数域的贴面路径生成。
- 把当前近似避障逻辑升级为更严格的刀具实体连续碰撞检测，并真正打通现有代码里为 `fcl` 预留的能力入口。
- 将 `PointCloudIPW` 从粗略点云近似升级为更真实的体素、高度场或窄带 SDF 材料去除模型。
- 在后处理阶段增加刀轴姿态平滑、回转轴限位规避和机床运动学约束优化。
- 扩展可视化器，使其支持切削段/过渡段分色、刀轴显示、刀具实体显示和逐段回放。

## 参考入口

- 顶层接口：`generateCncJobInterface(...)`。
- 主规划器：`FiveAxisCncPathGenerator`。
- 策略工厂：`ToolpathStrategyFactory`。
- 安全引擎：`TrimeshToolpathEngine`、`SafeEnvelope`、`SolidKeepOutClipper`。
- 后处理入口：`generateCncGcodeInterface(...)`。