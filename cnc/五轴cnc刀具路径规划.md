# 五轴cnc刀具路径规划

## 一、模块概述

本模块面向常温固化液态铟镓合金铸件的增减材复合制造场景，实现从 STL 几何模型到五轴 CNC 切削器位（CL）点列的全自动路径规划。整体制造工序分为四个步骤：壳体铣削（Step1）、冒口去除（Step2）、铸件精加工（Step3）、浇口去除（Step4）。每一步均在五轴局部坐标系（刀轴对齐局部 Z 轴）下完成路径生成、碰撞安全裁剪与过渡链接，最终输出带刀轴向量的五轴 CL 点 JSON 文件。

---

## 二、核心数据结构

## 2.1 CL 点格式

每个输出点包含：`pointId`、`position [x, y, z]`（WCS）、`toolAxis [ax, ay, az]`（单位向量）、`feedrate`、`segmentId`。段（Segment）是同一刀轴方向下的连续刀路单元。

## 2.2 局部坐标系

对每个候选刀轴 a\mathbf{a}a，构造旋转矩阵 RRR 将 a\mathbf{a}a 对齐到 [0,0,1]T[0, 0, 1]^T[0,0,1]T。所有路径生成与碰撞检测均在局部坐标系下进行，最终用 RTR^TRT 变换回 WCS。这一设计使得任意刀轴方向都可以用标准的"Z 向"高度图和抬刀逻辑处理，而无需重写策略代码。cad-journal+1

---

## 三、刀轴选择算法

## 3.1 问题建模

五轴加工的核心挑战之一是在保证曲面完整覆盖的前提下，选取尽可能少的离散刀轴方向。本模块将此问题建模为**加权集合覆盖问题（Weighted Set Cover Problem）**：

- 宇宙集 UUU：从目标曲面均匀采样的法向量点集，共 ∣U∣|U|∣U∣ 个样本点。
- 候选集族 F\mathcal{F}F：每个候选刀轴 ai\mathbf{a}_iai 对应一个子集 Si⊆US_i \subseteq USi⊆U，其中 SiS_iSi 包含所有满足 nj⋅ai≥θmin\mathbf{n}_j \cdot \mathbf{a}_i \geq \theta_{min}nj⋅ai≥θmin 的采样点（θmin\theta_{min}θmin 为最小法向点积阈值，控制有效加工角度范围）。
- 目标：选取 kkk 个刀轴，使覆盖率 ∣Si1∪⋯∪Sik∣/∣U∣|S_{i_1} \cup \cdots \cup S_{i_k}| / |U|∣Si1∪⋯∪Sik∣/∣U∣ 最大化，直至达到目标覆盖率阈值（默认 99.5%）。

集合覆盖问题是 NP-Hard 的，但贪心近似算法可以在多项式时间内获得理论最优值 (1−1/e)≈63.2%(1 - 1/e) \approx 63.2\%(1−1/e)≈63.2% 的近似比。[[people.ucsc](https://people.ucsc.edu/~lhu82/Biobibnet/20IROS_3D.pdf)]

## 3.2 贪心覆盖算法（Greedy Coverage）

每次迭代选择新增覆盖点数最多的刀轴，并可选地施加**多样性约束**：若候选刀轴与已选刀轴的点积绝对值超过 `diversityDot`（默认 0.985），则跳过，以保证刀轴方向的几何多样性，避免冗余加工。若严格多样性约束导致无法选出新轴，则放宽约束重试。

该算法思想与 IROS 2020 论文 "Max Orientation Coverage" 中的刀轴覆盖贪心采样方案高度一致。[[people.ucsc](https://people.ucsc.edu/~lhu82/Biobibnet/20IROS_3D.pdf)]

---

## 四、路径生成策略

## 4.1 投影栅格（Drop Raster）

在局部坐标系下，对目标网格的 XY 包围盒按步距（stepOver）生成平行扫描线，每条扫描线沿 Z 方向向下投射射线，取与目标网格的最高交点作为刀位。这是 CNC 粗加工中最常用的基础策略，对于高度图连续的曲面效率高。incoherency+1

## 4.2 Z 层粗铣（Z-Level Roughing）

按固定 Z 步距（stepdown）水平分层，每层生成等高线轮廓并按步距偏移，用于大体积材料去除。结合在制工件（IPW）过滤器，可以跳过已经切除材料的区域，减少空切路径。[[youtube](https://www.youtube.com/watch?v=upmgqonjbrs)][[core.ac](https://core.ac.uk/download/pdf/11742121.pdf)]

## 4.3 曲面精加工（Surface Finishing）

沿扫描轴方向生成贴合目标曲面的密集刀路，同时沿两个正交扫描方向（X/Y）各生成一次以减少残留。精加工路径不做平台高度过滤，允许刀具沿曲面倾斜运动，充分利用五轴联动优势。sciencedirect+1

---

## 五、在制工件过滤（IPW）

## 5.1 算法思想

工业 CAM 系统（如 NX CAM、Mastercam）中的在制工件（In-Process Workpiece, IPW）机制用于追踪已切除的材料体积，避免刀具在空切区域做无效运动。core+2

本模块采用**点云近似 IPW**方案：从目标网格表面均匀采样大量点，维护一个"活跃掩码"。刀具通过某区域后，刀具半径范围内且在刀路 Z 高度以下的采样点被标记为"已切除"（活跃掩码置 False）。后续刀路生成时，若某段路径与活跃 IPW 点云的最近邻距离超过阈值，则该段被过滤掉，不输出。

这一方案以采样点云近似真实 IPW 几何体，在精度和计算效率之间取得平衡，适合增减材复合加工的在线路径决策场景。

---

## 六、碰撞安全体系

碰撞安全是本模块最核心的设计。采用三层递进式安全保障架构，职责明确、互不冗余：

## 6.1 第一层：有符号安全间隙（SafeEnvelope）

## 问题起源

朴素的"到障碍物表面的欧氏距离"判断方案存在根本性缺陷：**障碍物内部的点到表面的距离也是正数**，距离中心越远的点反而距离越大，会被误判为安全。arxiv+1

## 有符号距离场（Signed Distance Field, SDF）

本模块对障碍物网格维护一个近似有符号距离场：

ds(p)={+d(p,∂Ω)p∉Ω−d(p,∂Ω)p∈Ωd_s(\mathbf{p}) = \begin{cases} +d(\mathbf{p}, \partial \Omega) & \mathbf{p} \notin \Omega \\ -d(\mathbf{p}, \partial \Omega) & \mathbf{p} \in \Omega \end{cases}

ds(p)={+d(p,∂Ω)−d(p,∂Ω)p∈/Ωp∈Ω

其中 Ω\OmegaΩ 为障碍物实体，∂Ω\partial \Omega∂Ω 为其表面，d(p,∂Ω)d(\mathbf{p}, \partial \Omega)d(p,∂Ω) 为点 p\mathbf{p}p 到表面三角面片采样点集的最近邻距离（通过 cKDTree 高效查询），内外判断通过 `trimesh.contains` 的射线投票法实现。安全判定条件为 ds(p)≥rtool+δmargind_s(\mathbf{p}) \geq r_{tool} + \delta_{margin}ds(p)≥rtool+δmargin。acris.aalto+2

从障碍物表面均匀采样 80,000 个点构建 KD 树，用于快速近邻查询（O(log⁡n)O(\log n)O(logn) 每次查询）。内外判断的计算量相对较高，但只在过渡路径安全验证时调用，不在密集路径点上批量调用。

对于线段安全性检测，在线段上按自适应步长采样，步长为 min⁡(stepSize,0.4⋅clearance)\min(\text{stepSize}, 0.4 \cdot \text{clearance})min(stepSize,0.4⋅clearance)，保证不遗漏小于安全间隙的障碍特征。[[sciencedirect](https://www.sciencedirect.com/science/article/abs/pii/S0010448524000526)]

## 安全间隙中心与半径

用障碍物包围盒中心和对角线半径估算包围球，全局安全 Z 高度为：

Zsafe=zcenter+rbbox+rtool+δmargin+δextraZ_{safe} = z_{center} + r_{bbox} + r_{tool} + \delta_{margin} + \delta_{extra}

Zsafe=zcenter+rbbox+rtool+δmargin+δextra

这比直接用包围盒最高点更鲁棒，对任意刀轴方向的局部坐标系均适用。

## 6.2 第二层：障碍高度图（Obstacle Height Map）

## 算法思想

对旋转到局部坐标系的障碍物网格，沿局部 Z 轴方向进行光栅化投影，生成一张"障碍高度图"（Obstacle Height Map）。高度图的每个格点 (xi,yj)(x_i, y_j)(xi,yj) 存储该列的最高障碍高度加上安全间隙：

H(xi,yj)=max⁡k{zk∣ray(xi,yj) hits face k}+rtool+δmarginH(x_i, y_j) = \max_k \{z_k \mid \text{ray}(x_i, y_j) \text{ hits face } k\} + r_{tool} + \delta_{margin}

H(xi,yj)=kmax{zk∣ray(xi,yj) hits face k}+rtool+δmargin

刀位点 p\mathbf{p}p 的安全条件为：pz≥H(px,py)p_z \geq H(p_x, p_y)pz≥H(px,py)。

这一思想来自高度场 CNC 路径规划的经典方法：对任意 (x,y)(x, y)(x,y) 位置，高度图直接给出刀具必须保持在其上方的最低安全高度，查询复杂度为 O(1)O(1)O(1)（经双线性插值），生成时使用批量射线投射（ray casting）。[[incoherency.co](https://incoherency.co.uk/blog/stories/cnc-heightmap-toolpaths.html)]

高度图适合快速的"刀路逐点过滤"和"过渡路径安全评估"，但只对"从正上方投射"可见的障碍特征有效，对侧面遮挡需配合第一层 SDF 或第三层硬裁剪。

## 6.3 第三层：实体硬裁剪（SolidKeepOutClipper）

对于必须完全禁止刀具进入的实体区域（如 Step1 中的铸件本体，Step4 中的零件主体），在路径输出前执行基于 `trimesh.contains` 的实体硬裁剪：将路径按 `sampleStep` 密化后，对所有点做"内/外"判断，剔除所有在实体内部的点，将路径切分为若干互不穿越实体的子段。

这是工业 CAM 中"硬禁区（Hard Keep-Out Zone）"概念的直接实现，作为前两层的最终保险层，保证路径输出不含任何穿越禁区实体的段。[[cad-journal](https://www.cad-journal.net/files/vol_15/CAD_15(1)_2018_76-89.pdf)]

---

## 七、路径链接与过渡策略

## 7.1 贪心最近邻路径排序

对同一刀轴下生成的若干子路径段，采用**最近邻贪心算法（Nearest Neighbor Heuristic）**进行排序，以最小化路径间的过渡距离（等价于旅行商问题 TSP 的贪心近似解）：imada.sdu+1

从任意路径出发，每次选择当前末端与下一路径起点或终点距离最近的路径（若选终点则将该路径反向），直至所有路径排序完毕。该算法时间复杂度为 O(n2)O(n^2)O(n2)，对典型规模的刀路段数（数十至数百段）效率充分。

## 7.2 五轴友好过渡路径（Local-Z Retract）

五轴加工的一个关键需求是：过渡路径（抬刀-移位-下刀）**必须在局部坐标系下沿局部 Z 轴方向抬刀**，而不能使用全局 WCS 的 Z 轴。这是因为刀轴在任意空间方向时，沿 WCS Z 轴的移动对应的是刀具侧向运动，极易穿越障碍物。sciencedirect+2

本模块的 `_buildSafeTransitLocal` 方法在局部坐标系下实现三级递进过渡策略：

1. **直接连接**：若直接线段通过三层安全验证（SDF + 高度图 + 实体），直接输出。
2. **一级抬刀**：在局部 Z 方向抬刀至 ZsafeZ_{safe}Zsafe，形成"上升—平移—下降"的门形路径，验证安全后输出。
3. **二/三级加高**：若一级抬刀仍不安全（如两点位于障碍物两侧），依次将抬刀高度增加 2×clearance2 \times \text{clearance}2×clearance 和 4×clearance4 \times \text{clearance}4×clearance，每次验证后取第一个安全方案。
4. **极端抬刀**：所有级别均不满足时，使用 Zsafe+8×clearanceZ_{safe} + 8 \times \text{clearance}Zsafe+8×clearance 的极高抬刀保底。

过渡路径的安全验证通过 `_isTransitSafe` 统一调度三层安全检查，避免重复计算。

---

## 八、各工步配置

| 工步 | 目标网格 | keepOutMesh | avoidanceMesh | solidClipMesh | 说明 |
| --- | --- | --- | --- | --- | --- |
| Step1 壳体铣削 | 模具壳体 | 铸件+浇口+冒口 | 铸件+浇口+冒口 | 铸件+浇口+冒口 | 铣除外壳，严格禁止进入铸件区域 |
| Step2 冒口去除 | 冒口 | 铸件+浇口 | 铸件+浇口 | 铸件+浇口 | 单轴（Z向），保护铸件和浇口 |
| Step3 铸件精加工 | 铸件 | 浇口 | 铸件 | 无 | 多轴贴面加工，刀具需接触铸件表面，不设硬裁剪 |
| Step4 浇口去除 | 浇口 | 铸件 | 铸件 | 铸件 | 单轴（Z向），保护铸件主体 |

Step3 不设 `solidClipMesh` 是有意为之：精加工刀路需要贴合铸件表面运动，若设实体硬裁剪则会将所有接触面附近的路径点全部剔除。此情况下安全保障由第一层 SDF（`avoidanceMesh=partMesh`）的有符号间隙和高度图共同承担，确保刀路不穿越铸件内部，同时允许表面接触。

---

## 九、软件架构与依赖

`textpathGenerator.py          ← 顶层协调器，工步配置与 CL 输出
    └── toolpathEngine.py ← 核心算法引擎
            SafeEnvelope          : 有符号距离场 + KDTree
            SolidKeepOutClipper   : 实体硬裁剪
            MeshCollisionChecker  : 碰撞检查器（封装 SafeEnvelope）
            PointCloudIPW         : 在制工件点云模拟
            TrimeshToolpathEngine : 高度图、路径链接、过渡生成
    └── toolpathStrategies.py ← 路径生成策略（dropRaster / zLevelRoughing / surfaceFinishing）
    └── geometryUtils.py      ← 旋转矩阵、网格合并、法向量采样`

**主要依赖库：**

| 库 | 用途 | 文档 |
| --- | --- | --- |
| `trimesh` | 三角网格加载、布尔运算、射线投射、`contains` 判断 | [trimesh.org](https://trimesh.org/) trimesh+1 |
| `scipy.spatial.cKDTree` | 曲面采样点的 K 近邻搜索，用于 SDF 近似 | SciPy 官方文档 |
| `numpy` | 批量矩阵运算、坐标变换 | NumPy 官方文档 |
| `fcl`（可选） | BVH 加速的精确碰撞检测，在 `_FCL_AVAILABLE=True` 时启用 | [flexible-collision-library/fcl](https://github.com/flexible-collision-library/fcl) github+1 |

---

## 十、参考文献

**五轴路径规划与碰撞避免：**

1. Chichell, J.Z. et al. (2024). *Collision-free Tool Motion Planning for 5-Axis CNC Machining*. Computer-Aided Design. DOI: 10.1016/j.cad.2024.000526[[sciencedirect](https://www.sciencedirect.com/science/article/abs/pii/S0010448524000526)]
2. Chichell, J.Z. et al. (2025). *Evolution-based tool path and motion planning for 5-axis CNC machining of free-form surfaces*. Computer-Aided Design. DOI: 10.1016/j.cad.2025.001125[[sciencedirect](https://www.sciencedirect.com/science/article/pii/S0010448525001125)]
3. Konobrytskyi, D. et al. (2018). *5-Axis Tool Path Planning Based on Highly Parallel Discrete Volumetric Geometry Representation*. CAD Journal, 15(1), 76–89.[[cad-journal](https://www.cad-journal.net/files/vol_15/CAD_15(1)_2018_76-89.pdf)]
4. Hao, J. et al. (2022). *Partition-based 3+2-axis tool path generation for freeform surface machining*. Journal of Computational Design and Engineering, 9(5).[[academic.oup](https://academic.oup.com/jcde/article/9/5/1585/6656372)]
5. Chu, C.-H. et al. (2022). *Tool path planning for five-axis flank milling of spiral bevel gears*. Journal of Computational Design and Engineering, 9(5).[[academic.oup](https://academic.oup.com/jcde/article-pdf/9/5/2024/46596511/https.pdf)]

**刀轴方向选取与覆盖算法：**

1. Hu, L. et al. (2020). *Max Orientation Coverage: Efficient Path Planning to Avoid Collisions in Five-Axis Machining*. IROS 2020.[[people.ucsc](https://people.ucsc.edu/~lhu82/Biobibnet/20IROS_3D.pdf)]
2. 贪心集合覆盖近似算法：Chvátal, V. (1979). *A greedy heuristic for the set-covering problem*. Mathematics of Operations Research, 4(3), 233–235.

**有符号距离场与碰撞检测：**

1. Sitzmann, V. et al. (2020). *Implicit Neural Representations with Periodic Activation Functions*. NeurIPS 2020.（SDF 神经隐式表示的基础工作）
2. Preuss, T. et al. (2025). *Implicit Neural Field-Based Process Planning for Multi-Axis Manufacturing*. arXiv:2511.17578.[[arxiv](https://arxiv.org/html/2511.17578v1)]
3. Pan, J. et al. (2012). *FCL: A General Purpose Library for Collision and Proximity Queries*. ICRA 2012. GitHub: [flexible-collision-library/fcl](https://github.com/flexible-collision-library/fcl)[[github](https://github.com/flexible-collision-library/fcl)]
4. Trimesh 项目文档：`trimesh.proximity.signed_distance` API，[trimesh.org/trimesh.proximity.html](https://trimesh.org/trimesh.proximity.html)[[trimesh](https://trimesh.org/trimesh.proximity.html)]
5. 基于高度图的刀具路径生成：pngcam 开源项目，James Stanley (2020). *CNC Toolpath Generation from Heightmaps*. [incoherency.co.uk](https://incoherency.co.uk/blog/stories/cnc-heightmap-toolpaths.html)[[incoherency.co](https://incoherency.co.uk/blog/stories/cnc-heightmap-toolpaths.html)]

**在制工件（IPW）：**

1. Ozturk, E. & Budak, E. (2010). *Machining Strategy Development in 5-Axis Milling Operations Using Process Models*. Academia.[[academia](https://www.academia.edu/18666336/Machining_strategy_development_in_5_axis_milling_operations_using_process_models)]
2. NX CAM 在线教程：*In-Process Workpiece (IPW) in 5-Axis Roughing*. Siemens NX Manufacturing Blog, 2019.[[blogs.sw.siemens](https://blogs.sw.siemens.com/nx-manufacturing/whats-new-in-nx-for-manufacturing-january-2019/)]

**路径排序（TSP 贪心近似）：**

1. Rosenkrantz, D.J. et al. (1977). *An Analysis of Several Heuristics for the Traveling Salesman Problem*. SIAM Journal on Computing.
2. Daniells, L. (2020). *The Travelling Salesman Problem — Nearest Neighbor Heuristic*. STOR-i, Lancaster University.[[lancaster.ac](https://www.lancaster.ac.uk/stor-i-student-sites/libby-daniells/2020/04/21/the-travelling-salesman-problem/)]

**五轴进给率规划（扩展参考）：**

1. Liu, H. et al. (2022). *Optimal Feedrate Planning on a Five-Axis Parametric Tool Path with Constraints*. Journal of Computational Design and Engineering, 9(6).[[academic.oup](https://academic.oup.com/jcde/article/9/6/2355/6786273)]

**开源参考项目：**

- **HASM-for-Alloy-Casting**：本模块所在项目，[github.com/XuanouYe/HASM-for-Alloy-Casting](https://github.com/XuanouYe/HASM-for-Alloy-Casting)
- **trimesh**：Python 三角网格处理库，[github.com/mikedh/trimesh](https://github.com/mikedh/trimesh)
- **FCL（Flexible Collision Library）**：C++ 精确碰撞检测库，[github.com/flexible-collision-library/fcl](https://github.com/flexible-collision-library/fcl)[[github](https://github.com/flexible-collision-library/fcl)]
- **pngcam**：高度图 CNC 刀路生成开源工具，James Stanley，[github.com/jes/pngcam](https://github.com/jes/pngcam)[[incoherency.co](https://incoherency.co.uk/blog/stories/cnc-heightmap-toolpaths.html)]
- **NCG CAM**：商业五轴 CAM 软件，Z 层粗铣策略参考，[ncgraphics.de](https://www.ncgraphics.de/cm/en/features/183-raster-roughing.html)[[ncgraphics](https://www.ncgraphics.de/cm/en/features/183-raster-roughing.html)]