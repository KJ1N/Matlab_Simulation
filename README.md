# MATLAB Route Planning and Cam Profile Design Toolkit
# MATLAB 路径规划与凸轮轮廓设计工具包

[English](#english) | [简体中文](#简体中文)

---

## English

### 1. Overview

This MATLAB package implements a route-design and mechanical cam-profile workflow. It uses Bézier curves to smooth a route through specified waypoints, evaluates the motion of a vehicle-like mechanism (including wheel and sensor positions), estimates steering-related quantities, and derives theoretical and virtual cam profiles. The main optimization script iteratively adjusts route control points against a set of geometric and kinematic criteria.

Coordinates and mechanical dimensions are expressed in **millimetres (mm)**.

### 2. Main workflow

1. **Configure the route and mechanism** in `route_design_AI.m`: waypoint coordinates, venue bounds, wheel offsets, sensor offset, initial direction, and cam parameters.
2. **Generate or resume route control points**. The script initializes control points if no matching `.mat` state file exists; otherwise, it loads the existing state.
3. **Smooth and sample the route** using `Bezier_superposition.m` and `Bezier_curve.m`.
4. **Simulate route geometry and steering** using `Simulation_cal.m` and `all_routes_cal.m`.
5. **Calculate the cam profile and its constraints** using `cam_cal_elliptic.m`.
6. **Optimize the route** by evaluating perturbed control-point candidates in parallel and updating the best candidate.
7. **Export a cam point set** by running `cam_calculat.m`. The output format is controlled by the `format` variable.


### 3. File guide

| File | Purpose |
|---|---|
| `route_design_AI.m` | Main route optimization script. Configures waypoints and mechanical parameters, evaluates candidate routes, updates control points, and saves optimization results. |
| `cam_calculat.m` | Loads saved optimization results, calculates cam profiles, exports cam coordinates, and visualizes the route and cam geometry. |
| `Simulation_cal.m` | Evaluates route geometry, path length, smoothness, sensor deviation, and steering-angle variation. |
| `Bezier_curve.m` | Generates Bézier curves from control points. |
| `Bezier_superposition.m` | Constructs smooth composite trajectories by blending multiple Bézier curve segments. |
| `cal_YangHui.m` | Calculates binomial coefficients used in Bézier curve generation. |
| `cal_endpoints.m` | Estimates endpoint derivatives using finite-difference methods. |
| `all_routes_cal.m` | Calculates vehicle-body, wheel, and sensor trajectories, as well as steering angles. |
| `cam_cal_elliptic.m` | Generates theoretical and virtual cam profiles and evaluates curvature and pressure-angle constraints. |
| `Curvature_radius.m` | Calculates the circumcircle radius defined by three points. |
| `distance_calculate.m` | Evaluates deviations between the sensor trajectory and target waypoints. |
| `Point_line_distance.m` | Calculates the perpendicular distance from a point to a line. |
| `Squares_standard_sum.m` | Computes an aggregate deviation metric based on squared values. |
| `score_cal.m` | Calculates the overall optimization score using configurable evaluation metrics. |
| `Optimize_P0.m` | Adjusts interior control points to improve trajectory smoothness. |
| **`Trajectory_A.mat`** | Saved optimization state corresponding to **Trajectory_A presented in the paper**, including optimized control points and optimization-related variables. |
| **`Trajectory_A_max.mat`** | Saved simulation and optimization results for **Trajectory_A presented in the paper**, used for subsequent trajectory analysis and cam-profile generation. |
| **`TrajectoryA.txt`** | Exported cam-profile coordinates corresponding to **Trajectory_A presented in the paper**. Contains 3D point coordinates (X, Y, Z) for importing the cam profile into SOLIDWORKS. |

**Note:** The three files `Trajectory_A.mat`, `Trajectory_A_max.mat`, and `TrajectoryA.txt` are the saved optimization and cam-profile output files associated with **Trajectory_A in the accompanying research paper**. They allow users to inspect the saved optimization state, analyze the resulting trajectory, and reconstruct the corresponding cam profile in SOLIDWORKS.


### 4. How to run

#### A. Optimize a route

1. Extract the ZIP archive to a writable folder.
2. Open MATLAB and set the Current Folder to the extracted `matlab_tulun` directory.
3. Open `route_design_AI.m`.
4. Review and edit the configuration section near the top, especially:
   - `data_name`: state-file prefix, e.g. `Trajectory_A` (the case included with this package).
   - `configuration`: selects which saved MAT file/configuration to load.
   - `P`: ordered route waypoints.
   - `Venue_size`: plot/venue bounds.
   - `driving_wheel`, `driven_wheel`, `Z_sensor`, `V_0`: mechanism offsets and initial direction.
   - `d_driving`, `i_total`, `c`, `thickness`, `d`, `r`: cam/mechanical parameters.
   - `traing_times`, `num`, `route_index`, `learning_efficiency`, `gradient_power0`: optimization settings.
5. Run the script. It displays route/cam plots and saves `.mat` state files as it iterates.

**Important:** the script loads existing MAT files when present. To start a fresh case, use a new `data_name` or back up and deliberately remove/rename the relevant state files. Do not overwrite valuable optimization results unintentionally.

#### B. Export a cam profile

1. Open `cam_calculat.m`.
2. Set `data_name` to the case to export. The provided script defaults to `Trajectory_A` and loads `Trajectory_A_max.mat`.
3. Set `save_name` and `format`:
   - `.ibl`: writes an IBL-style point/curve file.
   - `.txt`: writes tab-separated X/Y/Z coordinates, with Z set to 0.
   - `.scr`: writes a simple CAD `LINE` script.
4. Run the script. The exported file is written to the current working directory.

### 5. Building a cam model in SOLIDWORKS

The exported `TrajectoryA.txt` file contains cam-profile points in `X Y Z` format (`Z = 0`). These points can be used to create a planar cam curve in SOLIDWORKS:

1. Open `cam_calculat.m`, set `format` to `.txt`, and run the script to export the coordinates.
2. In SOLIDWORKS, import the text file using a curve-import command such as **Insert > Curve > Curve Through XYZ Points** (menu names may vary by version).
3. Verify the column format, units (**mm**), and resulting curve geometry. Adjust the text delimiter if required by your SOLIDWORKS version.
4. Use the imported curve to construct a suitable sketch and solid feature, then check the final cam geometry.

**Note:** Importing the point file creates a curve, not a finished cam part. Validate the profile and mechanical constraints before fabrication.

### 6. Requirements and compatibility notes

- MATLAB is required.
- `route_design_AI.m` uses `parfor`; a compatible Parallel Computing Toolbox/license is typically required to execute that section as written. If unavailable, consider replacing `parfor` with `for` for serial execution, subject to performance and MATLAB-version compatibility.
- The scripts use interactive plotting and `pause`, so execution may stop for visual inspection during the run.
- Keep the helper `.m` files in the same folder or on the MATLAB path.
- The MAT files are binary MATLAB data files. Their contents should be treated as case-specific saved state, not as universal defaults.


### 7. Notes and limitations

- Several objective metrics and target values are manually configured. Their units, normalization, and engineering suitability should be checked for the intended mechanism before using results for manufacturing or safety-critical decisions.
- Validate exported profiles in the target CAD/CAE application and against mechanical constraints before fabrication.
- Some filenames and comments may be case-specific. Update the waypoints and parameters consistently when adapting the package to a new course or mechanism.

---

## 简体中文

### 1. 项目简介

该 MATLAB 代码包实现了一套**路径规划与机械凸轮轮廓设计流程**。程序使用贝塞尔曲线对给定途经点进行平滑，模拟类似车辆的机构运动（包括车身、主动轮、从动轮和传感器的位置），计算与转向相关的参数，并进一步生成理论凸轮轮廓和虚拟凸轮轮廓。主程序通过迭代调整路径控制点，根据几何与运动学指标对候选路径进行评分和优化。

坐标与机械尺寸以**毫米（mm）**为单位。

### 2. 主要工作流程

1. **配置路径与机构参数**：在 `route_design_AI.m` 中设置途经点、场地范围、车轮偏移量、传感器位置、初始方向以及凸轮参数。
2. **初始化或恢复路径控制点**：若不存在对应的 `.mat` 状态文件，程序会初始化控制点；若存在，则读取已有状态继续计算。
3. **生成平滑路径**：通过 `Bezier_superposition.m` 和 `Bezier_curve.m` 进行贝塞尔曲线采样与分段融合。
4. **仿真路径和转向运动**：`Simulation_cal.m` 与 `all_routes_cal.m` 计算路径几何、车轮/传感器轨迹和前轮转角等数据。
5. **计算凸轮轮廓与约束指标**：`cam_cal_elliptic.m` 根据路径和转向数据生成理论凸轮及虚拟凸轮，并计算曲率、压力角等指标。
6. **迭代优化路径**：主程序并行评估控制点扰动后的候选方案，更新控制点并保存过程状态。
7. **导出凸轮点集**：运行 `cam_calculat.m`，根据 `format` 设置输出格式。


### 3. 文件说明

| 文件 | 功能 |
|---|---|
| `route_design_AI.m` | 主路径优化脚本。配置途经点和机械参数，评估候选路径，更新控制点，并保存优化结果。 |
| `cam_calculat.m` | 加载已保存的优化结果，计算凸轮轮廓，导出凸轮坐标，并可视化路径和凸轮几何形状。 |
| `Simulation_cal.m` | 评估路径几何特征、路径长度、平滑度、传感器偏差以及转向角变化情况。 |
| `Bezier_curve.m` | 根据控制点生成贝塞尔曲线。 |
| `Bezier_superposition.m` | 通过融合多个贝塞尔曲线段，构建平滑的复合轨迹。 |
| `cal_YangHui.m` | 计算贝塞尔曲线生成过程中所需的二项式系数（杨辉三角系数）。 |
| `cal_endpoints.m` | 使用有限差分方法估算端点导数。 |
| `all_routes_cal.m` | 计算车身、车轮和传感器的运动轨迹，以及相应的转向角。 |
| `cam_cal_elliptic.m` | 生成理论凸轮轮廓和虚拟凸轮轮廓，并评估曲率及压力角等约束条件。 |
| `Curvature_radius.m` | 计算由三个点确定的外接圆半径。 |
| `distance_calculate.m` | 评估传感器运动轨迹与目标途经点之间的偏差。 |
| `Point_line_distance.m` | 计算点到直线的垂直距离。 |
| `Squares_standard_sum.m` | 基于数据的平方值计算综合偏差指标。 |
| `score_cal.m` | 根据可配置的评价指标计算综合优化评分。 |
| `Optimize_P0.m` | 调整内部控制点，以改善轨迹的平滑性。 |
| **`Trajectory_A.mat`** | 论文中 **Trajectory_A** 对应的路径优化状态文件，包含优化后的控制点及优化相关变量。 |
| **`Trajectory_A_max.mat`** | 论文中 **Trajectory_A** 对应的仿真与优化结果文件，用于后续轨迹分析和凸轮轮廓生成。 |
| **`TrajectoryA.txt`** | 论文中 **Trajectory_A** 对应的凸轮轮廓坐标文件，包含三维坐标点（X、Y、Z），可用于在 SOLIDWORKS 中导入并重建凸轮轮廓。 |

**说明：** `Trajectory_A.mat`、`Trajectory_A_max.mat` 和 `TrajectoryA.txt` 这三个文件是配套论文中 **Trajectory_A** 对应的 MATLAB 优化与仿真结果及凸轮轮廓输出文件。用户可以通过这些文件查看已保存的优化状态、分析优化后的运动轨迹，并在 SOLIDWORKS 中重建相应的凸轮轮廓。


### 4. 使用方法

#### A. 优化路径

1. 将 ZIP 压缩包解压到可写目录。
2. 打开 MATLAB，将 Current Folder 切换到解压后的 `matlab_tulun` 目录。
3. 打开 `route_design_AI.m`。
4. 检查并按需修改脚本顶部的参数，重点包括：
   - `data_name`：状态文件前缀，例如本代码包提供的 `Trajectory_A`。
   - `configuration`：选择读取哪种已保存的 MAT 文件/配置。
   - `P`：按行排列的路径途经点坐标。
   - `Venue_size`：场地/绘图范围。
   - `driving_wheel`、`driven_wheel`、`Z_sensor`、`V_0`：机构偏移量与初始方向。
   - `d_driving`、`i_total`、`c`、`thickness`、`d`、`r`：凸轮与机械参数。
   - `traing_times`、`num`、`route_index`、`learning_efficiency`、`gradient_power0`：优化设置。
5. 运行脚本。程序会显示路径和凸轮图形，并在迭代过程中保存 `.mat` 状态文件。

**注意：** 若对应的 MAT 文件已经存在，程序会优先加载它们。若要从头开始新的案例，建议使用新的 `data_name`，或先备份再有意地重命名/移除相关状态文件，避免意外覆盖已有结果。

#### B. 导出凸轮轮廓

1. 打开 `cam_calculat.m`。
2. 将 `data_name` 设置为要导出的案例。当前脚本默认使用 `Trajectory_A`，并读取 `Trajectory_A_max.mat`。
3. 设置 `save_name` 和 `format`：
   - `.ibl`：输出 IBL 风格的点/曲线文件。
   - `.txt`：输出以制表符分隔的 X/Y/Z 坐标点，其中 Z = 0。
   - `.scr`：输出简单的 CAD `LINE` 脚本。
4. 运行脚本，导出文件将保存到当前工作目录。

### 5. 在 SOLIDWORKS 中建立凸轮模型

导出的 `TrajectoryA.txt` 包含 `X Y Z` 三列凸轮轮廓坐标（其中 `Z = 0`），可用于在 SOLIDWORKS 中创建平面凸轮曲线：

1. 打开 `cam_calculat.m`，将 `format` 设置为 `.txt` 并运行脚本，导出坐标文件。
2. 在 SOLIDWORKS 中使用曲线导入功能，例如 **插入 > 曲线 > 通过 XYZ 点的曲线**（不同版本菜单名称可能有所不同），选择该文本文件。
3. 检查坐标列格式、单位（**mm**）及导入后的曲线形状。如有必要，按软件要求调整文本分隔符。
4. 基于导入曲线创建草图和相应的实体特征，并检查最终凸轮几何形状。

**注意：** 导入坐标文件得到的是曲线，并非完整的凸轮实体。制造前仍需验证轮廓与机械约束。

### 6. 运行环境与兼容性

- 需要安装 MATLAB。
- `route_design_AI.m` 使用 `parfor`。按当前写法运行该部分通常需要兼容的 Parallel Computing Toolbox 及相应许可；若没有该工具箱，可考虑将 `parfor` 改为 `for` 进行串行计算，但需考虑运行速度和 MATLAB 版本差异。
- 脚本包含交互式绘图和 `pause`，运行时可能会暂停以便观察图形。
- 请确保所有辅助 `.m` 文件都在同一目录或已加入 MATLAB 搜索路径。
- `.mat` 文件是 MATLAB 二进制数据文件，保存的是特定案例的状态，不应直接视为适用于所有场景的通用默认参数。


### 7. 注意事项与局限

- 多个目标指标及其期望值由人工配置。将结果用于具体机构前，应检查指标单位、归一化方式及工程合理性。
