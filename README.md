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
| `route_design_AI.m` | Main route optimization/training script. Sets waypoints and mechanism parameters, scores candidate routes, updates control points, saves state, and plots progress. |
| `cam_calculat.m` | Loads a saved route state, calculates the cam profile, exports cam points, and animates/plots the route and cam. |
| `Simulation_cal.m` | Samples the route and computes route length, control-point smoothness measures, sensor-to-waypoint deviation, and steering-angle variation metrics. |
| `Bezier_curve.m` | Samples a Bézier curve from control points. |
| `Bezier_superposition.m` | Builds a smooth composite route by blending multiple Bézier curve segments with weights. |
| `cal_YangHui.m` | Computes binomial coefficients (Pascal-triangle coefficients) used by Bézier calculations. |
| `cal_endpoints.m` | Estimates endpoint derivatives from sampled values using finite-difference/binomial weights. |
| `all_routes_cal.m` | Computes body, driving-wheel, driven-wheel, and sensor positions along the route, plus front-wheel steering angles. |
| `cam_cal_elliptic.m` | Converts steering/route information into theoretical and virtual cam profiles; calculates curvature, pressure-angle, and profile-angle metrics. |
| `Curvature_radius.m` | Calculates the radius of the circle through three points; returns `Inf` for degenerate cases. |
| `distance_calculate.m` | Estimates deviation between the sensor path and the required waypoint path. |
| `Point_line_distance.m` | Computes the perpendicular distance from a point to a line segment's supporting line (with a coincident-endpoint fallback). |
| `Squares_standard_sum.m` | Computes a root-mean-square-like aggregate: `sqrt(sum(data.^2)/N) * N`. |
| `score_cal.m` | Converts metric-to-target ratios into a total score using configurable exponents. |
| `Optimize_P0.m` | Smooths interior control points toward the average of their neighbours. |
| `cal_endpoints.m` | Endpoint helper used when estimating endpoint values. |
| `shanghai_test1.mat`, `shanghai_test1_max.mat` | Saved optimization state/data for the `shanghai_test1` case. |
| `shanghai_del4.mat`, `shanghai_del4_max.mat` | Saved optimization state/data for the `shanghai_del4` case. |
| `cam.txt`, `cam.scr` | Example/export-related cam point data or CAD script files. Their exact provenance depends on the run that generated them. |

### 4. How to run

#### A. Optimize a route

1. Extract the ZIP archive to a writable folder.
2. Open MATLAB and set the Current Folder to the extracted `matlab_tulun` directory.
3. Open `route_design_AI.m`.
4. Review and edit the configuration section near the top, especially:
   - `data_name`: state-file prefix, e.g. `shanghai_test1`.
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
2. Set `data_name` to the case to export. The script currently refers to `shanghai_del4` and loads `shanghai_del4_max.mat`.
3. Set `save_name` and `format`:
   - `.ibl`: writes an IBL-style point/curve file.
   - `.txt`: writes tab-separated X/Y point coordinates.
   - `.scr`: writes a simple CAD `LINE` script.
4. Run the script. The exported file is written to the current working directory.

### 5. Create a curve model in SolidWorks

After exporting the cam coordinates to a `.txt` file, you can use the point data to create the corresponding curve in **SolidWorks (SW)**:

1. Run `cam_calculat.m` and set `format` to `.txt`.
2. In SolidWorks, use the curve-import command (for example, **Insert > Curve > Curve Through XYZ Points**; menu names may vary by version) and select the exported text file.
3. Check the file's coordinate layout and delimiter. If the SolidWorks command requires three-dimensional coordinates, convert each point from `X Y` to `X Y 0` so that the curve lies in the XY plane.
4. Confirm the units and the curve shape after import, then use the imported curve as the basis for the next modeling steps.

The exact import requirements depend on the SolidWorks version and the text-file format. If the file is not accepted directly, reformat the coordinates to match the format expected by the selected curve-import command.

### 5. Building a cam model in SOLIDWORKS

The exported point-coordinate file can be used to create a cam profile in SOLIDWORKS. For example, when `cam_calculat.m` exports a tab-delimited `.txt` file containing X/Y coordinates:

1. Run `cam_calculat.m` and set `format` to the text export option.
2. In SOLIDWORKS, import the generated coordinate file using the available curve/coordinate import workflow (the exact menu name may vary by version).
3. Check that the imported points use the intended units (millimetres) and coordinate system.
4. Create or fit a 2D curve through the imported points. Inspect the curve for discontinuities, self-intersections, or unexpected scaling.
5. If a 3D cam part is required, use the resulting profile in a suitable sketch and apply the required extrusion, revolve, or other feature based on the actual cam design.

The `.txt` output described here contains point coordinates; importing it does not automatically create a finished solid cam. Additional sketching/features and engineering checks may be required. Verify the profile and dimensions in SOLIDWORKS before manufacturing.

### 6. Requirements and compatibility notes

- MATLAB is required.
- `route_design_AI.m` uses `parfor`; a compatible Parallel Computing Toolbox/license is typically required to execute that section as written. If unavailable, consider replacing `parfor` with `for` for serial execution, subject to performance and MATLAB-version compatibility.
- The scripts use interactive plotting and `pause`, so execution may stop for visual inspection during the run.
- Keep the helper `.m` files in the same folder or on the MATLAB path.
- The MAT files are binary MATLAB data files. Their contents should be treated as case-specific saved state, not as universal defaults.


### 7. Notes and limitations

- Several objective metrics and target values are manually configured. Their units, normalization, and engineering suitability should be checked for the intended mechanism before using results for manufacturing or safety-critical decisions.
- Validate exported profiles in the target CAD/CAE application and against mechanical constraints before fabrication.
- Some filenames and comments are case-specific (for example, Shanghai route data). Update the waypoints and parameters consistently when adapting the package to a new course or mechanism.

---

## 简体中文

### 1. 项目简介

该 MATLAB 代码包实现了一套**路径规划与机械凸轮轮廓设计流程**。程序使用贝塞尔曲线对给定途经点进行平滑，模拟类似车辆的机构运动（包括车身、主动轮、从动轮和传感器的位置），计算与转向相关的参数，并进一步生成理论凸轮轮廓和虚拟凸轮轮廓。主程序通过迭代调整路径控制点，根据几何与运动学指标对候选路径进行评分和优化。

从代码注释和变量命名来看，坐标和机械尺寸通常以**毫米（mm）**为单位。
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
| `route_design_AI.m` | 主路径优化/训练脚本。配置途经点和机构参数，对候选路径评分、更新控制点、保存状态并绘制优化过程。 |
| `cam_calculat.m` | 读取已保存的路径状态，计算凸轮轮廓，导出凸轮点集，并绘制/动画展示路径与凸轮。 |
| `Simulation_cal.m` | 对路径采样，计算路径长度、控制点平滑度指标、传感器轨迹与途经点的偏差，以及转向角变化指标。 |
| `Bezier_curve.m` | 根据控制点生成贝塞尔曲线采样点。 |
| `Bezier_superposition.m` | 通过权重融合多个贝塞尔曲线段，构建平滑的复合路径。 |
| `cal_YangHui.m` | 计算贝塞尔曲线所需的二项式系数（杨辉三角系数）。 |
| `cal_endpoints.m` | 使用有限差分/二项式权重估算序列两端的导数或端点值。 |
| `all_routes_cal.m` | 沿路径计算车身、主动轮、从动轮和传感器的位置，以及前轮转角。 |
| `cam_cal_elliptic.m` | 根据转向和路径数据计算理论凸轮、虚拟凸轮，并计算曲率、压力角和轮廓角度变化等指标。 |
| `Curvature_radius.m` | 根据三个点计算外接圆半径；遇到退化情况时返回 `Inf`。 |
| `distance_calculate.m` | 估算传感器路径与目标途经点路径之间的偏离程度。 |
| `Point_line_distance.m` | 计算点到直线的垂直距离；当线段两端重合时，退化为点间距离。 |
| `Squares_standard_sum.m` | 计算平方均方根形式的聚合量：`sqrt(sum(data.^2)/N) * N`。 |
| `score_cal.m` | 根据指标与目标值的比值及可配置指数，计算综合评分。 |
| `Optimize_P0.m` | 将内部控制点向相邻控制点的平均位置平滑，以调整控制点分布。 |
| `shanghai_test1.mat`、`shanghai_test1_max.mat` | `shanghai_test1` 路径案例的已保存优化状态/数据。 |
| `shanghai_del4.mat`、`shanghai_del4_max.mat` | `shanghai_del4` 路径案例的已保存优化状态/数据。 |
| `cam.txt`、`cam.scr` | 凸轮点集或 CAD 脚本相关的示例/导出文件；具体来源取决于生成它们时运行的配置。 |

### 4. 使用方法

#### A. 优化路径

1. 将 ZIP 压缩包解压到可写目录。
2. 打开 MATLAB，将 Current Folder 切换到解压后的 `matlab_tulun` 目录。
3. 打开 `route_design_AI.m`。
4. 检查并按需修改脚本顶部的参数，重点包括：
   - `data_name`：状态文件前缀，例如 `shanghai_test1`。
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
2. 将 `data_name` 设置为要导出的案例。当前脚本使用 `shanghai_del4`，并读取 `shanghai_del4_max.mat`。
3. 设置 `save_name` 和 `format`：
   - `.ibl`：输出 IBL 风格的点/曲线文件。
   - `.txt`：输出以制表符分隔的 X/Y 坐标点。
   - `.scr`：输出简单的 CAD `LINE` 脚本。
4. 运行脚本，导出文件将保存到当前工作目录。

### 5. 在 SolidWorks 中生成曲线模型

凸轮坐标导出为 `.txt` 文件后，可以将该文件导入 **SolidWorks（SW）**，生成对应的曲线，作为后续凸轮建模的基础：

1. 运行 `cam_calculat.m`，将 `format` 设置为 `.txt`，生成凸轮坐标文件。
2. 在 SolidWorks 中使用曲线导入命令（例如 **插入 > 曲线 > 通过 XYZ 点的曲线**；不同版本的菜单名称可能略有差异），并选择导出的文本文件。
3. 检查坐标列和分隔符格式。如果所使用的 SolidWorks 命令要求三维坐标，可将每个点从 `X Y` 转换为 `X Y 0`，使曲线位于 XY 平面内。
4. 导入后检查曲线形状和单位，再以该曲线为基础进行后续凸轮模型构建。

具体导入要求取决于 SolidWorks 版本及文本文件格式。如果软件无法直接识别该文件，请按照所选曲线导入命令要求调整坐标格式。

### 5. 在 SOLIDWORKS 中建立凸轮模型

程序导出的坐标点文件可以用于在 SOLIDWORKS 中生成凸轮轮廓。例如，运行 `cam_calculat.m` 并选择文本导出格式后，可以得到包含 X/Y 坐标的 `.txt` 文件。

1. 运行 `cam_calculat.m`，将 `format` 设置为文本导出选项。
2. 在 SOLIDWORKS 中，使用相应的曲线/坐标导入功能导入生成的坐标文件（不同版本的菜单名称可能略有差异）。
3. 检查导入点的单位是否为预期的毫米（mm），并确认坐标系正确。
4. 根据导入的点创建或拟合二维曲线，检查曲线是否存在不连续、自相交或比例异常等问题。
5. 如果需要三维凸轮零件，可将生成的轮廓用于草图，并根据实际设计要求使用拉伸、旋转或其他特征完成建模。

需要注意：`.txt` 文件保存的是轮廓坐标点，导入后并不会自动生成完整的实体凸轮。通常还需要进一步创建草图和实体特征，并进行工程校核。制造前请在 SOLIDWORKS 中核对轮廓和尺寸。

### 6. 运行环境与兼容性

- 需要安装 MATLAB。
- `route_design_AI.m` 使用 `parfor`。按当前写法运行该部分通常需要兼容的 Parallel Computing Toolbox 及相应许可；若没有该工具箱，可考虑将 `parfor` 改为 `for` 进行串行计算，但需考虑运行速度和 MATLAB 版本差异。
- 脚本包含交互式绘图和 `pause`，运行时可能会暂停以便观察图形。
- 请确保所有辅助 `.m` 文件都在同一目录或已加入 MATLAB 搜索路径。
- `.mat` 文件是 MATLAB 二进制数据文件，保存的是特定案例的状态，不应直接视为适用于所有场景的通用默认参数。


### 7. 注意事项与局限

- 多个目标指标及其期望值由人工配置。将结果用于具体机构前，应检查指标单位、归一化方式及工程合理性。
- 在制造或用于安全相关场景之前，应在目标 CAD/CAE 软件中验证导出的轮廓，并检查其是否满足机械约束。
- 部分参数和文件名与特定案例（例如上海路径数据）绑定。迁移到新路线或新机构时，应一致地更新途经点及相关参数。

---
