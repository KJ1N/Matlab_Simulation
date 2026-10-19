<a id="english"></a>

# Trajectory Optimisation and Cam Profile Design for a Solar-Powered Vehicle

English | [跳转到中文版](#chinese)

This MATLAB project converts ordered checkpoints and vehicle geometry into front-wheel and sensor trajectories, then into theoretical and corrected cam profiles. It optimises control points using a perturbation search, evaluates candidates with `parfor`, and exports cam coordinates for CAD modelling.

Project lead: Chuheng Jin. This README describes **the MATLAB files and `shanghai_del4` example in the current root directory**. The GitHub repository also includes a `shanghai_test1` example. Supplementary reports and hardware-review materials from the local project workspace are not included in this repository.

## 1. Scope and workflow

```text
Ordered targets P + vehicle geometry + initial heading
                         ↓
Initial control points P0 → blended local Bezier curves
                         ↓
Body, wheel and sensor paths → front-wheel steering angles
                         ↓
Follower displacement → theoretical and corrected cams
                         ↓
12 features → parallel candidate evaluation → control-point update
                         ↓
MAT snapshots → recomputation → TXT / IBL / SCR export
```

- The path blends overlapping local Bezier curves; it does not interpolate every target exactly.
- Checkpoint penalties apply to the **sensor path**, which differs from the front-wheel path because of the mounting offset.
- The optimiser is a heuristic coordinate-perturbation search with an accumulated direction, step-size adjustment, and periodic perturbations. `AI` in the filename does not indicate a neural network or guarantee a global optimum.
- These are desktop geometry-design scripts, not RP2040 firmware or motor PWM, NFC, or charging-control software.

## 2. Requirements

- MATLAB: an R2021a installation was identified in the project environment, but MATLAB was not launched for this documentation task. The minimum compatible release has not been established.
- For parallel execution of `route_design_AI.m`, install and enable Parallel Computing Toolbox and configure an available local pool.
- Recomputing and exporting an existing snapshot can use the functions directly without entering the optimisation script's `parfor` loop.
- CAD software is needed for downstream modelling, not for the MATLAB calculations themselves.
- Several `.m` files contain Chinese comments encoded as GBK/GB18030. If comments appear corrupted, reopen with the appropriate encoding instead of saving damaged text over the source.

Check the environment in MATLAB:

```matlab
ver
which route_design_AI -all
which Simulation_cal -all
```

Set MATLAB's Current Folder to this README's directory. If you keep historical copies elsewhere, avoid recursively adding them to the MATLAB path: identically named functions may shadow this implementation.

## 3. Quick start

### 3.1 Recompute the supplied example first

The following example loads a complete snapshot into a structure, recomputes its geometry, and writes a new coordinate file. It does not optimise or modify the original MAT files.

Create a MATLAB script in the root directory, paste this code, and run it:

```matlab
S = load('shanghai_del4_max.mat');
sampleDensity = S.num;
endpointOrder = round(sqrt(sampleDensity / S.route_index));

[frontPath, ~, drivePath, ~, sensorPath, steering] = Simulation_cal( ...
    S.P0, S.route_index, sampleDensity, S.V_0, ...
    S.driving_wheel, S.driven_wheel, S.Z_sensor, endpointOrder, S.P);

[camTheory, camProfile] = cam_cal_elliptic( ...
    steering, S.c, S.thickness, drivePath, ...
    S.d_driving, S.i_total, S.d, S.r);

assert(isreal(camProfile) && all(isfinite(camProfile(:))), ...
    'Cam coordinates contain complex or non-finite values.');

figure;
subplot(1, 2, 1);
plot(frontPath(:,1), frontPath(:,2), 'r-', ...
     sensorPath(:,1), sensorPath(:,2), 'b-', ...
     S.P(:,1), S.P(:,2), 'ko');
axis equal; grid on;
xlabel('x / mm'); ylabel('y / mm');
legend('Front wheel', 'Sensor', 'Targets');

subplot(1, 2, 2);
plot(camTheory(:,1), camTheory(:,2), 'r-', ...
     camProfile(:,1), camProfile(:,2), 'b-');
axis equal; grid on;
xlabel('x / mm'); ylabel('y / mm');
legend('Theoretical cam', 'Corrected cam');

outFile = 'cam_preview_xyz.txt';
assert(~isfile(outFile), 'Output already exists; choose a new name.');
writematrix([camProfile, zeros(size(camProfile,1),1)], ...
    outFile, 'Delimiter', 'tab');
```

The assertion is a basic coordinate-validity check, not validation of self-intersection, manufacturing tolerances, or contact feasibility.

### 3.2 Create and optimise a new route

Open `route_design_AI.m` in the editor and change its configuration block:

```matlab
data_name = 'my_route_01';  % Choose a name that does not exist yet
configuration = 0;
traing_times = 3;          % Start with a few iterations to check the setup
Venue_size = [0, 8000, 0, 8000];
```

Set `P`, vehicle geometry, and cam parameters, save the script, then run:

```matlab
route_design_AI
```

The script plots the paths, steering angles, and cam, and creates `my_route_01.mat` and `my_route_01_max.mat`. Increase the iteration count after checking the configuration. Both files are repeatedly overwritten during optimisation.

**Edit parameters inside the script.** Both entry scripts start with `clc; clear;`; assigning similarly named variables in the Command Window will not override their configuration. Save any other workspace data before running them.

The optimisation script defaults to `data_name='shanghai_test1'`, whereas the export script defaults to `shanghai_del4`. These names are not synchronised automatically. Since `shanghai_test1.mat` is supplied, running the optimisation defaults loads its existing control points and overwrites its snapshots. Use a new name for an independent run.

### 3.3 Continue an existing route

The actual loading behaviour is:

| Condition | Behaviour | Important detail |
|---|---|---|
| Ordinary MAT file does not exist | Initialise control points from `P` | This branch is taken even if a matching `_max.mat` exists |
| Ordinary MAT exists and `configuration=0` | Load `<data_name>.mat` | The supplied small file contains only `P0` and `optimal_gradient`; other settings come from the script |
| Ordinary MAT exists and `configuration=1` | Load the entire `<data_name>_max.mat` workspace | Saved values overwrite matching script variables; both files must exist |

Some optimiser variables are reset and a new loop is started; this is not an instruction-by-instruction resume. With `configuration=0`, changing `P` does not rebuild a loaded `P0`. Prefer a new `data_name` for substantially different checkpoints or geometry.

## 4. Parameters

Lengths use **millimetres** and internal angles use **radians**. The local vehicle frame has its origin at the front wheel, its y-axis pointing forward along the initial body heading, and its x-axis pointing right.

### 4.1 Targets and geometry

| Parameter | Current optimisation-script value | Meaning |
|---|---|---|
| `P` | 9×2 array | Ordered targets; first and last rows are the start and finish |
| `Venue_size` | `[0,6000,0,6000]` | Plot limits, not enforced field constraints |
| `driving_wheel` | `[94,-130]` | Driven-by-motor wheel position relative to the front wheel |
| `driven_wheel` | `[-54,-130]` | Passive wheel position relative to the front wheel |
| `Z_sensor` | `[25,-53]` | Sensor position relative to the front wheel |
| `V_0` | `[-3,-1]` | Nonzero initial heading; normalised by the optimisation script |
| `d_driving` | `180` | Drive-wheel diameter |
| `i_total` | `36` | Drive-wheel revolutions per cam revolution |
| `c` | `-33` | Signed steering-linkage offset; its sign affects cam generation |
| `thickness` | `3` | Cam thickness |
| `d` | `130` | Reference cam diameter, not necessarily the finished minimum diameter |
| `r` | `2` | Radius of the surface contacting the cam |

The current default targets fit within `[0,6000,0,6000]`. Update the plot limits if you supply targets outside this range.

The supplied complete snapshot differs from the script defaults: `shanghai_del4_max.mat` has `i_total=29.8`, `c=-20`, and `Venue_size=[0,10000,0,10000]`. Use the snapshot's own settings when reproducing that example.

### 4.2 Optimisation and sampling

| Parameter | Default | Meaning |
|---|---|---|
| `traing_times` | `399` | Iteration count; spelling retained from source |
| `control_num` | `5` | Initial subdivisions of each target-to-target segment |
| `route_index` | `7` | Degree of each local Bezier curve |
| `num` | `20` | Sampling density per control-point interval |
| `gradient_power0` | `40` | Initial coordinate perturbation scale |
| `learning_efficiency` | `0.3` | Update coefficient for the accumulated search direction |
| `power_P0` | `0.6` | Initial control-point smoothing coefficient |
| `perturbation_frequency` | `50` | Interval for periodic penalty-exponent changes |
| `perturbation_power` | `2` | Penalty-exponent perturbation magnitude |
| `Scored_index` | `5+[0;0;3;0;0;0;0;0;0;0;0;0]` | Exponents for the 12 features |
| `E_*` | See script | Feature normalisation scales, not hard tolerances |

For `m` targets, initialisation creates `n=(m-1)*control_num+1` control points and `(n-1)*num+1` trajectory samples. The current defaults give **41 control points, 821 trajectory samples, and 165 candidates per iteration**. The 46-control-point / 185-candidate case discussed in earlier reports is a different historical snapshot.

Use finite real inputs and avoid coincident adjacent control points, zero heading vectors, and degenerate geometry. `n` must exceed `route_index`, with enough samples for endpoint extrapolation.

## 5. Parallel evaluation and scoring

Each iteration fixes the current control points and scoring settings and constructs `4*n+1` candidates: positive and negative perturbations of every control coordinate, plus a baseline containing only the current accumulated direction. Each candidate independently computes its path, cam, and score. Sorting and state updates then run serially.

`threads_num` is a **candidate count**, not a hardware-thread or worker count. Outer iterations depend on preceding states and cannot be parallelised in the same way.

With Parallel Computing Toolbox installed and sufficient resources, a local process pool can be configured before running the script; for example, in R2021a:

```matlab
pool = gcp('nocreate');
if isempty(pool)
    parpool('local', 4);
end
```

This leaves an existing pool unchanged. For serial debugging, replace `parfor thread=1:threads_num` with `for thread=1:threads_num` in a copy of the script. No standalone benchmark entry point or verified speedup results are supplied.

The total score is `-sum((eigenvalues ./ E_eigenvalues).^scored_index)`; the numerically largest score is selected. The 12 features are: front-wheel path length; control-polygon segment-length aggregate; sensor checkpoint deviation; control-polygon turning angles and their first/second differences; front-wheel steering angles and their first/second differences; a maximum theoretical-cam curvature metric; maximum pressure angle; and maximum adjacent tangent-direction change of the corrected profile.

For an N-element vector, `Squares_standard_sum(x)` returns `sqrt(N*sum(x.^2))`, not a conventional sum of squares or RMS. Angular differences are not divided by time and are not physical angular velocity or acceleration. Scales and exponents change during optimisation, so raw scores across iterations may not describe convergence of a fixed objective.

## 6. Original exporter and file formats

Configure `cam_calculat.m`:

```matlab
data_name = 'shanghai_del4';
save_name = 'cam_export_01';  % Prefer a new output name
format = '.txt';
num_route = 20;
```

Save and run `cam_calculat`. It loads `<data_name>_max.mat`, recomputes the cam, writes the file, and then displays an animation. Export uses `w+`, overwriting any matching output. The default is `cam.txt`; rename it or back it up first.

| Format | Actual content | Intended downstream use |
|---|---|---|
| `.txt` | Tab-separated X, Y, 0 per row, no header | SolidWorks XYZ-curve import or similar |
| `.ibl` | `open/arclength/begin section/begin curve` header, then index, X, Y, 0 | Creo curve import |
| `.scr` | `line` command followed by X,Y coordinates | AutoCAD command script |

The exported geometry is `cam_virtual`, the corrected profile. All Z coordinates are zero. Curve construction, closure checks, and solid modelling remain CAD steps. The `.scr` exporter does not emit an explicit close command; confirm command termination and format compatibility in the target CAD release.

## 7. File guide

| File | Responsibility |
|---|---|
| `route_design_AI.m` | Configuration, candidate generation, parallel scoring, updates, snapshots, and plots |
| `cam_calculat.m` | Load a complete snapshot, export the cam, and animate geometry |
| `Simulation_cal.m` | Assemble trajectory, vehicle geometry, and route features |
| `all_routes_cal.m` | Body, wheel, and sensor positions and front-wheel steering |
| `Bezier_superposition.m` / `Bezier_curve.m` | Blended trajectory and individual/piecewise Bezier evaluation |
| `cal_YangHui.m` / `cal_endpoints.m` | Binomial coefficients and endpoint extrapolation |
| `cam_cal_elliptic.m` | Follower displacement, return segment, theoretical cam, and contact correction |
| `score_cal.m` / `Squares_standard_sum.m` | Combined score and feature aggregation |
| `distance_calculate.m` / `Point_line_distance.m` | Sequential checkpoint search and point-to-line distance |
| `Curvature_radius.m` / `Optimize_P0.m` | Three-point curvature-radius estimate and control-point smoothing |
| `shanghai_del4.mat` | Small checkpoint: `P0`, `optimal_gradient` |
| `shanghai_del4_max.mat` | Whole-workspace snapshot including parameters, intermediate arrays, and graphics handles |
| `cam.txt` / `cam.scr` | Existing exports; filenames alone do not establish their source configuration |
| `shanghai_test1.mat` / `shanghai_test1_max.mat` | Additional supplied route: 41 control points; full snapshot uses a 180 mm drive wheel, ratio 36, and `c=-33` |

## 8. Known limitations and troubleshooting

1. **Complete snapshots can mix iteration states.** The optimiser updates `P0` and saves before updating `score_times`, trajectory, and cam arrays. Saved arrays may describe the preceding iteration, and the last score entry may still be unwritten. Recompute from saved `P0` before analysis/export; do not interpret state mismatch as numerical error. `_max` is overwritten every iteration and is not a rigorously maintained historical-best checkpoint.
2. **The exporter's trailing output names are misaligned.** `Simulation_cal` returns 17 outputs; the receiving list in `cam_calculat.m` omits `ddt_cp_sum`, shifting several later feature names. The earlier outputs used for cam construction are unaffected by this mismatch. Match the function signature explicitly, or use `~` for unused outputs when computing metrics.
3. **Changing export density should also update extrapolation settings.** The original exporter loads `index` from the snapshot without recomputing it for `num_route`. Section 3.1 recalculates `endpointOrder` for its chosen density.
4. **Checkpoint distance is heuristic.** The original routine searches sequential local minima and uses distance to infinite lines rather than a global minimum over finite segments. Unresolved intermediate targets may retain their initial zero values. Independently verify all target distances and traversal order.
5. **Degenerate geometry may produce NaN, Inf, or complex values.** Check repeated points, zero-length vectors, `acos` domains, and divisions during return-segment construction. Clipping and recovery are only partial in the source.
6. **Travel length must match the cam revolution.** A return segment is added when drive-wheel travel is below `pi*d_driving*i_total`. At or above that length, the existing travel is normalised to one revolution without explicitly rejecting excessive travel. Independently check transmission ratio and physical closure.
7. **Some loops and animation initialisation have portability issues.** Constructs such as `for i=2:size(points)` in `all_routes_cal.m` use a nonscalar upper bound. If MATLAB reports a colon-operand problem, make the dimension explicit with `size(points,1)`. The exporter also reuses `i` from an earlier loop when initialising animation. Prefer the direct function calls in Section 3.1 for automation.
8. **Some plot labels do not match their variables.** Certain plots label sample indices as distance, or label radians/displacement inconsistently. Inspect the plotted variables and relabel axes before using figures in reports.

This is primarily a discrete geometric and kinematic model. It does not model slip, structural compliance, backlash, motor dynamics, or changing illumination. A successful geometric simulation does not establish successful physical checkpoint detection. Check profile self-intersection, closure, contact feasibility, and manufacturing constraints before fabrication.

## 9. Documentation verification and references

This update adds a single bilingual README, with English first and Chinese second. Documentation was checked by statically reading the root `.m` files and inspecting the supplied MAT snapshots' fields and parameters. Algorithms were not changed; full optimisation and CAD imports were not run. Usage examples follow the current function interfaces but have not been executed in MATLAB.

- [Usage video linked by the original directory](https://www.bilibili.com/video/BV1xF411N7Vf)
- [MATLAB parfor documentation](https://www.mathworks.com/help/parallel-computing/parfor.html)

No explicit software licence is supplied in the current root directory. Add applicable licensing and source-material permissions before external distribution.

---

<a id="chinese"></a>

# 太阳能小车轨迹优化与凸轮轮廓设计

[Back to English](#english) | 中文

本项目使用 MATLAB 将有序打卡点和车辆几何参数转换为前轮轨迹、传感器轨迹及凸轮轮廓。程序通过控制点扰动搜索优化路线，使用 `parfor` 并行评价候选方案，并支持向 CAD 软件导出凸轮坐标。

项目负责人：金楚恒。本文档针对**当前根目录中的 MATLAB 代码与 `shanghai_del4` 示例**编写；GitHub 仓库还包含 `shanghai_test1` 示例。本地工作目录中的历史报告、Python 复核与硬件整理资料未包含在此仓库中。

## 1. 功能与计算流程

```text
有序目标点 P + 车辆几何 + 初始朝向
                 ↓
初始化控制点 P0 → 加权叠加贝塞尔曲线
                 ↓
推算车身、车轮及传感器轨迹，计算前轮转角
                 ↓
转角转换为推移量，生成理论与修正凸轮
                 ↓
评价 12 项特征 → 并行比较候选 → 更新控制点
                 ↓
保存 MAT 存档 → 重新计算 → 导出 TXT / IBL / SCR
```

- 路径由重叠的局部贝塞尔曲线加权生成，不保证精确通过全部目标点。
- 目标点约束作用于**传感器轨迹**；由于安装偏置，传感器轨迹与前轮轨迹不同。
- 优化算法是带累积搜索方向、步长调整及周期性扰动的启发式控制点搜索。文件名中的 `AI` 不表示神经网络训练，也不保证全局最优。
- 本目录是电脑端几何设计代码，不负责 RP2040 固件、电机 PWM、NFC 通信或充电控制。

## 2. 环境与依赖

- MATLAB：项目整理环境中发现了 R2021a 安装，但本次 README 编写未启动 MATLAB 验证，最低兼容版本未确认。
- 并行运行 `route_design_AI.m`：建议安装并启用 Parallel Computing Toolbox，并配置可用的本地并行池。
- 仅重新计算和导出已有存档：可直接调用本项目函数，无需运行优化入口中的 `parfor`。
- 导出后的建模需相应 CAD 软件；MATLAB 计算本身不要求安装 CAD 软件。
- 多个 `.m` 文件含 GBK/GB18030 编码的中文注释。若出现乱码，应以相应编码打开，不要把已损坏的文本重新保存覆盖。

可在 MATLAB 中检查环境：

```matlab
ver
which route_design_AI -all
which Simulation_cal -all
```

将 MATLAB 的 Current Folder 设置为本 README 所在目录。如果另行保存了历史代码副本，不要将它们全部递归加入路径，以免同名函数遮蔽当前实现。

## 3. 快速开始

### 3.1 推荐先复算现有示例

以下代码读取完整存档到结构体，用同一套参数重新计算轨迹和凸轮，并另存一个新的坐标文件。它不会启动优化，也不会写回原 MAT 文件。

在根目录新建一个 MATLAB 脚本，粘贴并运行：

```matlab
S = load('shanghai_del4_max.mat');
sampleDensity = S.num;
endpointOrder = round(sqrt(sampleDensity / S.route_index));

[frontPath, ~, drivePath, ~, sensorPath, steering] = Simulation_cal( ...
    S.P0, S.route_index, sampleDensity, S.V_0, ...
    S.driving_wheel, S.driven_wheel, S.Z_sensor, endpointOrder, S.P);

[camTheory, camProfile] = cam_cal_elliptic( ...
    steering, S.c, S.thickness, drivePath, ...
    S.d_driving, S.i_total, S.d, S.r);

assert(isreal(camProfile) && all(isfinite(camProfile(:))), ...
    'Cam coordinates contain complex or non-finite values.');

figure;
subplot(1, 2, 1);
plot(frontPath(:,1), frontPath(:,2), 'r-', ...
     sensorPath(:,1), sensorPath(:,2), 'b-', ...
     S.P(:,1), S.P(:,2), 'ko');
axis equal; grid on;
xlabel('x / mm'); ylabel('y / mm');
legend('Front wheel', 'Sensor', 'Targets');

subplot(1, 2, 2);
plot(camTheory(:,1), camTheory(:,2), 'r-', ...
     camProfile(:,1), camProfile(:,2), 'b-');
axis equal; grid on;
xlabel('x / mm'); ylabel('y / mm');
legend('Theoretical cam', 'Corrected cam');

outFile = 'cam_preview_xyz.txt';
assert(~isfile(outFile), 'Output already exists; choose a new name.');
writematrix([camProfile, zeros(size(camProfile,1),1)], ...
    outFile, 'Delimiter', 'tab');
```

这里的坐标检查只是基本有效性检查，不代表已完成自交、加工公差或机构接触验证。

### 3.2 新建并优化一条路线

在编辑器中打开 `route_design_AI.m`，修改开头的配置区：

```matlab
data_name = 'my_route_01';  % 使用尚未存在的新名称
configuration = 0;
traing_times = 3;          % 首次先少量迭代检查配置
Venue_size = [0, 8000, 0, 8000];
```

接着设置 `P`、车辆几何及凸轮参数，保存脚本后运行：

```matlab
route_design_AI
```

程序会显示路线、转角与凸轮图，并生成 `my_route_01.mat` 和 `my_route_01_max.mat`。确认结果有效后再增加迭代次数。两份文件在优化期间持续覆盖写入。

**必须编辑脚本内的配置。**两个入口脚本均以 `clc; clear;` 开始，在命令窗口预先设置同名变量不会替代脚本配置。运行前保存其他需要保留的工作区数据。

当前优化脚本默认 `data_name='shanghai_test1'`，而导出脚本默认读取 `shanghai_del4`；两者并不自动同步。仓库已包含 `shanghai_test1.mat`，直接运行默认优化配置会加载其控制点并覆盖对应存档；独立试验请使用新名称。

### 3.3 继续已有路线

`configuration` 的实际行为如下：

| 设置 | 加载行为 | 使用注意 |
|---|---|---|
| 指定名称的普通 MAT 不存在 | 从 `P` 初始化控制点 | 此时不进入恢复分支，即使存在同名 `_max.mat` |
| 普通 MAT 存在，`configuration=0` | 加载 `<data_name>.mat` | 当前示例文件只含 `P0`、`optimal_gradient`；其余配置来自脚本 |
| 普通 MAT 存在，`configuration=1` | 加载 `<data_name>_max.mat` 全工作区 | 存档会覆盖脚本中的同名参数；需要两份文件均存在 |

恢复过程仍会重置部分优化变量，并从新一轮循环开始，不是逐条指令恢复。`configuration=0` 时修改 `P` 并不会自动重建已有 `P0`；目标点或几何变化较大时，优先使用新的 `data_name`。

## 4. 参数说明

长度统一使用 **mm**，内部角度使用**弧度**。车辆局部坐标以前轮为原点，`y` 轴沿初始车身前进方向，`x` 轴指向右侧。

### 4.1 几何与目标参数

| 参数 | 当前优化脚本值 | 含义 |
|---|---|---|
| `P` | 9×2 数组 | 按通过顺序排列的目标点，首末行分别为起终点 |
| `Venue_size` | `[0,6000,0,6000]` | 绘图范围，不是优化器的场地边界约束 |
| `driving_wheel` | `[94,-130]` | 主动轮相对前轮的位置 |
| `driven_wheel` | `[-54,-130]` | 从动轮相对前轮的位置 |
| `Z_sensor` | `[25,-53]` | 传感器相对前轮的位置 |
| `V_0` | `[-3,-1]` | 非零初始朝向向量，优化脚本会归一化 |
| `d_driving` | `180` | 主动轮直径 |
| `i_total` | `36` | 凸轮一周对应的主动轮转数 |
| `c` | `-33` | 转向机构的有符号几何间距；正负号影响凸轮生成 |
| `thickness` | `3` | 凸轮厚度 |
| `d` | `130` | 凸轮参考直径，不应直接解释为成品最小直径 |
| `r` | `2` | 与凸轮接触的曲面半径 |

当前默认目标点位于 `[0,6000,0,6000]` 范围内。更换目标点后，应同步检查绘图范围是否覆盖整条路线。

随附完整存档与脚本默认值不同：`shanghai_del4_max.mat` 的 `i_total=29.8`、`c=-20`、`Venue_size=[0,10000,0,10000]`。复现实例时应使用存档自身参数。

### 4.2 优化与采样参数

| 参数 | 默认值 | 含义 |
|---|---|---|
| `traing_times` | `399` | 迭代次数；保留源码拼写 |
| `control_num` | `5` | 首次初始化时，每段目标点连线的划分数 |
| `route_index` | `7` | 局部贝塞尔曲线阶数 |
| `num` | `20` | 每个控制点间隔的采样密度 |
| `gradient_power0` | `40` | 初始坐标扰动尺度 |
| `learning_efficiency` | `0.3` | 累积搜索方向的更新系数 |
| `power_P0` | `0.6` | 控制点平滑系数的初始值 |
| `perturbation_frequency` | `50` | 周期性调整评分指数的间隔 |
| `perturbation_power` | `2` | 评分指数扰动幅度 |
| `Scored_index` | `5+[0;0;3;0;0;0;0;0;0;0;0;0]` | 12 项特征的惩罚指数 |
| `E_*` | 见脚本 | 各特征的归一化尺度，不是硬性容差 |

当目标点数为 `m`、控制点数为 `n` 时，首次初始化有 `n=(m-1)*control_num+1`，轨迹采样点数为 `(n-1)*num+1`。当前默认配置为 **41 个控制点、821 个轨迹点、每轮 165 个候选**。此前报告中 46 个控制点、185 个候选对应其他历史存档。

输入应使用有限实数，避免重复相邻控制点、零方向向量和退化几何。`n` 必须大于 `route_index`，并为端点外推保留足够采样点。

## 5. 并行计算与评分

每轮固定当前控制点和评分参数，构造 `4*n+1` 个候选：每个控制坐标的正负扰动，以及一个仅含当前累积搜索方向的基准候选。候选独立完成轨迹、凸轮和评分计算，随后串行排序并更新状态。

变量 `threads_num` 实际是**候选任务数**，不是 CPU 线程数或 worker 数。外层迭代依赖上一轮状态，不能直接用同样方法并行化。

在已安装 Parallel Computing Toolbox 且机器资源允许时，可在运行脚本前配置本地进程池，例如 R2021a：

```matlab
pool = gcp('nocreate');
if isempty(pool)
    parpool('local', 4);
end
```

若已有池，上述代码保持现有 worker 数。需要串行调试时，可在脚本副本中把 `parfor thread=1:threads_num` 改为 `for thread=1:threads_num`。仓库未提供独立基准测试入口，也没有可核验的加速比结果。

总分为 `-sum((eigenvalues ./ E_eigenvalues).^scored_index)`，程序选取数值最大的总分。12 项特征依次为：前轮路径长度、控制多边形段长聚合、传感器途经点偏差、控制多边形转角及一阶/二阶差分聚合、前轮转角及一阶/二阶差分聚合、理论凸轮最大曲率指标、最大压力角、修正轮廓最大相邻切向变化。

`Squares_standard_sum(x)` 对长度为 N 的向量返回 `sqrt(N*sum(x.^2))`，不是通常的平方和或 RMS。角度差分没有除以时间，不能当作物理角速度或角加速度。评分尺度与指数会变化，跨轮原始总分不一定代表同一固定目标的收敛。

## 6. 原导出脚本与文件格式

在 `cam_calculat.m` 中配置：

```matlab
data_name = 'shanghai_del4';
save_name = 'cam_export_01';  % 建议使用新名称
format = '.txt';
num_route = 20;
```

保存并运行 `cam_calculat`。它读取 `<data_name>_max.mat`，重新计算凸轮，先写文件，再显示动画。导出采用 `w+` 模式，会覆盖同名文件；默认输出 `cam.txt`，请先改名或备份。

| 格式 | 实际内容 | 预期用途 |
|---|---|---|
| `.txt` | 每行 X、Y、0，制表符分隔，无表头 | SolidWorks XYZ 曲线等导入流程 |
| `.ibl` | `open/arclength/begin section/begin curve` 头部及序号、X、Y、0 | Creo 曲线导入 |
| `.scr` | `line` 命令后跟 X,Y 坐标 | AutoCAD 命令脚本 |

导出的是 `cam_virtual` 修正轮廓。所有 Z 坐标均为 0，需要在 CAD 中完成曲线、闭合检查和实体建模。`.scr` 不会写出显式闭合指令；导入方式、命令结束方式及格式兼容性需在目标 CAD 版本中确认。

## 7. 文件导航

| 文件 | 职责 |
|---|---|
| `route_design_AI.m` | 参数入口、候选构造、并行评价、更新、存档与绘图 |
| `cam_calculat.m` | 加载完整存档、导出凸轮及播放几何动画 |
| `Simulation_cal.m` | 汇总轨迹、车体几何及路线特征 |
| `all_routes_cal.m` | 推算车身、车轮、传感器位置及前轮转角 |
| `Bezier_superposition.m` / `Bezier_curve.m` | 加权叠加轨迹及单段/分段贝塞尔求值 |
| `cal_YangHui.m` / `cal_endpoints.m` | 二项式系数与端点外推 |
| `cam_cal_elliptic.m` | 推移量、回程段、理论凸轮与接触修正 |
| `score_cal.m` / `Squares_standard_sum.m` | 综合评分与特征聚合 |
| `distance_calculate.m` / `Point_line_distance.m` | 顺序途经点搜索及点到直线距离 |
| `Curvature_radius.m` / `Optimize_P0.m` | 三点曲率半径估计与控制点平滑 |
| `shanghai_del4.mat` | 精简存档：`P0`、`optimal_gradient` |
| `shanghai_del4_max.mat` | 全工作区快照；包含参数、中间数组及绘图句柄 |
| `cam.txt` / `cam.scr` | 已有导出文件；不能仅凭文件名确定对应配置 |
| `shanghai_test1.mat` / `shanghai_test1_max.mat` | 另一组示例：41 个控制点；完整存档采用 180 mm 主动轮、传动比 36、`c=-33` |

## 8. 已知限制与排查

1. **完整快照存在状态错位。**优化入口先更新 `P0` 并保存，再更新 `score_times`、轨迹和凸轮数组。因此同一 `_max.mat` 中的数组可能属于前一轮；最终评分项也可能尚未填写。分析或导出时，应从保存的 `P0` 重新计算，不要把保存数组与重算数组的差异直接归因于数值误差。`_max` 文件每轮覆盖，并非严格维护的历史最优结果。
2. **导出脚本的尾部输出变量对齐有误。**`Simulation_cal` 有 17 个输出，`cam_calculat.m` 的接收列表少了 `ddt_cp_sum`，导致后续几个特征变量名称错位；用于凸轮计算的前部输出不受这一错位影响。需要指标时，应按函数签名显式对齐，或使用 `~` 跳过不用的输出。
3. **改变导出密度时还应更新外推参数。**原导出脚本从存档读取 `index`，不会随着 `num_route` 自动重算。第 3.1 节的示例按当前密度重新计算了 `endpointOrder`。
4. **途经点指标是启发式近似。**原函数按顺序寻找局部最近位置，再求无限直线距离，不是全局有限线段最短距离；未找到的中间点还可能保留初始化的零值。验收路线时需独立核对每个目标点及顺序。
5. **几何退化可能导致 NaN、Inf 或复数。**检查重复点、零长度向量、`acos` 输入范围以及回程构造中的除法。源码只对部分计算做了截断或恢复，不能视为完整异常处理。
6. **凸轮一周与行程需要匹配。**函数在主动轮行程小于 `pi*d_driving*i_total` 时补回程；达到或超过该长度时会直接将已有行程归一化为一周，没有显式拒绝超行程配置。应独立核对传动比与实际闭合运动。
7. **循环和动画存在可移植性问题。**`all_routes_cal.m` 等使用了 `for i=2:size(points)` 这类非标量上界；若 MATLAB 报告冒号操作数问题，应明确写成 `size(points,1)`。导出脚本动画初始化还复用了之前循环留下的 `i`。需要自动化时，优先采用第 3.1 节的函数调用方式。
8. **图轴标题不完全代表实际物理量。**部分图把样本索引标为距离，或把弧度/推移量标为其他量。生成报告时应根据实际绘图变量重新设置坐标轴。

本模型主要描述离散几何与运动学，不包含打滑、结构弹性、传动间隙、电机动力学或光照变化。几何仿真通过不等于实车成功打卡；加工前还需检查轮廓自交、闭合性、接触条件和制造约束。

## 9. 文档核对与参考

本次更新为单份双语 README，英文在前、中文在后；通过静态阅读根目录全部 `.m` 文件和检查仓库示例 MAT 的字段、参数完成核对，未修改算法、运行完整优化或验证 CAD 导入。本文中的使用代码是基于当前接口编写的示例，尚未在 MATLAB 中执行。

- [原目录提供的使用示例视频](https://www.bilibili.com/video/BV1xF411N7Vf)
- [MATLAB parfor 文档](https://www.mathworks.com/help/parallel-computing/parfor.html)

当前目录未提供明确的软件许可证；对外发布前应补充适用许可及各来源材料的授权说明。
