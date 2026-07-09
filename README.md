# 纯轨迹规划器使用说明

本目录把轨迹规划器从完整闭环系统中单独抽出来，只负责生成轨迹。

它包含：

- constrained QP minimum-snap 轨迹规划；
- shaped fast endpoint KKT 轨迹规划；
- ideal-state realtime shaped fast endpoint KKT preview；
- 轨迹数据导出和画图。

它不包含：

- 6DOF 飞机动力学；
- 控制器；
- 控制分配；
- 支架/地面力模型；
- closed loop actual。

## 目录结构

```text
trajectory_planner_only/
  README.md
  planner/
    gj_solve_min_snap_constrained.m
    gj_solve_min_snap_endpoint_fast_shaped.m
    gj_eval_traj.m
  scripts/
    run_first_case_qp_kkt.m
  results/
```

## 默认工况

默认工况参考 `GJ3/整理/轨迹生成器规划说明.md` 中的第一个 40 deg 基础工况：

```text
theta0 = 40 deg
x_f    = 40 m
h_f    = 10 m
V_f    = 9 m/s
a_x,f  = 1 m/s^2
a_h,f  = 0 m/s^2
T      = 5.5 s
release thrust acceleration = 16.3 m/s^2
```

起点：

```text
p0 = [0, 0, 0]
v0 = [0, 0, 0]
j0 = [0, 0, 0]
```

起点加速度由初始支角和 release thrust acceleration 估算：

```text
ax0 = a_release cos(theta0)
ah0 = a_release sin(theta0) - g
a0  = [ax0, 0, -ah0]
```

当前默认 `a_release = 16.3 m/s^2` 来自当前飞机推力模型的 0.95 油门估算：

```text
mass              = 3.2 kg
single rotor T    = 6.519 N
8 rotor total T   = 52.152 N
a_release = T/m   = 52.152 / 3.2 = 16.3 m/s^2
```

该值表示 8 个主桨沿机体系 `+X_b` 的等效推力加速度。轨迹规划时再根据初始支角 `theta0` 分解到前向和向上方向，并在高度方向扣除重力。

终点：

```text
pf = [x_f, 0, -h_f]
vf = [V_f, 0, 0]
af = [a_x,f, 0, -a_h,f]
jf = [0, 0, 0]
```

## 运行方式

在 MATLAB 中运行：

```matlab
cd('D:\D_zx\26WORK\ShengTai\0610ST_mini_controller\GJ3\trajectory_planner_only')
run('scripts/run_first_case_qp_kkt.m')
```

输出目录：

```text
results/first_case_40deg_qp_kkt/
```

主要输出：

```text
figures/qp_vs_fast_kkt_first_case.png
data/constrained_qp_trajectory.csv
data/fast_endpoint_kkt_trajectory.csv
data/realtime_fast_kkt_preview_trajectory.csv
planner_only_summary.csv
planner_only_log.mat
README.md
```

## 三条轨迹

### 1. constrained QP

离线完整约束 minimum-snap QP。它使用：

- 起终点等式约束；
- 高度单调、x 单调；
- 航迹角约束；
- 后段速度约束；
- 高度进度上界；
- soft slack。

输出文件：

```text
data/constrained_qp_trajectory.csv
```

### 2. one-shot fast endpoint KKT

从初始点到终点只求解一次 shaped endpoint KKT。

它保留起终点等式边界，并用 soft shaping 近似 constrained QP 的主要语义：

```text
h_tau    = [0.78, 0.84]
h_ratio  = [0.76, 0.82]
vx_tau   = 0.60
vx_target = 8.6 m/s
```

输出文件：

```text
data/fast_endpoint_kkt_trajectory.csv
```

### 3. realtime fast endpoint KKT preview

这是规划器本体的实时滚动口径。

当前 standalone 版本不接真实飞机状态，而是假设每个重规划时刻的当前状态理想等于 constrained QP 同时刻的：

```text
p, v, a, j
```

然后每 `0.01 s` 从该当前状态到固定终点重新求解 shaped fast endpoint KKT，并取 `0.01 s` preview 点串成轨迹。当前脚本设置对应 100 Hz 规划频率。

输出文件：

```text
data/realtime_fast_kkt_preview_trajectory.csv
```

注意：终端剩余时间小于 `0.4 s` 时，脚本进入 terminal passthrough，不再人为构造 `0.4 s` 的假 horizon。这样可以避免终端附近出现非物理的 preview 速度陡降。

## 100 Hz 使用要求

当前 100 Hz 口径只要求实时循环执行 shaped fast endpoint KKT，不要求每个 0.01 s 都重新做完整 constrained QP 或完整 validator。

推荐工程流程：

```text
100 Hz:
  读取当前状态
  fast endpoint KKT 重规划
  采样 preview 点
  输出 p/v/a/j reference

20 Hz 或更低频:
  validator / 约束一致性检查
  健康状态监控
  必要时调整 shaping、终端目标或降级策略

离线或任务开始前:
  constrained QP 基准规划
  语义约束验证
  参数整定
```

在当前 MATLAB planner-only 测试中，40 deg 默认工况、`0.01 s` 重规划的 fast KKT 单步求解统计为：

```text
平均求解时间    0.387 ms
95% 分位        0.825 ms
最大求解时间    11.716 ms
超过 10 ms 步数 1 / 551
```

唯一超过 10 ms 的点出现在 `t = 0 s` 首步，主要是 MATLAB 冷启动/JIT/函数预热尖峰。实际机载或稳定实时测试前应先 warm-up 一次 KKT 求解，实时循环中避免动态分配、画图、写 CSV、完整 validator 和 constrained QP 求解。

因此，本目录证明的是：在该工况和当前实现下，fast KKT 轨迹规划器本体具备 100 Hz 求解潜力；完整上机还需要低频 validator、预热、固定内存和工程化代码生成/封装。

## 无人机如何使用这个 planner

真实机载使用时，不需要 constrained QP 作为当前状态来源。无人机应使用传感器/估计器回传的实时状态。

每个控制周期或规划周期执行：

```text
输入：
  当前状态:
    p_now = [x, y, z]        NED 位置
    v_now = [vx, vy, vz]     NED 速度
    a_now = [ax, ay, az]     NED 加速度，可由估计或差分获得
    j_now = [jx, jy, jz]     NED jerk，可设为 0 或估计值

  终端目标:
    pf = [x_f, 0, -h_f]
    vf = [V_f, 0, 0]
    af = [a_x,f, 0, -a_h,f]
    jf = [0, 0, 0]

  任务参数:
    T_remain
    preview_time
    shaping 参数

求解：
  traj = gj_solve_min_snap_endpoint_fast_shaped(p_now, pf, T_remain, opts)

采样：
  sample = gj_eval_traj(traj, preview_time)

输出给下游控制器：
  p_ref = sample.p
  v_ref = sample.v
  a_ref = sample.a
  j_ref = sample.j
```

也就是说，机载 planner 的核心输入是“当前飞行器状态 + 固定终点状态”，核心输出是“短期预瞄轨迹点”。

## 单独拷贝能否运行

可以。只把 `trajectory_planner_only` 这个文件夹拷贝出去后，它可以独立完成“给定工况和语义参数 -> 输出 constrained QP、one-shot fast KKT、realtime fast KKT preview 轨迹”的工作。

独立运行需要：

```text
MATLAB
Optimization Toolbox，用于 constrained QP 中的 quadprog
本目录下的 planner/ 和 scripts/
```

其中：

- `constrained QP` 需要 `quadprog`，用于离线基准轨迹和语义约束验证；
- `fast endpoint KKT` 和 `realtime preview` 使用本目录内的解析/KKT 求解代码；
- 若只在机载端实时运行 fast KKT，可不在实时循环中调用 constrained QP；
- `results/` 不是运行依赖，只是当前算例输出，可以删除后重新生成。

修改工况时，主要改 `scripts/run_first_case_qp_kkt.m` 中的：

```text
theta_initial_deg
x_f, h_f
V_f
a_x_f, a_h_f
T
shaping / semantic constraint 参数
replan_interval_s
command_preview_s
```

脚本会重新生成轨迹 CSV、图像、summary 和 MAT 日志。

## 输出 CSV 字段

三个轨迹 CSV 均包含：

```text
t_s
x_m
h_m
vx_mps
hdot_mps
path_speed_mps
ax_mps2
ah_mps2
accel_norm_mps2
jx_mps3
jh_mps3
```

其中：

- `h_m = -z_ned`
- `hdot_mps = -vz_ned`
- `ah_mps2 = -az_ned`

## 当前结果解读

当前默认工况下，realtime preview 通常比 one-shot KKT 更贴近 constrained QP，因为它不断基于当前状态重新规划。

该目录的判断口径是：

```text
constrained QP vs one-shot fast KKT
constrained QP vs realtime fast KKT preview
```

它只评价轨迹规划器本体，不评价控制器是否能跟踪。
