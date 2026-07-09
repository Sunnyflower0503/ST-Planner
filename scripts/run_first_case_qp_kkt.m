%% run_first_case_qp_kkt.m
% Standalone trajectory-planner-only demo.
%
% This script only runs:
%   1) constrained QP minimum-snap planner
%   2) shaped fast endpoint KKT planner
%   3) ideal-state realtime shaped fast endpoint KKT preview
%
% It does not use the aircraft dynamics, controller, actuator allocation,
% ground/support model, or closed-loop simulation.

clear; clc; close all;

this_dir = fileparts(mfilename('fullpath'));
root_dir = fileparts(this_dir);
addpath(fullfile(root_dir, 'planner'), '-begin');

out_dir = fullfile(root_dir, 'results', 'first_case_40deg_qp_kkt');
fig_dir = fullfile(out_dir, 'figures');
data_dir = fullfile(out_dir, 'data');
if ~exist(fig_dir, 'dir'), mkdir(fig_dir); end
if ~exist(data_dir, 'dir'), mkdir(data_dir); end

cfg = planner_config();
c = first_case_40deg();
param = struct('g', 9.81);

fprintf('===== Planner-only first case: QP vs fast endpoint KKT =====\n');
fprintf('theta0 %.1f deg, xf %.1f m, hf %.1f m, Vf %.1f m/s, axf %.1f m/s^2, T %.2f s\n', ...
    c.theta_initial_deg, c.x_f_m, c.h_f_m, c.V_f_mps, c.a_x_f_mps2, c.t_total_s);

t = linspace(0, c.t_total_s, cfg.n_eval);
[traj_qp, exitflag_qp, info_qp] = solve_constrained(c, cfg, param);
sample_qp = gj_eval_traj(traj_qp, t);
curve_qp = curve_from_sample(sample_qp);

traj_kkt = solve_endpoint_fast_shaped(c, cfg, param);
sample_kkt = gj_eval_traj(traj_kkt, t);
curve_kkt = curve_from_sample(sample_kkt);

[curve_rt_preview, rt_preview_info] = solve_realtime_preview_from_qp(traj_qp, c, cfg, param);

fit = fit_metrics(curve_qp, curve_kkt, c, cfg);
fit_rt = fit_metrics_interp(curve_qp, curve_rt_preview, c, cfg);
summary = summary_table(c, exitflag_qp, info_qp, fit, fit_rt);

writetable(curve_table(curve_qp), fullfile(data_dir, 'constrained_qp_trajectory.csv'));
writetable(curve_table(curve_kkt), fullfile(data_dir, 'fast_endpoint_kkt_trajectory.csv'));
writetable(curve_table(curve_rt_preview), fullfile(data_dir, 'realtime_fast_kkt_preview_trajectory.csv'));
writetable(summary, fullfile(out_dir, 'planner_only_summary.csv'));
save(fullfile(out_dir, 'planner_only_log.mat'), ...
    'cfg', 'c', 'traj_qp', 'sample_qp', 'curve_qp', ...
    'traj_kkt', 'sample_kkt', 'curve_kkt', ...
    'curve_rt_preview', 'rt_preview_info', 'fit', 'fit_rt', 'summary');

plot_results(c, curve_qp, curve_kkt, curve_rt_preview, fit, fit_rt, fig_dir);
write_report(c, cfg, summary, fit, fit_rt, out_dir);

disp(summary);
fprintf('Saved planner-only results to: %s\n', out_dir);
fprintf('===== Planner-only first case complete =====\n');

function cfg = planner_config()
cfg = struct();
cfg.poly_order = 9;
cfg.n_eval = 401;
cfg.n_check = 140;
cfg.coeff_regularization = 1e-7;
cfg.release.thrust_accel_mps2 = 16.3;

cfg.shaped.h_tau = [0.78, 0.84];
cfg.shaped.h_ratio = [0.76, 0.82];
cfg.shaped.h_weight = [4.0e7, 4.0e7];
cfg.shaped.vx_enable = true;
cfg.shaped.vx_tau = 0.60;
cfg.shaped.vx_target_mps = 8.6;
cfg.shaped.vx_weight = 6.0e7;

cfg.h_upper_margin = 0.25;
cfg.h_upper_progress_enable = true;
cfg.h_upper_progress_tau = 0.88;
cfg.h_upper_progress_ratio = 0.95;
cfg.h_upper_progress_power = 0.55;
cfg.x_monotonic = true;
cfg.h_monotonic = true;
cfg.ax_min_initial = 0;
cfg.ax_min_frac = 0.12;
cfg.slack_weight = 1e5;
cfg.qp_slack_pass_tol = 1e-3;
cfg.kkt_fit_score_zero_rmse_m = 2.0;
cfg.realtime.replan_interval_s = 0.01;
cfg.realtime.command_preview_s = 0.01;
cfg.realtime.min_horizon_s = 0.4;
cfg.realtime.terminal_passthrough_enable = true;
end

function c = first_case_40deg()
c = struct();
c.theta_initial_deg = 40;
c.x_f_m = 40;
c.h_f_m = 10;
c.V_f_mps = 9;
c.a_x_f_mps2 = 1;
c.a_h_f_mps2 = 0;
c.t_total_s = 5.5;
c.h_shape_p = 1.15;
c.gamma_max_deg = 65;
c.release_thrust_accel_mps2 = 16.3;
end

function [traj, exitflag, info] = solve_constrained(c, cfg, param)
p0 = [0, 0, 0];
pf = [c.x_f_m, 0, -c.h_f_m];
release = release_accel_boundary(c, cfg, param);
opts = endpoint_opts(c, cfg, release);
ineq = struct();
ineq.n_check = cfg.n_check;
ineq.h_start_m = 0;
ineq.h_f_m = c.h_f_m;
ineq.h_shape_p = c.h_shape_p;
ineq.h_upper_margin = cfg.h_upper_margin;
ineq.h_upper_progress_enable = cfg.h_upper_progress_enable;
ineq.h_upper_progress_tau = cfg.h_upper_progress_tau;
ineq.h_upper_progress_ratio = cfg.h_upper_progress_ratio;
ineq.h_upper_progress_power = cfg.h_upper_progress_power;
ineq.gamma_max_deg = c.gamma_max_deg;
ineq.x_monotonic = cfg.x_monotonic;
ineq.h_monotonic = cfg.h_monotonic;
ineq.ax_min_initial = cfg.ax_min_initial;
ineq.ax_min_frac = cfg.ax_min_frac;
ineq.vx_min_late = 8.0;
ineq.vx_min_late_start_frac = 0.6;
ineq.slack_weight = cfg.slack_weight;
ineq.slack_pass_tol = cfg.qp_slack_pass_tol;
[traj, exitflag, info] = gj_solve_min_snap_constrained(p0, pf, c.t_total_s, opts, ineq);
end

function traj = solve_endpoint_fast_shaped(c, cfg, param)
p0 = [0, 0, 0];
pf = [c.x_f_m, 0, -c.h_f_m];
release = release_accel_boundary(c, cfg, param);
opts = endpoint_opts(c, cfg, release);
opts.shape_h_tau = cfg.shaped.h_tau;
opts.shape_h_ratio = cfg.shaped.h_ratio;
opts.shape_h_weight = cfg.shaped.h_weight;
opts.shape_vx_enable = cfg.shaped.vx_enable;
opts.shape_vx_tau = cfg.shaped.vx_tau;
opts.shape_vx_target_mps = cfg.shaped.vx_target_mps;
opts.shape_vx_weight = cfg.shaped.vx_weight;
traj = gj_solve_min_snap_endpoint_fast_shaped(p0, pf, c.t_total_s, opts);
end

function [curve, info] = solve_realtime_preview_from_qp(traj_qp, c, cfg, param)
dt = cfg.realtime.replan_interval_s;
t_grid = 0:dt:c.t_total_s;
if t_grid(end) < c.t_total_s
    t_grid(end + 1) = c.t_total_s;
end

n = numel(t_grid);
p = nan(3, n);
v = nan(3, n);
a = nan(3, n);
j = nan(3, n);
solve_time_s = nan(1, n);
preview_t = nan(1, n);
T_remain_log = nan(1, n);

target = struct();
target.pf = [c.x_f_m, 0, -c.h_f_m];
target.vf = [c.V_f_mps, 0, 0];
target.af = [c.a_x_f_mps2, 0, -c.a_h_f_mps2];
target.jf = [0, 0, 0];

state_qp = gj_eval_traj(traj_qp, t_grid);
for k = 1:n
    t_now = t_grid(k);
    T_actual = max(c.t_total_s - t_now, 0);
    if cfg.realtime.terminal_passthrough_enable ...
            && T_actual <= cfg.realtime.min_horizon_s
        query_t = min(t_now + cfg.realtime.command_preview_s, c.t_total_s);
        sample = gj_eval_traj(traj_qp, query_t);
        p(:, k) = sample.p(:, 1);
        v(:, k) = sample.v(:, 1);
        a(:, k) = sample.a(:, 1);
        j(:, k) = sample.j(:, 1);
        preview_t(k) = query_t - t_now;
        T_remain_log(k) = T_actual;
        solve_time_s(k) = 0;
        continue;
    end

    T_remain = T_actual;
    local_preview_t = min(max(cfg.realtime.command_preview_s, eps), 0.25 * T_remain);

    opts = struct();
    opts.v0 = state_qp.v(:, k).';
    opts.a0 = state_qp.a(:, k).';
    opts.j0 = state_qp.j(:, k).';
    opts.vf = target.vf;
    opts.af = target.af;
    opts.jf = target.jf;
    opts.poly_order = cfg.poly_order;
    opts.coeff_regularization = cfg.coeff_regularization;
    opts.enforce_end_accel = true;
    opts.shape_h_tau = cfg.shaped.h_tau;
    opts.shape_h_ratio = cfg.shaped.h_ratio;
    opts.shape_h_weight = cfg.shaped.h_weight;
    opts.shape_vx_enable = cfg.shaped.vx_enable;
    opts.shape_vx_tau = cfg.shaped.vx_tau;
    opts.shape_vx_target_mps = cfg.shaped.vx_target_mps;
    opts.shape_vx_weight = cfg.shaped.vx_weight;

    t_solve = tic;
    traj = gj_solve_min_snap_endpoint_fast_shaped( ...
        state_qp.p(:, k).', target.pf, T_remain, opts);
    solve_time_s(k) = toc(t_solve);
    sample = gj_eval_traj(traj, local_preview_t);
    p(:, k) = sample.p(:, 1);
    v(:, k) = sample.v(:, 1);
    a(:, k) = sample.a(:, 1);
    j(:, k) = sample.j(:, 1);
    preview_t(k) = local_preview_t;
    T_remain_log(k) = T_remain;
end

sample_rt = struct('t', t_grid, 'p', p, 'v', v, 'a', a, 'j', j);
curve = curve_from_sample(sample_rt);
info = struct();
info.t = t_grid;
info.preview_t = preview_t;
info.T_remain = T_remain_log;
info.solve_time_s = solve_time_s;
info.mean_solve_time_s = mean(solve_time_s, 'omitnan');
info.max_solve_time_s = max(solve_time_s, [], 'omitnan');
end

function opts = endpoint_opts(c, cfg, release)
opts = struct();
opts.v0 = [0, 0, 0];
opts.a0 = release.a_start_ned(:).';
opts.j0 = [0, 0, 0];
opts.vf = [c.V_f_mps, 0, 0];
opts.af = [c.a_x_f_mps2, 0, -c.a_h_f_mps2];
opts.jf = [0, 0, 0];
opts.poly_order = cfg.poly_order;
opts.coeff_regularization = cfg.coeff_regularization;
opts.enforce_end_accel = true;
end

function release = release_accel_boundary(c, cfg, param)
theta0 = deg2rad(c.theta_initial_deg);
at = cfg.release.thrust_accel_mps2;
if isfield(c, 'release_thrust_accel_mps2') && c.release_thrust_accel_mps2 > 0
    at = c.release_thrust_accel_mps2;
end
ax0 = at * cos(theta0);
ah0 = at * sin(theta0) - param.g;
release = struct();
release.a_start_ned = [ax0; 0; -ah0];
release.initial_forward_accel_mps2 = ax0;
release.initial_up_accel_mps2 = ah0;
end

function curve = curve_from_sample(sample)
vx = sample.v(1, :);
hdot = -sample.v(3, :);
ax = sample.a(1, :);
ah = -sample.a(3, :);
curve = struct();
curve.t = sample.t(:).';
curve.x = sample.p(1, :);
curve.h = -sample.p(3, :);
curve.vx = vx;
curve.hdot = hdot;
curve.speed = hypot(vx, hdot);
curve.ax = ax;
curve.ah = ah;
curve.accel = hypot(ax, ah);
curve.jx = sample.j(1, :);
curve.jh = -sample.j(3, :);
end

function tbl = curve_table(curve)
tbl = table(curve.t(:), curve.x(:), curve.h(:), ...
    curve.vx(:), curve.hdot(:), curve.speed(:), ...
    curve.ax(:), curve.ah(:), curve.accel(:), ...
    curve.jx(:), curve.jh(:), ...
    'VariableNames', {'t_s', 'x_m', 'h_m', ...
    'vx_mps', 'hdot_mps', 'path_speed_mps', ...
    'ax_mps2', 'ah_mps2', 'accel_norm_mps2', ...
    'jx_mps3', 'jh_mps3'});
end

function fit = fit_metrics(qp, kkt, c, cfg)
dx = kkt.x - qp.x;
dh = kkt.h - qp.h;
dv = kkt.speed - qp.speed;
da = kkt.accel - qp.accel;
path_error = hypot(dx, dh);
fit = struct();
fit.rmse_x_m = rms_local(dx);
fit.rmse_h_m = rms_local(dh);
fit.rmse_speed_mps = rms_local(dv);
fit.rmse_accel_mps2 = rms_local(da);
fit.rmse_path_m = rms_local(path_error);
fit.max_abs_x_m = max(abs(dx));
fit.max_abs_h_m = max(abs(dh));
fit.endpoint_x_error_m = dx(end);
fit.endpoint_h_error_m = dh(end);
fit.norm_rmse_path_pct = 100 * fit.rmse_path_m / max(hypot(c.x_f_m, c.h_f_m), eps);
fit.score = 100 * max(0, 1 - fit.rmse_path_m / cfg.kkt_fit_score_zero_rmse_m);
end

function fit = fit_metrics_interp(qp, kkt, c, cfg)
qp_i = struct();
qp_i.t = kkt.t;
qp_i.x = interp1(qp.t, qp.x, kkt.t, 'linear', 'extrap');
qp_i.h = interp1(qp.t, qp.h, kkt.t, 'linear', 'extrap');
qp_i.speed = interp1(qp.t, qp.speed, kkt.t, 'linear', 'extrap');
qp_i.accel = interp1(qp.t, qp.accel, kkt.t, 'linear', 'extrap');
fit = fit_metrics(qp_i, kkt, c, cfg);
end

function tbl = summary_table(c, exitflag_qp, info_qp, fit, fit_rt)
tbl = table(c.theta_initial_deg, c.x_f_m, c.h_f_m, c.V_f_mps, ...
    c.a_x_f_mps2, c.a_h_f_mps2, c.t_total_s, ...
    exitflag_qp, get_info_scalar(info_qp, 'max_slack'), ...
    fit.rmse_path_m, fit.rmse_x_m, fit.rmse_h_m, ...
    fit.rmse_speed_mps, fit.rmse_accel_mps2, ...
    fit.max_abs_x_m, fit.max_abs_h_m, ...
    fit.endpoint_x_error_m, fit.endpoint_h_error_m, ...
    fit.norm_rmse_path_pct, fit.score, ...
    fit_rt.rmse_path_m, fit_rt.rmse_x_m, fit_rt.rmse_h_m, ...
    fit_rt.rmse_speed_mps, fit_rt.rmse_accel_mps2, ...
    fit_rt.max_abs_x_m, fit_rt.max_abs_h_m, ...
    fit_rt.endpoint_x_error_m, fit_rt.endpoint_h_error_m, ...
    fit_rt.norm_rmse_path_pct, fit_rt.score, ...
    'VariableNames', {'theta_initial_deg', 'x_f_m', 'h_f_m', 'V_f_mps', ...
    'a_x_f_mps2', 'a_h_f_mps2', 'T_s', ...
    'qp_exitflag', 'qp_max_slack', ...
    'kkt_rmse_path_m', 'kkt_rmse_x_m', 'kkt_rmse_h_m', ...
    'kkt_rmse_speed_mps', 'kkt_rmse_accel_mps2', ...
    'kkt_max_abs_x_m', 'kkt_max_abs_h_m', ...
    'kkt_endpoint_x_error_m', 'kkt_endpoint_h_error_m', ...
    'kkt_norm_rmse_path_pct', 'kkt_fit_score', ...
    'rt_preview_rmse_path_m', 'rt_preview_rmse_x_m', 'rt_preview_rmse_h_m', ...
    'rt_preview_rmse_speed_mps', 'rt_preview_rmse_accel_mps2', ...
    'rt_preview_max_abs_x_m', 'rt_preview_max_abs_h_m', ...
    'rt_preview_endpoint_x_error_m', 'rt_preview_endpoint_h_error_m', ...
    'rt_preview_norm_rmse_path_pct', 'rt_preview_fit_score'});
end

function plot_results(c, qp, kkt, rt_preview, fit, fit_rt, fig_dir)
colors = lines(3);
fig = figure('Color', 'w', 'Position', [80, 70, 1220, 850]);
tiledlayout(2, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

nexttile;
plot(qp.x, qp.h, '-', 'Color', colors(1, :), 'LineWidth', 1.8); hold on;
plot(kkt.x, kkt.h, '--', 'Color', colors(2, :), 'LineWidth', 1.6);
plot(rt_preview.x, rt_preview.h, ':', 'Color', colors(3, :), 'LineWidth', 1.9);
plot(c.x_f_m, c.h_f_m, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 4);
grid on; xlabel('x / m'); ylabel('h / m'); title('Path: h vs x');
legend('constrained QP', 'one-shot fast KKT', 'realtime KKT preview', 'target', 'Location', 'best');

nexttile;
plot(qp.t, qp.h, '-', 'Color', colors(1, :), 'LineWidth', 1.8); hold on;
plot(kkt.t, kkt.h, '--', 'Color', colors(2, :), 'LineWidth', 1.6);
plot(rt_preview.t, rt_preview.h, ':', 'Color', colors(3, :), 'LineWidth', 1.9);
grid on; xlabel('time / s'); ylabel('h / m'); title('Height vs time');
legend('constrained QP', 'one-shot fast KKT', 'realtime KKT preview', 'Location', 'best');

nexttile;
plot(qp.t, qp.speed, '-', 'Color', colors(1, :), 'LineWidth', 1.8); hold on;
plot(kkt.t, kkt.speed, '--', 'Color', colors(2, :), 'LineWidth', 1.6);
plot(rt_preview.t, rt_preview.speed, ':', 'Color', colors(3, :), 'LineWidth', 1.9);
grid on; xlabel('time / s'); ylabel('path speed / m/s'); title('Path speed');
legend('constrained QP', 'one-shot fast KKT', 'realtime KKT preview', 'Location', 'best');

nexttile;
plot(qp.t, qp.accel, '-', 'Color', colors(1, :), 'LineWidth', 1.8); hold on;
plot(kkt.t, kkt.accel, '--', 'Color', colors(2, :), 'LineWidth', 1.6);
plot(rt_preview.t, rt_preview.accel, ':', 'Color', colors(3, :), 'LineWidth', 1.9);
grid on; xlabel('time / s'); ylabel('|a| / m/s^2'); title('Acceleration norm');
legend('constrained QP', 'one-shot fast KKT', 'realtime KKT preview', 'Location', 'best');

sgtitle(sprintf('Planner only: 40 deg first case, one-shot RMSE %.3f m, realtime preview RMSE %.3f m', ...
    fit.rmse_path_m, fit_rt.rmse_path_m));
saveas(fig, fullfile(fig_dir, 'qp_vs_fast_kkt_first_case.png'));
savefig(fig, fullfile(fig_dir, 'qp_vs_fast_kkt_first_case.fig'));
end

function write_report(c, cfg, summary, fit, fit_rt, out_dir)
txt = strings(0, 1);
txt(end + 1) = "# 纯轨迹规划器输出说明";
txt(end + 1) = "";
txt(end + 1) = "本目录只运行轨迹规划器，不包含 6DOF 动力学、控制器、控制分配、支架/地面力模型和 closed loop actual。";
txt(end + 1) = "";
txt(end + 1) = "## 运行命令";
txt(end + 1) = "";
txt(end + 1) = "在 MATLAB 中进入本仓库根目录后运行：";
txt(end + 1) = "";
txt(end + 1) = "```matlab";
txt(end + 1) = "run('scripts/run_first_case_qp_kkt.m')";
txt(end + 1) = "```";
txt(end + 1) = "";
txt(end + 1) = "## 工况";
txt(end + 1) = "";
txt(end + 1) = sprintf("- 初始支角 `theta0 = %.1f deg`", c.theta_initial_deg);
txt(end + 1) = sprintf("- 终端位置 `x_f = %.1f m`", c.x_f_m);
txt(end + 1) = sprintf("- 终端高度 `h_f = %.1f m`", c.h_f_m);
txt(end + 1) = sprintf("- 终端速度 `V_f = %.1f m/s`", c.V_f_mps);
txt(end + 1) = sprintf("- 终端水平加速度 `a_x,f = %.1f m/s^2`", c.a_x_f_mps2);
txt(end + 1) = sprintf("- 终端垂向加速度 `a_h,f = %.1f m/s^2`", c.a_h_f_mps2);
txt(end + 1) = sprintf("- 总时间 `T = %.2f s`", c.t_total_s);
txt(end + 1) = sprintf("- release thrust acceleration `%.1f m/s^2`", c.release_thrust_accel_mps2);
txt(end + 1) = "";
txt(end + 1) = "该工况是本 standalone planner 仓库内置的第一个 40 deg 基础工况。";
txt(end + 1) = "";
txt(end + 1) = "## 约束与语义";
txt(end + 1) = "";
txt(end + 1) = sprintf("- 多项式阶数：`%d`", cfg.poly_order);
txt(end + 1) = sprintf("- QP 检查点数：`%d`", cfg.n_check);
txt(end + 1) = "- 起点：`p0=[0,0,0]`, `v0=[0,0,0]`, `j0=[0,0,0]`";
txt(end + 1) = "- 起点加速度由初始支角和 release thrust acceleration 估算。";
txt(end + 1) = "- 终点：`pf=[x_f,0,-h_f]`, `vf=[V_f,0,0]`, `af=[a_x,f,0,-a_h,f]`, `jf=[0,0,0]`";
txt(end + 1) = "- QP 使用高度单调、x 单调、航迹角约束、后段速度约束和高度进度上界。";
txt(end + 1) = sprintf("- 高度进度上界：`tau=%.2f`, `ratio=%.2f`, `power=%.2f`", ...
    cfg.h_upper_progress_tau, cfg.h_upper_progress_ratio, cfg.h_upper_progress_power);
txt(end + 1) = sprintf("- fast endpoint KKT shaping：`h_tau=[%.2f %.2f]`, `h_ratio=[%.2f %.2f]`, `vx_tau=%.2f`, `vx_target=%.1f m/s`", ...
    cfg.shaped.h_tau(1), cfg.shaped.h_tau(2), cfg.shaped.h_ratio(1), cfg.shaped.h_ratio(2), ...
    cfg.shaped.vx_tau, cfg.shaped.vx_target_mps);
txt(end + 1) = "";
txt(end + 1) = "## 输出文件";
txt(end + 1) = "";
txt(end + 1) = "- `figures/qp_vs_fast_kkt_first_case.png`：QP、one-shot fast endpoint KKT 与 realtime KKT preview 对比图。";
txt(end + 1) = "- `data/constrained_qp_trajectory.csv`：QP 轨迹数据。";
txt(end + 1) = "- `data/fast_endpoint_kkt_trajectory.csv`：one-shot fast endpoint KKT 轨迹数据。";
txt(end + 1) = "- `data/realtime_fast_kkt_preview_trajectory.csv`：realtime fast endpoint KKT preview 轨迹数据。";
txt(end + 1) = "- `planner_only_summary.csv`：贴合指标。";
txt(end + 1) = "- `planner_only_log.mat`：MATLAB 完整日志。";
txt(end + 1) = "";
txt(end + 1) = "## 轨迹图像";
txt(end + 1) = "";
txt(end + 1) = "![QP vs fast KKT](figures/qp_vs_fast_kkt_first_case.png)";
txt(end + 1) = "";
txt(end + 1) = "## 本次结果";
txt(end + 1) = "";
txt(end + 1) = sprintf("- QP exitflag：`%g`", summary.qp_exitflag(1));
txt(end + 1) = sprintf("- QP max slack：`%.6g`", summary.qp_max_slack(1));
txt(end + 1) = sprintf("- KKT path RMSE：`%.6f m`", fit.rmse_path_m);
txt(end + 1) = sprintf("- KKT x RMSE：`%.6f m`", fit.rmse_x_m);
txt(end + 1) = sprintf("- KKT h RMSE：`%.6f m`", fit.rmse_h_m);
txt(end + 1) = sprintf("- KKT speed RMSE：`%.6f m/s`", fit.rmse_speed_mps);
txt(end + 1) = sprintf("- KKT endpoint x error：`%.6g m`", fit.endpoint_x_error_m);
txt(end + 1) = sprintf("- KKT endpoint h error：`%.6g m`", fit.endpoint_h_error_m);
txt(end + 1) = sprintf("- KKT fit score：`%.2f / 100`", fit.score);
txt(end + 1) = "";
txt(end + 1) = "## realtime preview 结果";
txt(end + 1) = "";
txt(end + 1) = sprintf("- realtime replan interval：`%.3f s`", cfg.realtime.replan_interval_s);
txt(end + 1) = sprintf("- realtime command preview：`%.3f s`", cfg.realtime.command_preview_s);
txt(end + 1) = sprintf("- terminal passthrough threshold：`%.3f s`", cfg.realtime.min_horizon_s);
txt(end + 1) = "- realtime preview 假设当前状态理想等于 QP 同时刻的 `p/v/a/j`，不接入真实动力学和控制器。";
txt(end + 1) = "- 当剩余时间小于最小 horizon 时，不再构造人工 0.4 s 终端重规划，而是直接输出终端短预瞄状态，避免终端速度假跳变。";
txt(end + 1) = sprintf("- realtime preview path RMSE：`%.6f m`", fit_rt.rmse_path_m);
txt(end + 1) = sprintf("- realtime preview x RMSE：`%.6f m`", fit_rt.rmse_x_m);
txt(end + 1) = sprintf("- realtime preview h RMSE：`%.6f m`", fit_rt.rmse_h_m);
txt(end + 1) = sprintf("- realtime preview speed RMSE：`%.6f m/s`", fit_rt.rmse_speed_mps);
txt(end + 1) = sprintf("- realtime preview endpoint x error：`%.6g m`", fit_rt.endpoint_x_error_m);
txt(end + 1) = sprintf("- realtime preview endpoint h error：`%.6g m`", fit_rt.endpoint_h_error_m);
txt(end + 1) = sprintf("- realtime preview fit score：`%.2f / 100`", fit_rt.score);
txt(end + 1) = "";
txt(end + 1) = "## 注意";
txt(end + 1) = "";
txt(end + 1) = "这里输出了两种 KKT 口径：one-shot shaped endpoint KKT 和 ideal-state realtime KKT preview。realtime preview 只验证规划器滚动重规划本体，不包含 controller command ref 和 actual closed loop。";

fid = fopen(fullfile(out_dir, 'README.md'), 'w');
fprintf(fid, '%s\n', txt);
fclose(fid);
end

function y = rms_local(x)
x = x(isfinite(x));
if isempty(x)
    y = NaN;
else
    y = sqrt(mean(x .^ 2));
end
end

function v = get_info_scalar(s, name)
if isstruct(s) && isfield(s, name) && isscalar(s.(name))
    v = s.(name);
else
    v = NaN;
end
end
