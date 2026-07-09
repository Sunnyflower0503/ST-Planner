function [traj, exitflag, info] = gj_solve_min_snap_constrained(p0, pf, T, opts, ineq_spec)
%GJ_SOLVE_MIN_SNAP_CONSTRAINED Endpoint minimum-snap with sampled inequalities.
%
% The trajectory is a single configurable-order polynomial segment in NED.
% The decision vector stacks all polynomial coefficients as [x; y; z; slack].
% Sampled path constraints are softened by nonnegative slack variables so a
% sweep can rank near-feasible cases instead of failing hard.

if nargin < 4
    opts = struct();
end
if nargin < 5
    ineq_spec = struct();
end

p0 = double(p0(:)');
pf = double(pf(:)');
T = double(T);
if numel(p0) ~= 3 || numel(pf) ~= 3
    error('gj_solve_min_snap_constrained:BadPosition', 'p0 and pf must be 1x3 NED vectors.');
end
if ~(isfinite(T) && T > 0)
    error('gj_solve_min_snap_constrained:BadTime', 'T must be positive.');
end
if exist('quadprog', 'file') ~= 2
    error('gj_solve_min_snap_constrained:MissingQuadprog', ...
        'quadprog is required. Install/enable Optimization Toolbox.');
end

dim = 3;
order = get_scalar_alias(opts, {'poly_order', 'order'}, 9);
order = round(order);
if order < 7
    error('gj_solve_min_snap_constrained:BadOrder', ...
        'Polynomial order must be at least 7 for minimum-snap endpoint constraints.');
end
n_coef = order + 1;
n_poly = dim * n_coef;

v0 = get_vec_alias(opts, {'v0', 'v_start'}, dim);
a0 = get_vec_alias(opts, {'a0', 'a_start'}, dim);
j0 = get_vec_alias(opts, {'j0', 'j_start'}, dim);
vf = get_vec_alias(opts, {'vf', 'v_end'}, dim);
af = get_vec_alias(opts, {'af', 'a_end'}, dim);
jf = get_vec_alias(opts, {'jf', 'j_end'}, dim);
coeff_regularization = get_scalar(opts, 'coeff_regularization', 1e-8);
enforce_end_accel = get_bool(opts, 'enforce_end_accel', false);
enforce_end_jerk = get_bool(opts, 'enforce_end_jerk', false);
end_accel_weight = get_scalar(opts, 'end_accel_weight', 20.0);
end_jerk_weight = get_scalar(opts, 'end_jerk_weight', 0.2);
qp_algorithm = get_char(opts, 'qp_algorithm', 'interior-point-convex');
if isfield(opts, 'x0')
    x0 = double(opts.x0(:));
else
    x0 = [];
end

spec = default_ineq_spec(ineq_spec, -pf(3));
Q1 = build_snap_cost(n_coef, T);
Hc = blkdiag(Q1, Q1, Q1) + coeff_regularization * eye(n_poly);
Q_snap = blkdiag(Q1, Q1, Q1);
[Aeq, beq] = build_endpoint_equalities(n_coef, T, p0, pf, v0, a0, j0, vf, af, jf, ...
    enforce_end_accel, enforce_end_jerk);
[Hc, fc] = add_soft_endpoint_derivative_cost(Hc, zeros(n_poly, 1), n_coef, T, ...
    af, jf, end_accel_weight, end_jerk_weight);
[G_soft, h_soft, G_hard, h_hard, ineq_meta] = build_ineq_constraints(n_coef, T, spec);

n_soft = size(G_soft, 1);
n_var = n_poly + n_soft;
H = blkdiag(Hc, 2 * spec.slack_weight * eye(n_soft));
f = [fc; zeros(n_soft, 1)];
A = [G_soft, -eye(n_soft); G_hard, zeros(size(G_hard, 1), n_soft)];
b = [h_soft; h_hard];
Aeq_full = [Aeq, zeros(size(Aeq, 1), n_soft)];
lb = [-inf(n_poly, 1); zeros(n_soft, 1)];
ub = [];
if ~isempty(x0) && numel(x0) ~= n_var
    x0 = [];
end
if isempty(x0) && strcmpi(qp_algorithm, 'active-set')
    x0 = zeros(n_var, 1);
end

qp_opts = optimoptions('quadprog', 'Display', 'off', ...
    'Algorithm', qp_algorithm, ...
    'MaxIterations', spec.max_iterations, ...
    'ConstraintTolerance', spec.constraint_tolerance, ...
    'OptimalityTolerance', spec.optimality_tolerance);

[sol, fval, exitflag, output] = quadprog(H, f, A, b, Aeq_full, beq, lb, ub, x0, qp_opts);
if isempty(sol)
    sol = nan(n_var, 1);
end

c = sol(1:n_poly);
slack = sol(n_poly + 1:end);
coeff = zeros(n_coef, dim, 1);
for d = 1:dim
    idx = dim_indices(d, n_coef);
    coeff(:, d, 1) = c(idx);
end

traj = struct();
traj.type = sprintf('minimum_snap_%dth_order_constrained', order);
traj.t_breaks = [0, T];
traj.durations = T;
traj.waypoints = [p0; pf];
traj.order = order;
traj.coeff = coeff;
traj.dim = dim;
traj.constraint_residual_inf = norm(Aeq * c - beq, inf);
traj.ineq_max_violation = max([G_soft * c - h_soft; G_hard * c - h_hard]);
traj.hard_ineq_max_violation = max(G_hard * c - h_hard);
traj.ineq_slack_l1 = sum(slack, 'omitnan');
traj.notes = 'Single-segment constrained minimum-snap polynomial. Coefficients use local segment time tau.';

info = struct();
info.fval = fval;
info.output = output;
info.slack = slack;
info.max_slack = max(slack, [], 'omitnan');
info.slack_l1 = sum(slack, 'omitnan');
info.snap_cost = c' * Q_snap * c;
info.ineq_meta = ineq_meta;
info.spec = spec;
info.solution = sol;
info.qp_algorithm = qp_algorithm;
end

function spec = default_ineq_spec(user, h_f_m)
spec = struct();
spec.n_check = get_scalar(user, 'n_check', 120);
spec.h_start_m = get_scalar(user, 'h_start_m', 0);
spec.h_f_m = get_scalar(user, 'h_f_m', h_f_m);
spec.h_shape_p = get_scalar(user, 'h_shape_p', 0.7);
spec.h_upper_margin = get_scalar(user, 'h_upper_margin', 0.25);
spec.h_upper_progress_enable = get_bool(user, 'h_upper_progress_enable', false);
spec.h_upper_progress_tau = get_scalar(user, 'h_upper_progress_tau', 0.8);
spec.h_upper_progress_ratio = get_scalar(user, 'h_upper_progress_ratio', 0.9);
spec.h_upper_progress_power = get_scalar(user, 'h_upper_progress_power', 0.7);
spec.gamma_max_deg = get_scalar(user, 'gamma_max_deg', 65);
spec.x_monotonic = get_bool(user, 'x_monotonic', true);
spec.h_monotonic = get_bool(user, 'h_monotonic', true);
spec.monotonic_velocity = get_bool(user, 'monotonic_velocity', true);
spec.ax_min_initial = get_scalar(user, 'ax_min_initial', 0);
spec.ax_min_frac = get_scalar(user, 'ax_min_frac', 0.3);
spec.vx_min_late = get_scalar(user, 'vx_min_late', 0);
spec.vx_min_late_start_frac = get_scalar(user, 'vx_min_late_start_frac', 0.6);
spec.slack_weight = get_scalar(user, 'slack_weight', 1e5);
spec.constraint_tolerance = get_scalar(user, 'constraint_tolerance', 1e-8);
spec.optimality_tolerance = get_scalar(user, 'optimality_tolerance', 1e-8);
spec.max_iterations = get_scalar(user, 'max_iterations', 1000);
spec.slack_pass_tol = get_scalar(user, 'slack_pass_tol', 1e-4);
spec.n_check = max(3, round(spec.n_check));
end

function [Aeq, beq] = build_endpoint_equalities(n_coef, T, p0, pf, v0, a0, j0, vf, af, jf, ...
    enforce_end_accel, enforce_end_jerk)
rows = {};
rhs = [];
for d = 1:3
    rows{end+1} = full_row(d, n_coef, 0, 0); %#ok<AGROW>
    rhs(end+1, 1) = p0(d); %#ok<AGROW>
    rows{end+1} = full_row(d, n_coef, T, 0); %#ok<AGROW>
    rhs(end+1, 1) = pf(d); %#ok<AGROW>
    rows{end+1} = full_row(d, n_coef, 0, 1); %#ok<AGROW>
    rhs(end+1, 1) = v0(d); %#ok<AGROW>
    rows{end+1} = full_row(d, n_coef, 0, 2); %#ok<AGROW>
    rhs(end+1, 1) = a0(d); %#ok<AGROW>
    rows{end+1} = full_row(d, n_coef, 0, 3); %#ok<AGROW>
    rhs(end+1, 1) = j0(d); %#ok<AGROW>
    rows{end+1} = full_row(d, n_coef, T, 1); %#ok<AGROW>
    rhs(end+1, 1) = vf(d); %#ok<AGROW>
    if enforce_end_accel
        rows{end+1} = full_row(d, n_coef, T, 2); %#ok<AGROW>
        rhs(end+1, 1) = af(d); %#ok<AGROW>
    end
    if enforce_end_jerk
        rows{end+1} = full_row(d, n_coef, T, 3); %#ok<AGROW>
        rhs(end+1, 1) = jf(d); %#ok<AGROW>
    end
end
Aeq = vertcat(rows{:});
beq = rhs;
end

function [H, f] = add_soft_endpoint_derivative_cost(H, f, n_coef, T, af, jf, accel_weight, jerk_weight)
for d = 1:3
    if accel_weight > 0
        r = full_row(d, n_coef, T, 2);
        H = H + 2 * accel_weight * (r' * r);
        f = f - 2 * accel_weight * af(d) * r';
    end
    if jerk_weight > 0
        r = full_row(d, n_coef, T, 3);
        H = H + 2 * jerk_weight * (r' * r);
        f = f - 2 * jerk_weight * jf(d) * r';
    end
end
end

function [G_soft, h_soft, G_hard, h_hard, meta] = build_ineq_constraints(n_coef, T, spec)
soft_rows = {};
h_soft = [];
hard_rows = {};
h_hard = [];
soft_meta = {};
hard_meta = {};
N = spec.n_check;
gamma_tan = tand(spec.gamma_max_deg);

for k = 1:(N - 1)
    t = (k / N) * T;
    tau = t / T;
    h_lower = height_lower_bound(spec, tau);
    h_upper = height_upper_bound(spec, tau);

    soft_rows{end+1} = full_row(3, n_coef, t, 0); %#ok<AGROW>
    h_soft(end+1, 1) = -h_lower; %#ok<AGROW>
    soft_meta{end+1} = struct('kind', 'height_lower', 't', t); %#ok<AGROW>

    soft_rows{end+1} = -full_row(3, n_coef, t, 0); %#ok<AGROW>
    h_soft(end+1, 1) = h_upper; %#ok<AGROW>
    soft_meta{end+1} = struct('kind', 'height_upper', 't', t); %#ok<AGROW>

    hard_rows{end+1} = -full_row(3, n_coef, t, 1) - gamma_tan * full_row(1, n_coef, t, 1); %#ok<AGROW>
    h_hard(end+1, 1) = 0; %#ok<AGROW>
    hard_meta{end+1} = struct('kind', 'gamma_limit', 't', t); %#ok<AGROW>

    if t <= spec.ax_min_frac * T + eps
        soft_rows{end+1} = -full_row(1, n_coef, t, 2); %#ok<AGROW>
        h_soft(end+1, 1) = -spec.ax_min_initial; %#ok<AGROW>
        soft_meta{end+1} = struct('kind', 'initial_ax_min', 't', t); %#ok<AGROW>
    end
end

for k = 0:(N - 1)
    t0 = (k / N) * T;
    t1 = ((k + 1) / N) * T;
    if spec.x_monotonic
        hard_rows{end+1} = full_row(1, n_coef, t0, 0) - full_row(1, n_coef, t1, 0); %#ok<AGROW>
        h_hard(end+1, 1) = 0; %#ok<AGROW>
        hard_meta{end+1} = struct('kind', 'x_monotonic', 't', t0); %#ok<AGROW>
    end
    if spec.h_monotonic
        hard_rows{end+1} = full_row(3, n_coef, t1, 0) - full_row(3, n_coef, t0, 0); %#ok<AGROW>
        h_hard(end+1, 1) = 0; %#ok<AGROW>
        hard_meta{end+1} = struct('kind', 'height_monotonic', 't', t0); %#ok<AGROW>
    end
end

if spec.monotonic_velocity
    for k = 0:N
        t = (k / N) * T;
        if spec.x_monotonic
            hard_rows{end+1} = -full_row(1, n_coef, t, 1); %#ok<AGROW>
            h_hard(end+1, 1) = 0; %#ok<AGROW>
            hard_meta{end+1} = struct('kind', 'vx_nonnegative', 't', t); %#ok<AGROW>
        end
        if spec.h_monotonic
            hard_rows{end+1} = full_row(3, n_coef, t, 1); %#ok<AGROW>
            h_hard(end+1, 1) = 0; %#ok<AGROW>
            hard_meta{end+1} = struct('kind', 'height_rate_nonnegative', 't', t); %#ok<AGROW>
        end
    end
end

if spec.vx_min_late > 0
    late_start_t = spec.vx_min_late_start_frac * T;
    for k = 0:N
        t = (k / N) * T;
        if t >= late_start_t - eps
            hard_rows{end+1} = -full_row(1, n_coef, t, 1); %#ok<AGROW>
            h_hard(end+1, 1) = -spec.vx_min_late; %#ok<AGROW>
            hard_meta{end+1} = struct('kind', 'late_vx_min', 't', t); %#ok<AGROW>
        end
    end
end

G_soft = vertcat(soft_rows{:});
if isempty(hard_rows)
    G_hard = zeros(0, 3 * n_coef);
else
    G_hard = vertcat(hard_rows{:});
end
meta = struct('soft', {soft_meta}, 'hard', {hard_meta});
end

function h_lower = height_lower_bound(spec, tau)
h_delta = max(spec.h_f_m - spec.h_start_m, 0);
h_lower = spec.h_start_m + h_delta * tau^spec.h_shape_p;
end

function h_upper = height_upper_bound(spec, tau)
terminal_upper = spec.h_f_m + spec.h_upper_margin;
if ~spec.h_upper_progress_enable
    h_upper = terminal_upper;
    return;
end

tau_gate = min(max(spec.h_upper_progress_tau, 1e-3), 0.999);
ratio_gate = min(max(spec.h_upper_progress_ratio, 0.05), 1.0);
power = max(spec.h_upper_progress_power, 0.05);
if tau <= tau_gate
    ratio = ratio_gate * (tau / tau_gate)^power;
else
    s = (tau - tau_gate) / (1 - tau_gate);
    ratio = ratio_gate + (1 - ratio_gate) * s;
end
h_delta = max(spec.h_f_m - spec.h_start_m, 0);
h_upper = min(terminal_upper, spec.h_start_m + h_delta * ratio);
end

function Q = build_snap_cost(n_coef, T)
Q = zeros(n_coef, n_coef);
for i = 4:n_coef-1
    ci = deriv_coeff(i, 4);
    for j = 4:n_coef-1
        cj = deriv_coeff(j, 4);
        power = i + j - 7;
        Q(i + 1, j + 1) = ci * cj * T^power / power;
    end
end
end

function row = full_row(dim_idx, n_coef, tau, deriv)
row = zeros(1, 3 * n_coef);
idx = dim_indices(dim_idx, n_coef);
for k = deriv:n_coef-1
    row(idx(k + 1)) = deriv_coeff(k, deriv) * tau^(k - deriv);
end
end

function idx = dim_indices(dim_idx, n_coef)
first = (dim_idx - 1) * n_coef + 1;
idx = first:(first + n_coef - 1);
end

function v = get_vec_alias(opts, names, dim)
v = [];
for i = 1:numel(names)
    if isfield(opts, names{i})
        v = double(opts.(names{i}));
        break;
    end
end
if isempty(v)
    v = zeros(1, dim);
end
v = v(:)';
if numel(v) ~= dim
    error('gj_solve_min_snap_constrained:BadOption', 'Vector option must have %d entries.', dim);
end
end

function value = get_scalar(opts, name, default_value)
if isfield(opts, name)
    value = double(opts.(name));
else
    value = default_value;
end
end

function value = get_scalar_alias(opts, names, default_value)
value = default_value;
for i = 1:numel(names)
    if isfield(opts, names{i})
        value = double(opts.(names{i}));
        return;
    end
end
end

function value = get_bool(opts, name, default_value)
if isfield(opts, name)
    value = logical(opts.(name));
else
    value = default_value;
end
end

function value = get_char(opts, name, default_value)
if isfield(opts, name)
    value = char(opts.(name));
else
    value = default_value;
end
end

function c = deriv_coeff(power, deriv)
if deriv == 0
    c = 1;
else
    c = factorial(power) / factorial(power - deriv);
end
end
