function traj = gj_solve_min_snap_endpoint_fast_shaped(p0, pf, T, opts)
%GJ_SOLVE_MIN_SNAP_ENDPOINT_FAST_SHAPED Fast 9th-order endpoint solver with soft shape cost.
%
% Hard constraints stay identical to the endpoint-fast kernel:
% p0/v0/a0/j0 -> pf/vf/af/jf.
%
% The extra shaping terms are soft quadratic costs in normalized time:
% h(0.75T) ~= 0.80*h_f, h(0.80T) ~= 0.85*h_f, and optionally
% vx(0.70T) ~= 8.5 m/s. The cached KKT map keeps runtime work small.

if nargin < 4
    opts = struct();
end

p0 = double(p0(:)');
pf = double(pf(:)');
T = double(T);
dim = 3;
order = round(get_scalar(opts, 'poly_order', 9));
if order ~= 9
    error('gj_solve_min_snap_endpoint_fast_shaped:BadOrder', ...
        'Shaped fast solver currently expects poly_order = 9.');
end
if numel(p0) ~= dim || numel(pf) ~= dim
    error('gj_solve_min_snap_endpoint_fast_shaped:BadPosition', 'p0 and pf must be 1x3 vectors.');
end
if ~(isfinite(T) && T > 0)
    error('gj_solve_min_snap_endpoint_fast_shaped:BadTime', 'T must be positive.');
end

n_coef = order + 1;
v0 = get_vec_alias(opts, {'v0', 'v_start'}, dim);
a0 = get_vec_alias(opts, {'a0', 'a_start'}, dim);
j0 = get_vec_alias(opts, {'j0', 'j_start'}, dim);
vf = get_vec_alias(opts, {'vf', 'v_end'}, dim);
af = get_vec_alias(opts, {'af', 'a_end'}, dim);
jf = get_vec_alias(opts, {'jf', 'j_end'}, dim);
coeff_regularization = get_scalar(opts, 'coeff_regularization', 1e-9);

shape = shape_spec(opts);
A_tau = endpoint_constraint_matrix(n_coef, 1);
time_scale = T .^ (0:order)';

coeff = zeros(n_coef, dim, 1);
coeff_tau = zeros(n_coef, dim);
residual = zeros(1, dim);
for d = 1:dim
    b_tau = [p0(d); pf(d); ...
        T * v0(d); T^2 * a0(d); T^3 * j0(d); ...
        T * vf(d); T^2 * af(d); T^3 * jf(d)];
    map = normalized_shaped_map(n_coef, coeff_regularization, shape, d);
    y_shape = shape_targets_for_dim(d, p0, pf, T, shape);
    d_tau = map * [b_tau; y_shape];
    coeff_tau(:, d) = d_tau;
    coeff(:, d, 1) = d_tau ./ time_scale;
    residual(d) = norm(A_tau * d_tau - b_tau, inf);
end

traj = struct();
traj.type = 'minimum_snap_9th_order_fast_endpoint_shaped';
traj.t_breaks = [0, T];
traj.durations = T;
traj.waypoints = [p0; pf];
traj.order = order;
traj.coeff = coeff;
traj.coeff_tau = coeff_tau;
traj.dim = dim;
traj.constraint_residual_inf = residual;
traj.ineq_max_violation = NaN;
traj.hard_ineq_max_violation = NaN;
traj.ineq_slack_l1 = NaN;
traj.shape = shape;
traj.notes = 'Fast 9th-order normalized endpoint minimum-snap KKT map with soft shape costs.';
end

function shape = shape_spec(opts)
shape = struct();
shape.h_tau = get_vec(opts, 'shape_h_tau', [0.75, 0.80]);
shape.h_ratio = get_vec(opts, 'shape_h_ratio', [0.80, 0.85]);
shape.h_weight = get_vec(opts, 'shape_h_weight', [8.0, 10.0]);
shape.vx_enable = get_bool(opts, 'shape_vx_enable', true);
shape.vx_tau = get_scalar(opts, 'shape_vx_tau', 0.70);
shape.vx_target_mps = get_scalar(opts, 'shape_vx_target_mps', 8.5);
shape.vx_weight = get_scalar(opts, 'shape_vx_weight', 2.0);

shape.h_tau = shape.h_tau(:)';
shape.h_ratio = shape.h_ratio(:)';
shape.h_weight = shape.h_weight(:)';
if numel(shape.h_tau) ~= numel(shape.h_ratio) || numel(shape.h_tau) ~= numel(shape.h_weight)
    error('gj_solve_min_snap_endpoint_fast_shaped:BadShape', ...
        'shape_h_tau, shape_h_ratio, and shape_h_weight must have matching lengths.');
end
shape.n_h = numel(shape.h_tau);
end

function y = shape_targets_for_dim(dim_idx, p0, pf, T, shape)
h0 = -p0(3);
hf = -pf(3);
h_delta = hf - h0;
y = zeros(shape_target_count(dim_idx, shape), 1);
idx = 0;
if dim_idx == 3
    for i = 1:shape.n_h
        idx = idx + 1;
        h_target = h0 + shape.h_ratio(i) * h_delta;
        y(idx) = -h_target;
    end
end
if shape.vx_enable && dim_idx == 1
    idx = idx + 1;
    y(end) = T * shape.vx_target_mps;
end
end

function n = shape_target_count(dim_idx, shape)
n = 0;
if dim_idx == 3
    n = n + shape.n_h;
end
if shape.vx_enable && dim_idx == 1
    n = n + 1;
end
end

function map = normalized_shaped_map(n_coef, coeff_regularization, shape, dim_idx)
persistent cache_keys cache_maps
key = [n_coef, coeff_regularization, shape.h_tau, shape.h_ratio, ...
    shape.h_weight, double(shape.vx_enable), shape.vx_tau, ...
    shape.vx_target_mps, shape.vx_weight, dim_idx];
if ~isempty(cache_keys)
    for i = 1:numel(cache_keys)
        if isequal(cache_keys{i}, key)
            map = cache_maps{i};
            return;
        end
    end
end

Q = build_snap_cost(n_coef, 1) + coeff_regularization * eye(n_coef);
A = endpoint_constraint_matrix(n_coef, 1);
[R, weights] = shape_matrix(n_coef, shape, dim_idx);
if isempty(R)
    F = zeros(n_coef, 0);
else
    W = diag(weights);
    Q = Q + R' * W * R;
    F = R' * W;
end

n_con = size(A, 1);
KKT = [Q, A'; A, zeros(n_con)];
n_y = size(R, 1);
rhs_map = [zeros(n_coef, n_con), F; eye(n_con), zeros(n_con, n_y)];
warn_state = warning('off', 'MATLAB:nearlySingularMatrix');
cleanup = onCleanup(@() warning(warn_state));
sol_map = KKT \ rhs_map;
map = sol_map(1:n_coef, :);

cache_keys{end + 1} = key;
cache_maps{end + 1} = map;
end

function [R, weights] = shape_matrix(n_coef, shape, dim_idx)
rows = {};
weights = [];
if dim_idx == 3
    for i = 1:shape.n_h
        rows{end + 1, 1} = basis_row(n_coef, shape.h_tau(i), 0); %#ok<AGROW>
        weights(end + 1, 1) = shape.h_weight(i); %#ok<AGROW>
    end
end
if shape.vx_enable && dim_idx == 1
    rows{end + 1, 1} = basis_row(n_coef, shape.vx_tau, 1);
    weights(end + 1, 1) = shape.vx_weight;
end
if isempty(rows)
    R = zeros(0, n_coef);
else
    R = vertcat(rows{:});
end
end

function A = endpoint_constraint_matrix(n_coef, T)
rows = {
    basis_row(n_coef, 0, 0)
    basis_row(n_coef, T, 0)
    basis_row(n_coef, 0, 1)
    basis_row(n_coef, 0, 2)
    basis_row(n_coef, 0, 3)
    basis_row(n_coef, T, 1)
    basis_row(n_coef, T, 2)
    basis_row(n_coef, T, 3)
    };
A = vertcat(rows{:});
end

function row = basis_row(n_coef, tau, deriv)
row = zeros(1, n_coef);
for k = deriv:n_coef-1
    row(k + 1) = deriv_coeff(k, deriv) * tau^(k - deriv);
end
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
    error('gj_solve_min_snap_endpoint_fast_shaped:BadOption', 'Vector option must have %d entries.', dim);
end
end

function value = get_vec(opts, name, default_value)
if isfield(opts, name)
    value = double(opts.(name));
else
    value = default_value;
end
end

function value = get_scalar(opts, name, default_value)
if isfield(opts, name)
    value = double(opts.(name));
else
    value = default_value;
end
end

function value = get_bool(opts, name, default_value)
if isfield(opts, name)
    value = logical(opts.(name));
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
