function sample = gj_eval_traj(traj, t_query)
%GJ_EVAL_TRAJ Evaluate a GJ polynomial trajectory.
%
% sample fields are dim x n_t:
%   p, v, a, j, s

t_query = double(t_query(:)');
n_t = numel(t_query);
dim = traj.dim;

p = zeros(dim, n_t);
v = zeros(dim, n_t);
a = zeros(dim, n_t);
j = zeros(dim, n_t);
snp = zeros(dim, n_t);
seg_idx = zeros(1, n_t);

for it = 1:n_t
    [seg, tau] = locate_segment(traj, t_query(it));
    seg_idx(it) = seg;
    for d = 1:dim
        c = traj.coeff(:, d, seg);
        p(d, it) = eval_poly_deriv(c, tau, 0);
        v(d, it) = eval_poly_deriv(c, tau, 1);
        a(d, it) = eval_poly_deriv(c, tau, 2);
        j(d, it) = eval_poly_deriv(c, tau, 3);
        snp(d, it) = eval_poly_deriv(c, tau, 4);
    end
end

sample = struct();
sample.t = t_query;
sample.p = p;
sample.v = v;
sample.a = a;
sample.j = j;
sample.snap = snp;
sample.segment = seg_idx;
end

function [seg, tau] = locate_segment(traj, t)
if t <= traj.t_breaks(1)
    seg = 1;
    tau = 0;
    return;
end
if t >= traj.t_breaks(end)
    seg = numel(traj.durations);
    tau = traj.durations(end);
    return;
end
seg = find(traj.t_breaks <= t, 1, 'last');
if seg >= numel(traj.t_breaks)
    seg = numel(traj.durations);
end
tau = t - traj.t_breaks(seg);
end

function y = eval_poly_deriv(c, tau, deriv)
y = 0;
order = numel(c) - 1;
for k = deriv:order
    y = y + c(k + 1) * deriv_coeff(k, deriv) * tau^(k - deriv);
end
end

function c = deriv_coeff(power, deriv)
if deriv == 0
    c = 1;
else
    c = factorial(power) / factorial(power - deriv);
end
end
