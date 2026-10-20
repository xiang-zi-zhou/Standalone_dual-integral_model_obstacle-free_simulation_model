function m = obstacle_metrics(X, U, cfg)
% 单圆障碍：中心到原始边界的净距离；逐段求二次运动的距离极值。
% 假定 U 在每个 dt 内恒定，与当前双积分动力学一致。
obs = cfg.task.obs(:);
dt = cfg.model.dt;
N = size(U,2);
assert(isequal(size(X), [4,N+1]) && all(isfinite(X(:))) && ...
    all(isfinite(U(:))) && dt > 0, '避障统计需要有限且尺寸一致的轨迹。');
node = vecnorm(X(1:2,:) - obs(1:2),2,1) - obs(3);
segment = zeros(1,N);
times = zeros(1,N);
for k = 1:N
    % z = tau/dt in [0,1], q(z) = d + b*z + a*z^2
    d = X(1:2,k) - obs(1:2);
    b = dt*X(3:4,k);
    a = 0.5*dt^2*U(:,k);
    coeff = [2*(a.'*a), 3*(a.'*b), b.'*b+2*(a.'*d), b.'*d];
    first = find(coeff ~= 0,1);
    z = [0;1];
    if ~isempty(first)
        candidates = roots(coeff(first:end));
        candidates = real(candidates(abs(imag(candidates)) < 1e-9));
        z = [z; candidates(candidates >= 0 & candidates <= 1)];
    end
    q = d + b*z.' + a*(z.'.^2);
    [distance, j] = min(vecnorm(q,2,1));
    segment(k) = distance - obs(3);
    times(k) = (k-1+z(j))*dt;
end
[clearance, k] = min(segment);
% 显式包括存储的全部节点（含终端），避免遗漏。
[nodeMin, j] = min(node);
minTime = times(k);
if nodeMin < clearance
    clearance = nodeMin;
    minTime = (j-1)*dt;
end
m.node_clearance = node;
m.interval_clearance = segment;
m.min_obstacle_clearance = clearance;
m.min_safety_margin = clearance - cfg.task.obs_safe;
m.min_clearance_time = minTime;
m.terminal_clearance = node(end);
m.obstacle_collision = clearance <= 0; % 接触原始边界也计入
m.safety_violation = max(0, -m.min_safety_margin);
m.collision_intervals = nnz(segment <= 0);
m.unsafe_intervals = nnz(segment < cfg.task.obs_safe);
end
