function m = obstacle_metrics(X, U, cfg)
% 多圆障碍物避障统计。
%
% 输入：
%   X   : 4×(N+1)，每列为 [px; py; vx; vy]
%   U   : 2×N，每列为一个控制区间内的恒定加速度
%   obs : M×3，每行为 [cx, cy, radius]
%
% 净距离 = 轨迹位置到圆心的距离 - 原始障碍物半径
%   净距离 > 0：在原始障碍物外
%   净距离 = 0：接触原始障碍物边界
%   净距离 < 0：进入原始障碍物内部
%
% 对每个控制区间，检查连续二次运动轨迹的最小距离，
% 而不只是检查离散节点。
    obs = cfg.task.obs;
    safe = cfg.task.obs_safe;
    dt = cfg.model.dt;

    M = size(obs,1);
    N = size(U,2);
    % ---------- 输入检查 ----------
    assert(N >= 1 && isequal(size(U), [2,N]) && ...
        isequal(size(X), [4,N+1]) && all(isfinite(X(:))) && ...
        all(isfinite(U(:))), '避障统计需要有限且尺寸一致的轨迹。');
    assert(isscalar(dt) && isfinite(dt) && dt > 0, 'dt必须是有限正数。');
    assert(ismatrix(obs) && size(obs,2) == 3 && M >= 1 && ...
        all(isfinite(obs(:))) && all(obs(:,3) > 0), ...
        '障碍物应为M×3矩阵，每行[cx, cy, radius]，半径必须为正。');
    assert(isscalar(safe) && isfinite(safe) && safe >= 0, ...
        'obs_safe必须是有限非负标量。');
    % ---------- 保存每个障碍物的独立结果 ----------
    node_all = zeros(M, N+1);
    segment_all = zeros(M, N);
    times_all = zeros(M, N);
    for i = 1:M
        center = obs(i, 1:2).';
        radius = obs(i, 3);

        % 所有离散节点相对于第i个障碍物的净距离
        node_all(i, :) = vecnorm(X(1:2, :) - center, 2, 1) - radius;
        % "2"表示求范数的方式；“1”表示沿其运算的维度
        for k = 1:N
            % z = tau/dt in [0,1], 相对圆心的位置为q(z) = d + b*z + a*z^2【匀加速公式】
            d = X(1:2,k) - center;
            b = dt * X(3:4,k);
            a = 0.5*dt^2*U(:,k);

            % ||q(z)||^2的导数除以2后的多项式系数
            coeff = [
                2*(a.'*a), ...
                3*(a.'*b), ...
                b.'*b + 2*(a.'*d), ...
                b.'*d
            ];

            % 区间端点始终是最小值候选点
            z = [0;1];
            % 去掉最高次项前面的零系数
            first = find(coeff ~= 0, 1);

            if ~isempty(first)
                candidates = roots(coeff(first:end));
                % 保留近似为实数的根
                candidates = real(candidates(abs(imag(candidates)) < 1e-9));
                z = [z; candidates(candidates >= 0 & candidates <= 1)];% 只保留当前控制区间内的根
            end

            % 计算所有候选时刻的相对位置
            q = d + b*z.' + a*(z.'.^2);
            % 找到该区间内到当前圆心的最小距离
            [distance, j] = min(vecnorm(q,2,1));
            segment_all(i, k) = distance - radius;
            % 将归一化时间z换算成整条轨迹的绝对时间
            times_all(i, k) = (k-1+z(j))*dt;
        end
    end
    % ---------- 对全部障碍物汇总 ----------
    % 每个节点到所有障碍物的最小净距离
    [node, node_obs] = min(node_all, [], 1);

    % 每个区间对所有障碍物的最小净距离
    [segment, segment_obs] = min(segment_all, [], 1);

    % 提取每个区间最小净距离对应的实际时刻
    idx = sub2ind([M, N], segment_obs, 1:N);
    times = times_all(idx);

    % 整条连续轨迹的最小净距离
    [clearance, k] = min(segment);
    minTime = times(k);
    minObs = segment_obs(k);

    % 显式包括存储的全部节点，含终端节点
    [nodeMin, j] = min(node);

    if nodeMin < clearance
        clearance = nodeMin;
        minTime = (j - 1) * dt;
        minObs = node_obs(j);
    end

    % ---------- 保留原有输出字段 ----------
    m.node_clearance = node;
    m.interval_clearance = segment;
    m.min_obstacle_clearance = clearance;
    m.min_safety_margin = clearance - safe;
    m.min_clearance_time = minTime;
    m.terminal_clearance = node(end);

    % 接触原始障碍物边界也算碰撞
    m.obstacle_collision = clearance <= 0;

    % 安全距离不足的最大程度
    m.safety_violation = max(0, -m.min_safety_margin);

    % 对区间计数：同一区间碰到多个障碍物也只计一次
    m.collision_intervals = nnz(segment <= 0);
    m.unsafe_intervals = nnz(segment < safe);

    % ---------- 新增输出，方便排查 ----------
    m.min_clearance_obstacle = minObs;

    % 每个障碍物的独立距离数据
    m.node_clearance_by_obstacle = node_all;
    m.interval_clearance_by_obstacle = segment_all;

    % 每个节点、区间的最小净距离对应哪个障碍物
    m.node_nearest_obstacle = node_obs;
    m.interval_nearest_obstacle = segment_obs;
end
