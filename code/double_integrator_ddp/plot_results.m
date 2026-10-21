function figs = plot_results(sol, cfg)
%PLOT_RESULTS 双积分轨迹、速度、加速度及优化过程。
%   figs = plot_results(sol, cfg)
%   必需字段：sol.X (4×(N+1))、sol.U (2×N)。
%   可选字段：sol.history.J、sol.parts；缺少时对应面板显示说明。
%   兼容当前 solve_ddp 的输出，不依赖 sol.metrics 或 sol.J。
%   返回 trajectory、optimization、avoidance 三个窗口句柄。

    X = sol.X;
    U = sol.U;
    N = size(U, 2);
    assert(N >= 1 && isequal(size(X), [4, N + 1]) ...
        && size(U, 1) == 2, 'X 应为 4×(N+1)，U 应为 2×N，N >= 1。');
    assert(all(isfinite(X(:))) && all(isfinite(U(:))), ...
        '轨迹中含有 NaN 或 Inf，无法绘图。');
    dt = cfg.model.dt;
    assert(isscalar(dt) && isfinite(dt) && dt > 0, 'dt 必须为正数。');

    % 状态有 N+1 个采样点；控制有 N 个区间，最后一段保持到 T。
    tx = (0:N) * dt;
    U_hold = [U, U(:, end)];
    speed = sqrt(sum(X(3:4, :).^2, 1));
    accel = sqrt(sum(U.^2, 1));
    goal = cfg.task.x_goal(:);
    % 同时校验障碍物配置，并为距离图计算连续区间的最小净距离。
    avoidance = obstacle_metrics(X, U, cfg);
    obs = cfg.task.obs;
    M = size(obs, 1);

    figs.trajectory = figure('Name', '双积分：轨迹与控制', 'Color', 'w', ...
        'Position', [50,50,1200,800]);
    subplot(2, 2, 1);
    hold on;
    theta = linspace(0, 2*pi, 200);
    circle = [cos(theta); sin(theta)];
    hObs = gobjects(M, 1);
    hSafe = gobjects(M, 1);
    for i = 1:M
        center = obs(i, 1:2).';
        boundary = center + obs(i, 3)*circle;
        safe = center + (obs(i, 3)+cfg.task.obs_safe)*circle;
        hObs(i) = fill(boundary(1,:), boundary(2,:), [0.35 0.35 0.35], ...
            'EdgeColor', 'k');
        hSafe(i) = plot(safe(1,:), safe(2,:), '--', ...
            'Color', [0.9 0.45 0], 'LineWidth', 1.2);
        text(center(1), center(2), sprintf('%d', i), ...
            'Color', 'w', 'FontWeight', 'bold', ...
            'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');
    end
    hPath = plot(X(1, :), X(2, :), 'b-', 'LineWidth', 1.5);
    hStart = plot(X(1, 1), X(2, 1), 'go', 'MarkerFaceColor', 'g');
    hEnd = plot(X(1, end), X(2, end), 'bs', 'MarkerFaceColor', 'b');
    hGoal = plot(goal(1), goal(2), 'rx', 'MarkerSize', 10, 'LineWidth', 2);
    axis equal; grid on;
    xlabel('p_x (m)'); ylabel('p_y (m)');
    title(sprintf('平面轨迹：终点位置误差 %.3g m', ...
        norm(X(1:2, end) - goal(1:2))));
    legend([hPath,hStart,hEnd,hGoal,hObs(1),hSafe(1)], ...
        {'优化轨迹', '起点', '实际终点', '目标位置', '原始障碍物', '安全边界'}, ...
        'Location', 'best');

    subplot(2, 2, 2);
    plot(tx, X(3, :), tx, X(4, :), 'LineWidth', 1.2);
    hold on;
    plot(tx, speed, 'k-', 'LineWidth', 1.5);
    plot([0, tx(end)], cfg.limit.vmax * [1, 1], 'r--');
    grid on; xlim([0, tx(end)]);
    xlabel('时间 (s)'); ylabel('速度 (m/s)');
    title(sprintf('速度：最大模长 %.3g m/s', max(speed)));
    legend('v_x', 'v_y', '||v||_2', '速度模长上限', 'Location', 'best');

    subplot(2, 2, 3);
    stairs(tx, U_hold(1, :), 'LineWidth', 1.2);
    hold on;
    stairs(tx, U_hold(2, :), 'LineWidth', 1.2);
    stairs(tx, [accel, accel(end)], 'k-', 'LineWidth', 1.5);
    plot([0, tx(end)], cfg.limit.amax * [1, 1], 'r--');
    grid on; xlim([0, tx(end)]);
    xlabel('时间 (s)'); ylabel('加速度 (m/s^2)');
    title(sprintf('分段常值控制：最大模长 %.3g m/s^2', max(accel)));
    legend('a_x', 'a_y', '||a||_2', '加速度模长上限', 'Location', 'best');

    subplot(2, 2, 4);
    plot(tx, max(0, speed - cfg.limit.vmax), 'LineWidth', 1.2);
    hold on;
    stairs(tx, [max(0, accel - cfg.limit.amax), ...
        max(0, accel(end) - cfg.limit.amax)], 'LineWidth', 1.2);
    grid on; xlim([0, tx(end)]);
    xlabel('时间 (s)'); ylabel('超限量（各自单位）');
    title('软约束超限：罚函数不保证严格满足上限');
    legend('速度超限 (m/s)', '加速度超限 (m/s^2)', 'Location', 'best');

    figs.optimization = figure('Name', '双积分：优化与代价', 'Color', 'w', ...
        'Position', [100,100,1200,520]);
    subplot(1, 2, 1);
    % 约定 J(1) 是初始代价，之后每项是一次接受更新后的代价。
    if isfield(sol, 'history') && isfield(sol.history, 'J') ...
            && ~isempty(sol.history.J)
        J = sol.history.J(:);
        plot(0:numel(J)-1, J, '-o', 'LineWidth', 1.2);
        grid on;
        xlabel('接受更新次数（0 为初始轨迹）'); ylabel('总代价 J');
        title('代价收敛过程');
    else
        axis off;
        text(0.5, 0.5, {'尚未记录 sol.history.J', ...
            '在求解器中保存初始及每次接受更新后的代价'}, ...
            'Units', 'normalized', 'HorizontalAlignment', 'center');
    end

    subplot(1, 2, 2);
    names = {'control', 'speed_penalty', 'accel_penalty', ...
        'terminal_position', 'terminal_velocity', 'obstacle_penalty', 'terminal_obstacle'};
    if isfield(sol, 'parts') && isstruct(sol.parts) ...
            && all(isfield(sol.parts, names))
        values = zeros(1, numel(names));
        for i = 1:numel(names)
            component = sol.parts.(names{i});
            % rollout 的过程代价已包含 dt，这里只求和，不再乘 dt。
            values(i) = sum(component(:));
        end
        bar(values);
        set(gca, 'XTick', 1:numel(names), 'XTickLabel', ...
            {'控制能量', '速度罚项', '加速度罚项', '终端位置', '终端速度', '过程避障', '终端避障'});
        xtickangle(30);
        grid on; ylabel('代价');
        title(sprintf('最终代价组成：总和 %.6g', sum(values)));
    else
        axis off;
        text(0.5, 0.5, '尚未提供完整的 sol.parts 代价分项', ...
            'Units', 'normalized', 'HorizontalAlignment', 'center');
    end

    figs.avoidance = figure('Name', '双积分：障碍距离', 'Color', 'w');
    hold on;
    colors = lines(M);
    hIndividual = gobjects(M, 1);
    labels = cell(1, M);
    for i = 1:M
        hIndividual(i) = plot(tx, avoidance.node_clearance_by_obstacle(i,:), ...
            '-', 'Color', colors(i,:), 'LineWidth', 0.8);
        labels{i} = sprintf('障碍物%d：节点净距离', i);
    end
    hNearest = plot(tx, avoidance.node_clearance, 'k-', 'LineWidth', 1.8);
    hInterval = plot(tx(1:end-1)+dt/2, avoidance.interval_clearance, ...
        'm.', 'MarkerSize', 8);
    hRequired = yline(cfg.task.obs_safe, '--', ...
        'Color', [0.9 0.45 0], 'LineWidth', 1.2);
    hCollision = yline(0, 'k--');
    hMinimum = plot(avoidance.min_clearance_time, ...
        avoidance.min_obstacle_clearance, 'ro', 'MarkerFaceColor', 'r');
    grid on; xlim([0,tx(end)]);
    xlabel('时间 (s)'); ylabel('轨迹位置到原始障碍物边界的净距离 (m)');
    title({sprintf('最小净距离 %.4f m；最小安全余量 %.4f m', ...
        avoidance.min_obstacle_clearance, avoidance.min_safety_margin), ...
        sprintf('对应障碍物%d，发生时刻 %.3f s', ...
        avoidance.min_clearance_obstacle, avoidance.min_clearance_time)});
    legend([hIndividual; hNearest; hInterval; hRequired; hCollision; hMinimum], ...
        [labels, {'最近障碍物：节点净距离', '各区间最小值（标于区间中点）', ...
        '要求的安全距离', '碰撞边界', '全局最小净距离（实际时刻）'}], ...
        'Location', 'best');
end
