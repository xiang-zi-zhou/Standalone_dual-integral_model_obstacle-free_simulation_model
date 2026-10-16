function figs = plot_results(sol, cfg)
%PLOT_RESULTS 双积分轨迹、速度、加速度及优化过程。
%   figs = plot_results(sol, cfg)
%   必需字段：sol.X (4×(N+1))、sol.U (2×N)。
%   可选字段：sol.history.J、sol.parts；缺少时对应面板显示说明。
%   兼容当前 solve_ddp 的输出，不依赖 sol.metrics 或 sol.J。
%   figs.trajectory 和 figs.optimization 为两个窗口的句柄。

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

    figs.trajectory = figure('Name', '双积分：轨迹与控制', 'Color', 'w');
    subplot(2, 2, 1);
    plot(X(1, :), X(2, :), 'b-', 'LineWidth', 1.5);
    hold on;
    plot(X(1, 1), X(2, 1), 'go', 'MarkerFaceColor', 'g');
    plot(X(1, end), X(2, end), 'bs', 'MarkerFaceColor', 'b');
    plot(goal(1), goal(2), 'rx', 'MarkerSize', 10, 'LineWidth', 2);
    axis equal; grid on;
    xlabel('p_x (m)'); ylabel('p_y (m)');
    title(sprintf('平面轨迹：终点位置误差 %.3g m', ...
        norm(X(1:2, end) - goal(1:2))));
    legend('优化轨迹', '起点', '实际终点', '目标位置', 'Location', 'best');

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

    figs.optimization = figure('Name', '双积分：优化与代价', 'Color', 'w');
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
        'terminal_position', 'terminal_velocity'};
    if isfield(sol, 'parts') && isstruct(sol.parts) ...
            && all(isfield(sol.parts, names))
        values = zeros(1, numel(names));
        for i = 1:numel(names)
            component = sol.parts.(names{i});
            % rollout 的过程代价已包含 dt，这里只求和，不再乘 dt。
            values(i) = sum(component(:));
        end
        bar(values);
        set(gca, 'XTick', 1:5, 'XTickLabel', ...
            {'控制能量', '速度罚项', '加速度罚项', '终端位置', '终端速度'});
        grid on; ylabel('代价');
        title(sprintf('最终代价组成：总和 %.6g', sum(values)));
    else
        axis off;
        text(0.5, 0.5, '尚未提供完整的 sol.parts 代价分项', ...
            'Units', 'normalized', 'HorizontalAlignment', 'center');
    end
end
