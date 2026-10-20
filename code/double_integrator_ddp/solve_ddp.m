function sol = solve_ddp(x0, U_init, dynamics, stageCost, terminalCost, backward_pass, cfg)
    % 管理优化迭代，整理调用顺序，并调整lambda、alpha等参数
    % 输入：
    %   x0: 初始状态
    %   U_init: 初始控制序列
    %   dynamics: 系统动力学模型
    %   stageCost: 过程代价函数
    %   terminalCost: 终端代价函数
    %   backward_pass: 反向传播函数
    %   cfg: 配置参数

    % 输出：
    % sol: 结构体，包含优化结果和相关信息
    % sol.X
    % sol.U
    % sol.totalCost
    % sol.solve_time  求解耗时（秒），包含迭代日志和最终指标计算，不包含绘图/最终汇总打印
    % sol.parts

    % sol.status
    % sol.iterations

    % sol.history.J
    % sol.history.alpha
    % sol.history.lambda

    % sol.metrics.position_error
    % sol.metrics.velocity_error  相对目标速度的误差模长
    % sol.metrics.terminal_speed
    % sol.metrics.max_speed
    % sol.metrics.max_accel
    % sol.metrics.speed_violation
    % sol.metrics.accel_violation

    solve_timer = tic;
    assert(isequal(size(U_init), [2, cfg.model.N]) && ...
           all(isfinite(U_init(:))) && numel(x0) == 4 && all(isfinite(x0(:))), ...
           'DDP 输入无效：x0 必须为有限四维状态，U_init 必须为有限的 2×N 控制序列。');
    % 当前正则化重试使用乘十增长，因此初值必须严格为正。
    assert(cfg.solver.lambda_init > 0 && ...
           cfg.solver.lambda_min > 0 && ...
           cfg.solver.lambda_min <= cfg.solver.lambda_init && ...
           cfg.solver.lambda_init <= cfg.solver.lambda_max && ...
           isfinite(cfg.solver.lambda_max), '正则化参数须满足 0 < min <= init <= max < Inf。');

    % 生成初始控制序列
    U = U_init;
    lambda = cfg.solver.lambda_init;
    alpha_list = cfg.solver.alpha_list;

    % 生成初始状态序列
    [X, totalCost, parts] = rollout(x0, U, dynamics, stageCost, terminalCost, cfg);
    assert(all(isfinite(X(:))) && isfinite(totalCost), ...
           '初始 rollout 产生非有限状态或代价，请检查初值、动力学和代价参数。');

    % 初始化
    status = 'max_iterations';
    history = struct('J', [], 'alpha', [], 'lambda', []);
    history.J(end + 1) = totalCost;
    history.alpha(end + 1) = NaN; % 初始迭代没有alpha
    history.lambda(end + 1) = lambda; % 初始迭代的lambda
    iter = 0;

    % 开始迭代，最多max_iterations次
    for iteration = 1:cfg.solver.max_iter
        iter = iteration;
        % 防止初始控制序列即最优时冗余计算
        grad_inf = control_gradient_norm( ...
        X, U, cfg, dynamics, stageCost, terminalCost);

        if grad_inf <= cfg.solver.tol_grad
            status = 'stationary';
            fprintf('Iteration %d: Gradient norm %.3e is below the threshold %.3e. Stopping.\n', ...
                    iter, grad_inf, cfg.solver.tol_grad);
            break;
        end
        % 进行反向传播
        [kff, Kfb, info] = backward_pass(X, U, cfg, lambda, stageCost);
        while ~info.success
            if lambda >= cfg.solver.lambda_max
                % 达到lambda上限，无法继续增大，返回失败状态
                status = 'backward_fail';
                fprintf('Iteration %d: Backward pass failed and lambda reached maximum limit. Stopping.\n', iter);
                break;
            end
            % 内部重试不增加外层 iter；保留当前已接受轨迹。
            lambda = min(lambda * 10, cfg.solver.lambda_max);
            fprintf('Iteration %d: Backward pass failed (%s). Retrying with lambda = %.3e.\n', ...
                    iter, info.message, lambda);
            [kff, Kfb, info] = backward_pass(X, U, cfg, lambda, stageCost);
        end
        if ~info.success
            break; % 统一在函数末尾整理结果并打印汇总
        end

        accepted = false;          % 每轮线搜索前重置
        % 进行前向传播，尝试不同的alpha
        for alpha = alpha_list
            [X_new, U_new, totalCost_new, parts_new] = forward_pass(X, U, kff, Kfb, alpha, cfg, dynamics, stageCost, terminalCost);
            if isfinite(totalCost_new) && totalCost_new < totalCost
                accepted = true;
                % 成功下降，更新状态和控制序列
                X = X_new;
                U = U_new;
                history.J(end + 1) = totalCost_new;
                history.alpha(end + 1) = alpha;
                history.lambda(end + 1) = lambda;
                
                totalCost = totalCost_new;
                parts = parts_new;
                lambda = max(lambda / 10, cfg.solver.lambda_min);  % 成功下降时，减小lambda
                break; % 成功下降，退出alpha循环
            end
        end
        
        if ~accepted
            % 所有alpha尝试失败，增大lambda并继续下一次迭代
            lambda = min(lambda * 10, cfg.solver.lambda_max);
            fprintf('Iteration %d: No alpha led to cost reduction. Increasing lambda to %.6f and retrying.\n', iter, lambda);
            continue; % 直接继续下一次迭代，不再判断
        end

        fprintf('Iteration %d: totalCost = %.6f, alpha = %.4f, lambda = %.6f\n', ...
                iter, totalCost, history.alpha(end), history.lambda(end));
        % 检查停止条件
        if abs(history.J(end) - history.J(end-1)) / max(1, history.J(end-1)) < cfg.solver.tol_cost
            status = 'success';
            break; % 代价变化小于阈值，停止迭代
        end
                
    end
    
    % 重新评价最终已接受轨迹，避免返回上一轮更新前的梯度。
    grad_inf = control_gradient_norm(X, U, cfg, dynamics, stageCost, terminalCost);
    metrics = compute_metrics(X, U, cfg);
    metrics.control_gradient_inf = grad_inf;
    sol = struct('X', X, 'U', U, 'totalCost', totalCost, 'parts', parts, ...
                             'status', status, 'iterations', iter, ...
                             'history', history, 'metrics', metrics);
    sol.accepted_steps = numel(history.J) - 1;
    sol.solve_time = toc(solve_timer);
    print_summary(sol);

end

function metrics = compute_metrics(X, U, cfg)
    % 本求解器当前针对四维双积分状态 [px; py; vx; vy]。
    pos = X(1:2, end);
    vel = X(3:4, end);
    speed = vecnorm(X(3:4, :), 2, 1);
    x_goal = cfg.task.x_goal(:);
    position_error = norm(pos - x_goal(1:2));
    velocity_error = norm(vel - x_goal(3:4));

    terminal_speed = norm(vel);
    max_speed = max(speed);
    max_accel = max(vecnorm(U,2,1));

    if isfield(cfg, 'limit')
        vmax = getfield(cfg.limit, 'vmax');
        amax = getfield(cfg.limit, 'amax');
    else
        vmax = inf;
        amax = inf;
    end

    speed_violation = max(0, max(speed) - vmax);
    accel_violation = max(0, max_accel - amax);

    metrics = struct('position_error', position_error, ...
                    'velocity_error', velocity_error, ...
                    'terminal_speed', terminal_speed, ...
                    'max_speed', max_speed, ...
                    'max_accel', max_accel, ...
                    'speed_violation', speed_violation, ...
                    'accel_violation', accel_violation);
    avoidance = obstacle_metrics(X, U, cfg);
    names = fieldnames(avoidance);
    for j = 1:numel(names)
        metrics.(names{j}) = avoidance.(names{j});
    end
end

function print_summary(sol)
    fprintf('\n========== DDP 求解结果汇总 ==========\n');
    fprintf('退出状态：%s\n', sol.status);
    fprintf('外层迭代索引：%d；接受更新次数：%d\n', sol.iterations, sol.accepted_steps);
    fprintf('终端位置：[%.6f, %.6f] m\n', sol.X(1,end), sol.X(2,end));
    fprintf('终端速度：[%.6f, %.6f] m/s\n', sol.X(3,end), sol.X(4,end));
    fprintf('终端位置误差：%.9f m\n', sol.metrics.position_error);
    fprintf('终端速度误差：%.9f m/s\n', sol.metrics.velocity_error);
    fprintf('总代价：%.12g\n', sol.totalCost);
    fprintf('求解时长：%.6f s（不含绘图及本汇总打印）\n', sol.solve_time);
    fprintf('最大速度：%.6f m/s；超限量：%.6f m/s\n', ...
            sol.metrics.max_speed, sol.metrics.speed_violation);
    fprintf('最大加速度：%.6f m/s^2；超限量：%.6f m/s^2\n', ...
            sol.metrics.max_accel, sol.metrics.accel_violation);
    fprintf('最终控制梯度无穷范数：%.3e\n', sol.metrics.control_gradient_inf);
    fprintf('最小障碍净距离：%.6f m；发生时间：%.6f s\n', ...
        sol.metrics.min_obstacle_clearance, sol.metrics.min_clearance_time);
    fprintf('最小安全余量：%.6f m；安全距离违反量：%.6f m\n', ...
        sol.metrics.min_safety_margin, sol.metrics.safety_violation);
    fprintf('原始障碍碰撞/接触：%d；碰撞区间数：%d；安全距离不足区间数：%d\n', ...
        sol.metrics.obstacle_collision, sol.metrics.collision_intervals, sol.metrics.unsafe_intervals);
    fprintf('终端障碍净距离：%.6f m；终端避障代价：%.6g\n', ...
        sol.metrics.terminal_clearance, sol.parts.terminal_obstacle);
    if strcmp(sol.status, 'success')
        fprintf('停止原因：相对代价变化达到阈值；约束与终端精度请按上述指标判断。\n');
    end
    fprintf('=====================================\n\n');
end

function grad_inf = control_gradient_norm( ...
    X, U, cfg, dynamics, stageCost, terminalCost)

    N = size(U, 2);
    G = zeros(size(U));

    % terminalCost 第 4 个输出是终端状态梯度
    [~, ~, ~, p] = terminalCost(X(:, end), cfg);

    for i = N:-1:1
        [~, fx, fu] = dynamics(X(:, i), U(:, i), cfg);

        % stageCost 第 5、6 个输出分别是 lx、lu
        [~, ~, ~, ~, lx, lu] = ...
            stageCost(X(:, i), U(:, i), cfg);

        % 此时 p 对应下一时刻，先计算当前控制梯度
        G(:, i) = lu + fu' * p;

        % 再更新到当前时刻
        p = lx + fx' * p;
    end

    if any(~isfinite(G(:)))
        grad_inf = Inf;
    else
        grad_inf = max(abs(G(:)));
    end
end
