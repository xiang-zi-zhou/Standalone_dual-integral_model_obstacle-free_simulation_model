function [U0] = astar_path2u0(astar, cfg)
    % 将 A* 粗路径转换为初始控制序列 u0
    pathXY = astar.pathXY; % 获取 A* 粗路径点坐标
    N = cfg.model.N; % 控制步数
    dt = cfg.model.dt; % 时间步长
    L = astar.pathLength; % 粗路径总长度
    x0 = cfg.task.x0;
    xg = cfg.task.x_goal;

    s = [0; cumsum(vecnorm(diff(pathXY), 2, 2))]; % 路径点的累积弧长
    tau = linspace(0, 1, N + 1).'; % 归一化时间 t/T，范围必须为 [0, 1]
    progress = 3*tau.^2 - 2*tau.^3; % 进度函数，范围 [0, 1] % .^ 表示逐元素乘方
    Pref = interp1(s, pathXY, L*progress, "linear").'; % 完成插值
    assert(all(isfinite(Pref(:))), ...
        '参考路径含 NaN 或 Inf，请检查弧长插值范围与路径数据。');

    % 3. 位置预测矩阵：第 k 行对应 p_k
    G = zeros(N,N);
    for k = 1:N
        G(k,1:k) = dt^2 * (k - (1:k) + 0.5);
    end

    % 不施加控制时的位置
    time = (1:N).' * dt;
    base = ones(N,1)*x0(1:2).' + time*x0(3:4).';

    % 4. 正则化最小二乘，额外强调终端位置和速度
    lambda = 0.01;  % 加速度正则化权重，需结合结果调整
    wGoal  = 100;   % 终端位置额外权重
    wVel   = 10;    % 终端速度权重

    Afit = [G;
            sqrt(wGoal)*G(end,:);
            sqrt(wVel)*dt*ones(1,N);
            sqrt(lambda)*eye(N)];

    Bfit = [Pref(:,2:end).' - base;
            sqrt(wGoal)*(xg(1:2).' - base(end,:));
            sqrt(wVel)*(xg(3:4).' - x0(3:4).');
            zeros(N,2)];

    U0 = (Afit \ Bfit).';
    assert(all(isfinite(U0(:))), '初始控制序列含 NaN 或 Inf。');

    % 5. 严格按现有离散动力学生成初始状态
    X0 = zeros(4,N+1);
    X0(:,1) = x0;
    for k = 1:N
        X0(:,k+1) = cfg.model.A*X0(:,k) + cfg.model.B*U0(:,k);
    end

end
