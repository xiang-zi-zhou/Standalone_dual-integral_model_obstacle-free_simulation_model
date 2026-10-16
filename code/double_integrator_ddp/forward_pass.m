function [X_new, U_new, totalCost, parts] = forward_pass(X, U, kff, Kfb, alpha, cfg, dynamics, stageCost, terminalCost)
    % 前向传播，计算新的状态序列和控制序列
    % 输入：
    %   X: 当前状态序列
    %   U: 当前控制序列
    %   kff: feedforward增益
    %   Kfb: feedback增益
    %   alpha: 步长
    % 输出：
    %   X_new: 新的状态序列
    %   U_new: 新的控制序列
    %   totalCost: 总代价
    %   parts: 代价组成部分

    N = size(U, 2);
    nx = size(X, 1);
    nu = size(U, 1);

    X_new = zeros(nx, N + 1);
    U_new = zeros(nu, N);
    X_new(:, 1) = X(:, 1);  % 初始状态不变

    for i = 1:N
        u_nominal = U(:, i);
        x_nominal = X(:, i);

        % 更新控制输入
        u_new = u_nominal + alpha * kff(:, i) + Kfb(:, :, i) * (X_new(:, i) - x_nominal);
        U_new(:, i) = u_new;

        % 更新状态
        x_new = dynamics(X_new(:, i), u_new, cfg);
        X_new(:, i + 1) = x_new;
    end

    % 计算总代价,这里重复计算了一遍动力学，后续可改进
    [~, totalCost, parts] = rollout(X_new(:, 1), U_new, dynamics, stageCost, terminalCost, cfg);
end