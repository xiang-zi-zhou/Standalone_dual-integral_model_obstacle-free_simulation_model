function [kff, Kfb, info] = backward_pass(X, U, cfg, lambda, stageCost)

    % 初始化要存储的x_old,u_old,kff,Kfb
    [nx, ~] = size(X);
    [nu, N] = size(U);

    kff = zeros(nu, N);
    Kfb = zeros(nu, nx, N);
    info = struct('success', false, ...
                'failed_step', [], ...
                'message', '');
    % 计算终点处Vxx,Vx
    [~, ~, ~, Vx, Vxx] = terminal_cost(X(:, end), cfg);


    for i = N:-1:1
        % 取L导数
        x = X(:, i);
        u = U(:, i);
        [~, ~, ~, ~, Lx, Lu, Lxx, Luu, Lux]=stageCost(x, u, cfg);

        [~, fx, fu] = dynamics(x, u, cfg);
        % 双积分模型fxx,fux,fuu是零，因此下面Qxx,Qux,Quu中含该项都省略了

        % 这里Vx都是后一时刻的,即Vx'
        Qx = Lx + fx' * Vx;
        Qu = Lu + fu' * Vx;
        Qxx = Lxx + fx' * Vxx * fx;
        Qux = Lux + fu' * Vxx * fx;
        Quu = Luu + fu' * Vxx * fu;

        % 设置正则化
        Quu = 0.5 * (Quu + Quu');    % 对称化
        H = Quu + lambda * eye(nu);

        % 检查正定性
        [L,flag] = chol(H,'lower');
        if flag == 0
            % 计算增益
            kff(:, i) = - (L' \ (L \ Qu));
            Kfb(:, :, i) = - (L' \ (L \ Qux));

        else
            % 终止迭代，返回失败
            info.failed_step = i;
            info.message = 'H = Q_uu + lambda*I 非正定，请增大 lambda 后重试';
            return
        end

        % 更新价值函数 用 \(H\) 求 \(k,K\)，再把这个策略代入原始二次模型，计算价值函数近似。
        % 与论文中简化后的更新策略有差异了
        Vx = Qx + Kfb(:, :, i)' * Quu * kff(:, i) + Kfb(:, :, i)' * Qu + Qux' * kff(:, i);
        Vxx = Qxx + Kfb(:, :, i)' * Quu * Kfb(:, :, i) + Kfb(:, :, i)' * Qux + Qux' * Kfb(:, :, i);
        Vxx = 0.5 * (Vxx + Vxx');


    end

    info.success = true;
    info.message = '反向传播成功。';

end