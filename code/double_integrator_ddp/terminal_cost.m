function [phi, phi_pos, phi_vel, phix, phixx, phi_obs] = terminal_cost(x, cfg)
    p = x(1:2); % 终点位置
    v = x(3:4); % 终点速度
    phi_pos = 0.5 * (p - cfg.task.x_goal(1:2))' * cfg.cost.Qfx * (p - cfg.task.x_goal(1:2));
    phi_vel = 0.5 * (v - cfg.task.x_goal(3:4))' * cfg.cost.Qfv * (v - cfg.task.x_goal(3:4));
    % 终端项不乘 dt；保留前五个输出的位置，兼容反向传播。
    terminal_cfg = cfg;
    terminal_cfg.cost.rho_obs = cfg.cost.rho_obs_terminal;
    [phi_obs, obs_p, obs_pp] = obstacle_penalty(terminal_cfg, p);
    phi = phi_pos + phi_vel + phi_obs;
    phix = [cfg.cost.Qfx * (p - cfg.task.x_goal(1:2)); cfg.cost.Qfv * (v - cfg.task.x_goal(3:4))];
    phixx = [cfg.cost.Qfx, zeros(2, 2); zeros(2, 2), cfg.cost.Qfv];
    phix(1:2) = phix(1:2) + obs_p;
    phixx(1:2,1:2) = phixx(1:2,1:2) + obs_pp;
end
