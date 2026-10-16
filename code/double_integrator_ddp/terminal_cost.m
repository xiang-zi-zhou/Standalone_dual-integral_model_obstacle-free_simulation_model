function [phi, phi_pos, phi_vel, phix, phixx] = terminal_cost(x, cfg)
    p = x(1:2); % 终点位置
    v = x(3:4); % 终点速度
    phi_pos = 0.5 * (p - cfg.task.x_goal(1:2))' * cfg.cost.Qfx * (p - cfg.task.x_goal(1:2));
    phi_vel = 0.5 * (v - cfg.task.x_goal(3:4))' * cfg.cost.Qfv * (v - cfg.task.x_goal(3:4));
    phi = phi_pos + phi_vel;
    phix = [cfg.cost.Qfx * (p - cfg.task.x_goal(1:2)); cfg.cost.Qfv * (v - cfg.task.x_goal(3:4))];
    phixx = [cfg.cost.Qfx, zeros(2, 2); zeros(2, 2), cfg.cost.Qfv];
end