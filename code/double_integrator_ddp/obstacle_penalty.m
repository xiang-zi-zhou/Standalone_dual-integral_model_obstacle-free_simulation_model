function [c, cp, cpp] = obstacle_penalty(cfg, p)
    obs = cfg.task.obs;
    center = obs(1:2)';
    r = obs(3);
    safe_r = r + cfg.task.obs_safe;
    r2 = safe_r^2;
    d = p - center;
    g = 1 - (d.'*d)/r2;
    rho = cfg.cost.rho_obs;

    c = 0;
    cp  = zeros(2,1);
    cpp = zeros(2,2);

    if g > 0
        c   = rho*g^2;
        cp  = -4*rho*g/r2*d;
        cpp = 8*rho/r2^2*(d*d.') - 4*rho*g/r2*eye(2);
    end
end