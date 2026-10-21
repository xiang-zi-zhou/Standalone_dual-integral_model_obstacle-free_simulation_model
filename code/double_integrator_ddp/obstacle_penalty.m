function [c, cp, cpp] = obstacle_penalty(cfg, p)
    obs = cfg.task.obs;
    rho = cfg.cost.rho_obs;

    c = 0;
    cp  = zeros(2,1);
    cpp = zeros(2,2);

    for i = 1:size(obs, 1)
        center = obs(i, 1:2)';
        safe_r = obs(i, 3) + cfg.task.obs_safe;
        r2 = safe_r^2;

        d = p - center;
        g = 1 - (d.'*d)/r2;

        if g > 0
            c   = c + rho*g^2;
            cp  = cp - 4*rho*g/r2*d;
            cpp = cpp + 8*rho/r2^2*(d*d.') - 4*rho*g/r2*eye(2);
        end
    end
end
