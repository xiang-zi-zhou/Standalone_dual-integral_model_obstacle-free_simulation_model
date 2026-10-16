function [xn, fx, fu] = dynamics(x, u, cfg)
    fx = cfg.model.A;
    fu = cfg.model.B;
    xn = fx * x + fu * u;
end