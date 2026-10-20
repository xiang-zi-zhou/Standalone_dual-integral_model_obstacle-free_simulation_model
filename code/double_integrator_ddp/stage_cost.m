function [l, cv, cu, l_control, lx, lu, lxx, luu, lux, co] = stage_cost(x, u, cfg)
% 单步代价及其导数
% x: [px; py; vx; vy]，4×1
% u: [ax; ay]，2×1
% 要求 R 为对称矩阵
% 输出：
% l: 单步代价
% cv: 速度超限罚函数值
% cu: 加速度超限罚函数值
% l_control:控制能量代价
h = cfg.model.dt;
R = cfg.cost.R;

p = x(1:2);
v = x(3:4);


% 速度超限罚函数及其关于 v 的导数
[cv, cv_v, cv_vv] = limit_penalty( ...
	v, cfg.limit.vmax, cfg.cost.rho_v);

% 加速度超限罚函数及其关于 u 的导数
[cu, cu_u, cu_uu] = limit_penalty( ...
	u, cfg.limit.amax, cfg.cost.rho_u);

% 避障函数
[co, co_p, co_pp] = obstacle_penalty(cfg, p);

% 1. 单步代价
l_control = h * (0.5 * u' * R * u);
l = h * (cv + cu + co) + l_control;

% 2. 状态梯度：位置避障与速度限制
lx = zeros(4, 1);
lx(1:2) = h * co_p;
lx(3:4) = h * cv_v;

% 3. 控制梯度
lu = h * (R * u + cu_u);

% 4. 状态 Hessian
lxx = zeros(4, 4);
lxx(1:2,1:2) = h * co_pp;
lxx(3:4, 3:4) = h * cv_vv;

% 5. 控制 Hessian
luu = h * (R + cu_uu);

% 6. 混合导数
lux = zeros(2, 4);
end
