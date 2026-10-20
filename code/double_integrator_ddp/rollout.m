function [X, totalCost, parts] = rollout(x0, U, dynamics, stageCost, terminalCost, cfg)
%% MATLAB 中 X(:,1) 对应数学上的 \(x_0\)，X(:,N+1) 对应 \(x_N\)
x = x0(:);     % 当前状态
N = size(U, 2);   % 控制序列的列数，即总步长数
X = zeros(numel(x), N + 1);  % 保存整个状态序列
X(:, 1) = x;
stageCosts = zeros(1, N);
speed_penalty = zeros(1, N);
accel_penalty = zeros(1, N);
control = zeros(1, N);
obstacle = zeros(1, N);
h = cfg.model.dt;

for k = 1:N
	u = U(:, k);
	[l, cv, cu, l_control, ~, ~, ~, ~, ~, co] = stageCost(x, u, cfg);
    obstacle(k) = h * co;
	stageCosts(k) = l;
	speed_penalty(k) = h * cv; % 速度罚函数值
	accel_penalty(k) = h * cu; % 加速度罚函数值
	control(k) = l_control; % 控制能量代价
	x = dynamics(x, u, cfg);
	x = x(:);
	X(:, k + 1) = x;
end

[terminal, phi_pos, phi_vel, ~, ~, phi_obs] = terminalCost(x, cfg);
stageTotal = sum(stageCosts);
totalCost = stageTotal + terminal;
parts = struct('terminal_position', phi_pos, ...
               'terminal_obstacle', phi_obs, 'obstacle_penalty', obstacle, ...
               'terminal_velocity', phi_vel, ...
			   'control', control, ...
			   'speed_penalty', speed_penalty, ...
			   'accel_penalty', accel_penalty, ...
               'stageCosts', stageCosts, ... % 过程代价函数序列
			   'stageCost_sum', stageTotal, ... % 过程代价总和
			   'terminalCost', terminal, ...
			   'totalCost', totalCost);
end
