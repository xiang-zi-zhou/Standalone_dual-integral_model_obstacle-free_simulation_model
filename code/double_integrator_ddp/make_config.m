function cfg = make_config(scene_id)
% 默认使用斜向障碍墙；可传入 obstacle_scenarios 中的场景 id。
% 例如 make_config('original') 或 make_config('asymmetric_cluster')。
    if nargin < 1
        scene_id = 'diagonal_wall';
    end

    %% ====dynamics
    T_desired = 12;           % 希望的总规划时间，单位s
    cfg.model.dt = 0.05;      % 每隔0.05s更新一次加速度
    cfg.model.N = round(T_desired / cfg.model.dt); % round四舍五入取整
    cfg.model.T_actual = cfg.model.N * cfg.model.dt;
    dt = cfg.model.dt;
    cfg.model.A = [ [1, 0, dt, 0];
                    [0, 1, 0, dt];
                    [0, 0, 1, 0];
                    [0, 0, 0, 1]];
    dt2 = 0.5*dt*dt;
    cfg.model.B = [[dt2, 0];[0, dt2];[dt, 0];[0, dt]];

    % a*初始化世界
    cfg.world.x = 30;
    cfg.world.y = 30;
    cfg.world.resolution = 4; % 每米4个格子，即每格0.25米

    
    %% ====task
    % 状态变量x = [x; y; vx; vy]
    cfg.task.x0 = [0; 0; 0; 0];
    cfg.task.x_goal = [30; 30; 0; 0];
    % 每行：[圆心x, 圆心y, 半径]，单位m；统一从场景库读取。
    scene_id = 'asymmetric_cluster';
    scenarios = obstacle_scenarios();
    selected = find(strcmp({scenarios.id}, scene_id), 1);
    assert(~isempty(selected), '未知场景，请查看 obstacle_scenarios.m 中的 id。');
    cfg.task.scene_id = scenarios(selected).id;
    cfg.task.obs = scenarios(selected).obs;
    cfg.task.obs_safe = 0.5;

    cfg.cost.Qfx = [80, 0; 0, 80];
    cfg.cost.Qfv = [50, 0; 0, 50];
    % 障碍墙调参：降低控制能量权重，提升避障及运动限制的重要性。
    % 对比记录：results/20261021_weight_tuning_run02/obs1000_limits2p5
    cfg.cost.R = 0.02 * eye(2); % 原I的1/50；dt和二次型前的1/2保持不变
    cfg.cost.rho_v = 50;       % 原20的2.5倍
    cfg.cost.rho_u = 200;      % 原80的2.5倍
    cfg.cost.rho_obs = 1000;   % 原50的20倍，仍为软约束
    cfg.cost.rho_obs_terminal = 1000; % 独立终端避障权重，不乘 dt

    cfg.limit.vmax = 5;
    cfg.limit.amax = 2;

    cfg.solver.max_iter = 100;
    cfg.solver.alpha_list = [1, 0.5, 0.25, 0.125, 0.0625]; % 从大步长开始，遇到下降就接受
    cfg.solver.lambda_init = 1e-3;
    cfg.solver.lambda_min = 1e-6;
    cfg.solver.lambda_max = 1e6;
    cfg.solver.tol_cost = 1e-6;
    cfg.solver.tol_grad = 1e-6; 

    

end
