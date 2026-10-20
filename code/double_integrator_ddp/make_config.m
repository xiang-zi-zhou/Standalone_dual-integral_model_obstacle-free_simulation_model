function cfg = make_config()

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

    % 障碍物
    cfg.world.obstacle1 = [12, 8; 18, 20]; % 矩形障碍物，左下角坐标为(12, 8)，右上角坐标为(18, 20)
    cfg.world.obstacle2 = [15, 10; 23, 18]; % 原始正方形：[15,23]×[10,18]，绕世界原点逆时针旋转9°
    % task_one障碍物
    cfg.world.obstacle3 = [15, 14, 3]; % 圆形障碍物，圆心为(15, 14)，半径为3米 
    
    %% ====task
    % 状态变量x = [x; y; vx; vy]
    cfg.task.x0 = [0; 0; 0; 0];
    cfg.task.x_goal = [30; 30; 0; 0];
    cfg.task.obs = cfg.world.obstacle3;
    cfg.task.obs_safe = 0.5;

    cfg.cost.Qfx = [80, 0; 0, 80];
    cfg.cost.Qfv = [50, 0; 0, 50];
    cfg.cost.R = [1, 0; 0, 1];
    cfg.cost.rho_v = 20;
    cfg.cost.rho_u = 80;
    cfg.cost.rho_obs = 50;
    cfg.cost.rho_obs_terminal = 50; % 独立终端避障权重，不乘 dt

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
