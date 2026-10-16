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

    %% ====task
    % 状态变量x = [x; y; vx; vy]
    cfg.task.x0 = [0; 0; 0; 0];
    cfg.task.x_goal = [30; 30; 0; 0];

    cfg.cost.Qfx = [80, 0; 0, 80];
    cfg.cost.Qfv = [50, 0; 0, 50];
    cfg.cost.R = [1, 0; 0, 1];
    cfg.cost.rho_v = 20;
    cfg.cost.rho_u = 80;

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