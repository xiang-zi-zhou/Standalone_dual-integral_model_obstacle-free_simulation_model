cfg = make_config();

x0 = cfg.task.x0;

% 接入A*初始粗路径生成
astar = astar_initialization(cfg);
astar_check(astar)
U0 = astar_path2u0(astar, cfg); % 初始控制序列生成 

sol = solve_ddp(x0, U0, @dynamics, @stage_cost, @terminal_cost, @backward_pass, cfg);

figs = plot_results(sol, cfg);

% 对比零初值
U02 = zeros(2, cfg.model.N); % 初始控制序列生成 
sol2 = solve_ddp(x0, U02, @dynamics, @stage_cost, @terminal_cost, @backward_pass, cfg);
figs2 = plot_results(sol2, cfg);
