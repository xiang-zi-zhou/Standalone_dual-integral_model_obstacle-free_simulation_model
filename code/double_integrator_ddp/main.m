cfg = make_config();

x0 = cfg.task.x0;
U0 = zeros(2, cfg.model.N); % 初始控制序列生成 

sol = solve_ddp(x0, U0, @dynamics, @stage_cost, @terminal_cost, @backward_pass, cfg);

figs = plot_results(sol, cfg);