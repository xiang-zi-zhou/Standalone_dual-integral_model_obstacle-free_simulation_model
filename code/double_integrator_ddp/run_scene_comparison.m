function summary = run_scene_comparison(output_dir, repetitions)
% Reproduce all scenes with identical costs/limits/time; preserve every run.
% Example: run_scene_comparison('C:/.../results/my_run', 3)
    if nargin < 2, repetitions = 3; end
    assert(repetitions >= 1 && repetitions == floor(repetitions));
    assert(~isfolder(output_dir), 'Output directory already exists; choose a new directory.');
    mkdir(output_dir);
    source_dir = fileparts(mfilename('fullpath'));
    copyfile(fullfile(source_dir, '*.m'), fullfile(output_dir, 'source'));
    old_visibility = get(groot, 'defaultFigureVisible');
    set(groot, 'defaultFigureVisible', 'off');
    cleanup = onCleanup(@() set(groot, 'defaultFigureVisible', old_visibility)); %#ok<NASGU>
    base_cfg = make_config();
    scenarios = obstacle_scenarios();
    save(fullfile(output_dir, 'experiment_config.mat'), 'base_cfg', 'scenarios', 'repetitions');
    rows = struct([]);
    examples = cell(numel(scenarios), 2);
    methods = {'astar', 'zero'};
    for s = 1:numel(scenarios)
        scene = scenarios(s);
        cfg = base_cfg; cfg.task.obs = scene.obs; cfg.task.scene_id = scene.id;
        scene_dir = fullfile(output_dir, scene.id); mkdir(scene_dir);
        save(fullfile(scene_dir, 'config.mat'), 'cfg', 'scene');
        write_text(fullfile(scene_dir, 'config.json'), jsonencode(cfg, 'PrettyPrint', true));
        for rep = 1:repetitions
            % Alternate execution order to reduce systematic warm-up bias.
            order = 1:2; if mod(rep,2) == 0, order = 2:-1:1; end
            for method_index = order
                method = methods{method_index};
                prefix = fullfile(scene_dir, sprintf('%s_rep%d', method, rep));
                fprintf('RUN %s %s repetition %d/%d\n', scene.id, method, rep, repetitions);
                astar = []; init_text = ''; solve_text = '';
                try
                    init_timer = tic;
                    if method_index == 1
                        init_text = evalc('astar = astar_initialization(cfg); U0 = astar_path2u0(astar, cfg);');
                    else
                        U0 = zeros(2, cfg.model.N);
                    end
                    init_seconds = toc(init_timer);
                    [X0, J0] = rollout(cfg.task.x0, U0, @dynamics, @stage_cost, @terminal_cost, cfg);
                    initial_metrics = obstacle_metrics(X0,U0,cfg);
                    solve_text = evalc(['sol = solve_ddp(cfg.task.x0, U0, @dynamics, ' ...
                        '@stage_cost, @terminal_cost, @backward_pass, cfg);']);
                    write_text([prefix '.log'], [init_text, solve_text]);
                    save([prefix '.mat'], 'cfg', 'scene', 'sol', 'astar', 'U0', 'X0', ...
                        'J0', 'initial_metrics', 'init_seconds');
                    row = struct('scene', scene.id, 'scene_name', scene.name, ...
                        'method', method, 'repetition', rep, 'status', sol.status, ...
                        'cost', sol.totalCost, 'iterations', sol.iterations, ...
                        'accepted_steps', sol.accepted_steps, 'init_s', init_seconds, ...
                        'solve_s', sol.solve_time, 'total_s', init_seconds+sol.solve_time, ...
                        'clearance_m', sol.metrics.min_obstacle_clearance, ...
                        'safety_margin_m', sol.metrics.min_safety_margin, ...
                        'collision', sol.metrics.obstacle_collision, ...
                        'unsafe_intervals', sol.metrics.unsafe_intervals, ...
                        'position_error_m', sol.metrics.position_error, ...
                        'velocity_error_mps', sol.metrics.velocity_error, ...
                        'speed_violation_mps', sol.metrics.speed_violation, ...
                        'accel_violation_mps2', sol.metrics.accel_violation, ...
                        'initial_cost', J0, 'initial_clearance_m', initial_metrics.min_obstacle_clearance);
                    if isempty(rows)
                        rows = row;
                    else
                        rows(end+1) = row; %#ok<AGROW>
                    end
                    if rep == 1
                        examples{s,method_index} = struct('sol', sol, 'astar', astar, 'X0', X0);
                        figs = plot_results(sol,cfg);
                        names = fieldnames(figs);
                        for f = 1:numel(names)
                            fig = figs.(names{f});
                            exportgraphics(fig, [prefix '_' names{f} '.png'], 'Resolution', 140);
                            savefig(fig, [prefix '_' names{f} '.fig']); close(fig);
                        end
                    end
                    writetable(struct2table(rows), fullfile(output_dir, 'all_runs.csv'));
                catch err
                    write_text([prefix '_error.log'], [init_text, solve_text, getReport(err, 'extended', 'hyperlinks','off')]);
                    rethrow(err);
                end
            end
        end
        fig = comparison_plot(cfg, scene, examples{s,1}, examples{s,2});
        exportgraphics(fig, fullfile(scene_dir,'comparison.png'), 'Resolution',150);
        savefig(fig, fullfile(scene_dir,'comparison.fig')); close(fig);
    end
    summary = struct2table(rows);
    write_text(fullfile(output_dir,'all_runs.json'), jsonencode(rows, 'PrettyPrint',true));
    save(fullfile(output_dir,'all_results.mat'), 'summary', 'examples', 'scenarios', 'base_cfg');
    fig = figure('Color','w','Position',[50 50 1500 1100]);
    tiledlayout(3,3,'TileSpacing','compact');
    for s = 1:numel(scenarios)
        nexttile; cfg = base_cfg; cfg.task.obs = scenarios(s).obs;
        draw_scene(cfg,examples{s,1},examples{s,2},false);
        title(sprintf('%s\nJ: A*=%.2f / Zero=%.2f', scenarios(s).name, ...
            examples{s,1}.sol.totalCost, examples{s,2}.sol.totalCost));
    end
    sgtitle('相同参数，仅改变障碍物布局：蓝色 A*初始化，红色 零初值DDP');
    exportgraphics(fig, fullfile(output_dir,'overview.png'), 'Resolution',150);
    savefig(fig, fullfile(output_dir,'overview.fig')); close(fig);
    fprintf('COMPLETED: %d runs saved in %s\n',height(summary),output_dir);
end

function fig = comparison_plot(cfg,scene,a,z)
    fig = figure('Color','w','Position',[50 50 1250 900]);
    tiledlayout(2,2,'TileSpacing','compact');
    nexttile; draw_scene(cfg,a,z,true); title(scene.name);
    nexttile; hold on;
    semilogy(0:numel(a.sol.history.J)-1,a.sol.history.J,'b-o','MarkerSize',3);
    semilogy(0:numel(z.sol.history.J)-1,z.sol.history.J,'r-o','MarkerSize',3);
    set(gca,'YScale','log'); grid on; xlabel('接受更新次数'); ylabel('总代价（对数）');
    legend('A*初始化','零初值'); title('收敛过程（含初始代价）');
    nexttile; hold on; t = (0:cfg.model.N)*cfg.model.dt;
    plot(t,a.sol.metrics.node_clearance,'b-','LineWidth',1.5);
    plot(t,z.sol.metrics.node_clearance,'r-','LineWidth',1.5);
    yline(cfg.task.obs_safe,'--','安全距离'); yline(0,'k:','碰撞边界');
    grid on; xlabel('时间 (s)'); ylabel('到最近障碍物的节点净距离 (m)');
    legend('A*初始化','零初值','Location','best');
    title(sprintf('连续区间最小净距离：A* %.3f / Zero %.3f m', ...
        a.sol.metrics.min_obstacle_clearance,z.sol.metrics.min_obstacle_clearance));
    nexttile; hold on;
    plot(t,vecnorm(a.sol.X(3:4,:),2,1),'b-','LineWidth',1.5);
    plot(t,vecnorm(z.sol.X(3:4,:),2,1),'r-','LineWidth',1.5);
    yline(cfg.limit.vmax,'k--','速度上限'); grid on;
    xlabel('时间 (s)'); ylabel('速度模长 (m/s)'); legend('A*初始化','零初值');
    title(sprintf('终点误差：A* %.4f / Zero %.4f m', ...
        a.sol.metrics.position_error,z.sol.metrics.position_error));
end

function draw_scene(cfg,a,z,show_initial)
    hold on; theta = linspace(0,2*pi,160); circle = [cos(theta);sin(theta)];
    for i = 1:size(cfg.task.obs,1)
        c = cfg.task.obs(i,1:2).'; r = cfg.task.obs(i,3);
        p = c+r*circle; q = c+(r+cfg.task.obs_safe)*circle;
        fill(p(1,:),p(2,:),[.65 .65 .65],'EdgeColor',[.3 .3 .3],'HandleVisibility','off');
        plot(q(1,:),q(2,:),'--','Color',[.85 .55 .1],'HandleVisibility','off');
        text(c(1),c(2),sprintf('%d',i),'HorizontalAlignment','center');
    end
    if show_initial
        plot(a.astar.pathXY(:,1),a.astar.pathXY(:,2),':','Color',[.2 .65 .2], ...
            'DisplayName','A*栅格路径');
        plot(a.X0(1,:),a.X0(2,:),'--','Color',[.3 .75 .8], ...
            'DisplayName','A*拟合后的初始轨迹');
    end
    plot(a.sol.X(1,:),a.sol.X(2,:),'b-','LineWidth',1.8,'DisplayName','A*初始化DDP');
    plot(z.sol.X(1,:),z.sol.X(2,:),'r-','LineWidth',1.5,'DisplayName','零初值DDP');
    plot(cfg.task.x0(1),cfg.task.x0(2),'go','MarkerFaceColor','g','HandleVisibility','off');
    plot(cfg.task.x_goal(1),cfg.task.x_goal(2),'kx','LineWidth',2,'HandleVisibility','off');
    axis equal; grid on; xlabel('x (m)'); ylabel('y (m)');
    if show_initial, legend('Location','best'); end
end

function write_text(path,content)
    fid = fopen(path,'w','n','UTF-8'); assert(fid >= 0);
    cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
    fprintf(fid,'%s',content);
end
