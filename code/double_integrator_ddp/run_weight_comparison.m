function summary = run_weight_comparison(output_dir, weights, ids)
% Fixed wall scene, identical solver settings, paired initializations per weight set.
% Preserve all configurations, solver logs, trajectories, component costs and figures.
    assert(~isfolder(output_dir), 'Choose a new output directory.');
    mkdir(output_dir);
    copyfile(fullfile(fileparts(mfilename('fullpath')), '*.m'), fullfile(output_dir,'source'));
    visibility = get(groot,'defaultFigureVisible');
    set(groot,'defaultFigureVisible','off');
    cleanup = onCleanup(@() set(groot,'defaultFigureVisible',visibility)); %#ok<NASGU>
    % Columns: R diagonal, rho_obs, rho_v, rho_u, Qfx diagonal, Qfv diagonal.
    if nargin < 2
        weights = [
        1,    50,   20,   80,  80,  50;
        .02,  50,   20,   80,  80,  50;
        .02, 250,   20,   80,  80,  50;
        .02,1000,   20,   80,  80,  50;
        .02,5000,   20,   80,  80,  50;
        .02,1000,  200,  800,  80,  50;
        .02,5000, 2000, 8000,  80,  50;
        .02,1000,  200,  800, 800, 500
        ];
        ids = {'baseline','energy_div50','obs250','obs1000','obs5000', ...
            'balanced','strong_limits','terminal10'};
    end
    assert(size(weights,2)==6 && size(weights,1)==numel(ids) && numel(ids)<=8);
    methods = {'astar','zero'};
    part_names = {'control','speed_penalty','accel_penalty','terminal_position', ...
        'terminal_velocity','obstacle_penalty','terminal_obstacle'};
    cfg_base = make_config('diagonal_wall');
    all_solutions = cell(numel(ids),2); configs = cell(numel(ids),1);
    rows = struct([]); frozen_rows = struct([]);
    % Geometry and initialization do not depend on cost weights. Reuse identical seeds.
    init_timer = tic;
    astar = astar_initialization(cfg_base);
    U_astar = astar_path2u0(astar,cfg_base);
    astar_init_s = toc(init_timer);
    save(fullfile(output_dir,'design.mat'),'weights','ids','cfg_base','astar','U_astar','astar_init_s');
    for k = 1:numel(ids)
        cfg = cfg_base;
        cfg.cost.R = weights(k,1)*eye(2);
        cfg.cost.rho_obs = weights(k,2);
        cfg.cost.rho_obs_terminal = weights(k,2);
        cfg.cost.rho_v = weights(k,3); cfg.cost.rho_u = weights(k,4);
        cfg.cost.Qfx = weights(k,5)*eye(2); cfg.cost.Qfv = weights(k,6)*eye(2);
        configs{k} = cfg;
        folder = fullfile(output_dir,ids{k}); mkdir(folder);
        write_text(fullfile(folder,'config.json'),jsonencode(cfg,'PrettyPrint',true));
        for method_index = 1:2
            method = methods{method_index};
            fprintf('RUN %s / %s\n',ids{k},method);
            if method_index == 1, U0 = U_astar; else, U0 = zeros(2,cfg.model.N); end
            [X0,J0,initial_parts] = rollout(cfg.task.x0,U0,@dynamics,@stage_cost,@terminal_cost,cfg);
            log_text = evalc(['sol = solve_ddp(cfg.task.x0,U0,@dynamics,@stage_cost,' ...
                '@terminal_cost,@backward_pass,cfg);']);
            write_text(fullfile(folder,[method '.log']),log_text);
            save(fullfile(folder,[method '.mat']),'cfg','sol','U0','X0','J0','initial_parts','astar');
            all_solutions{k,method_index} = sol;
            row = struct('profile',ids{k},'method',method,'R',weights(k,1), ...
                'rho_obs',weights(k,2),'rho_v',weights(k,3),'rho_u',weights(k,4), ...
                'Qfx',weights(k,5),'Qfv',weights(k,6),'cost',sol.totalCost, ...
                'status',sol.status,'iterations',sol.iterations,'solve_s',sol.solve_time, ...
                'clearance_m',sol.metrics.min_obstacle_clearance, ...
                'safety_margin_m',sol.metrics.min_safety_margin, ...
                'collision',sol.metrics.obstacle_collision, ...
                'position_error_m',sol.metrics.position_error, ...
                'velocity_error_mps',sol.metrics.velocity_error, ...
                'speed_violation_mps',sol.metrics.speed_violation, ...
                'accel_violation_mps2',sol.metrics.accel_violation, ...
                'gradient_inf',sol.metrics.control_gradient_inf, ...
                'raw_control_integral',0.5*cfg.model.dt*sum(sol.U(:).^2));
            component_sum = 0;
            for j = 1:numel(part_names)
                row.(part_names{j}) = sum(sol.parts.(part_names{j})(:));
                component_sum = component_sum + row.(part_names{j});
            end
            assert(abs(component_sum-sol.totalCost) < 1e-8*max(1,abs(sol.totalCost)));
            if isempty(rows), rows = row; else, rows(end+1) = row; end %#ok<AGROW>
            % Reprice the unchanged baseline trajectory to separate weight effects
            % from changes to the optimized trajectory. Never compare optimized J
            % across different objectives as if they were the same quantity.
            old = all_solutions{1,method_index};
            [~,fixed_cost,fixed_parts] = rollout(cfg.task.x0,old.U,@dynamics,@stage_cost,@terminal_cost,cfg);
            frozen = struct('profile',ids{k},'baseline_trajectory',method,'cost',fixed_cost);
            for j = 1:numel(part_names)
                frozen.(part_names{j}) = sum(fixed_parts.(part_names{j})(:));
            end
            if isempty(frozen_rows), frozen_rows = frozen; else, frozen_rows(end+1) = frozen; end %#ok<AGROW>
            writetable(struct2table(rows),fullfile(output_dir,'all_runs.csv'));
            writetable(struct2table(frozen_rows),fullfile(output_dir,'frozen_baseline_costs.csv'));
            figs = plot_results(sol,cfg); names = fieldnames(figs);
            for j = 1:numel(names)
                fig = figs.(names{j});
                exportgraphics(fig,fullfile(folder,[method '_' names{j} '.png']),'Resolution',130);
                savefig(fig,fullfile(folder,[method '_' names{j} '.fig'])); close(fig);
            end
        end
        fig = figure('Color','w','Position',[40 40 1250 900]);
        tiledlayout(2,2,'TileSpacing','compact');
        a = all_solutions{k,1}; z = all_solutions{k,2};
        nexttile; draw_paths(cfg,a,z); title(ids{k},'Interpreter','none');
        nexttile; semilogy(0:numel(a.history.J)-1,a.history.J,'b-o'); hold on;
        semilogy(0:numel(z.history.J)-1,z.history.J,'r-o'); grid on;
        xlabel('接受更新次数'); ylabel('总代价（对数）'); legend('A*','Zero');
        nexttile; t=(0:cfg.model.N)*cfg.model.dt;
        plot(t,a.metrics.node_clearance,'b-',t,z.metrics.node_clearance,'r-','LineWidth',1.3);
        hold on; yline(.5,'--'); yline(0,'k:'); grid on;
        xlabel('时间 (s)'); ylabel('节点最小净距离 (m)');
        title(sprintf('连续最小净距离 A* %.4f / Zero %.4f m', ...
            a.metrics.min_obstacle_clearance,z.metrics.min_obstacle_clearance));
        legend('A*','Zero','安全距离','碰撞边界','Location','best');
        nexttile; components=zeros(2,numel(part_names));
        for j=1:numel(part_names)
            components(1,j)=sum(a.parts.(part_names{j})(:));
            components(2,j)=sum(z.parts.(part_names{j})(:));
        end
        bar(components,'stacked'); set(gca,'XTick',1:2,'XTickLabel',{'A*','Zero'});
        legend({'控制','速度罚项','加速度罚项','终点位置','终点速度','过程避障','终端避障'},'Location','eastoutside');
        ylabel('加权代价'); title(sprintf('总代价 A* %.4g / Zero %.4g',a.totalCost,z.totalCost));
        exportgraphics(fig,fullfile(folder,'comparison.png'),'Resolution',140);
        savefig(fig,fullfile(folder,'comparison.fig')); close(fig);
    end
    summary = struct2table(rows);
    save(fullfile(output_dir,'all_results.mat'),'summary','all_solutions','configs','frozen_rows','weights','ids');
    write_text(fullfile(output_dir,'all_runs.json'),jsonencode(rows,'PrettyPrint',true));
    fig = figure('Color','w','Position',[40 40 1600 850]); tiledlayout(2,4,'TileSpacing','compact');
    for k=1:numel(ids)
        nexttile; a=all_solutions{k,1}; z=all_solutions{k,2};
        draw_paths(configs{k},a,z);
        title(sprintf('%s\nmin distance A* %.3f / Zero %.3f',ids{k}, ...
            a.metrics.min_obstacle_clearance,z.metrics.min_obstacle_clearance),'Interpreter','none');
    end
    sgtitle('同一障碍墙、不同权重；蓝色 A* 初始化，红色零初值');
    exportgraphics(fig,fullfile(output_dir,'overview.png'),'Resolution',140);
    savefig(fig,fullfile(output_dir,'overview.fig')); close(fig);
    fprintf('WEIGHT_SWEEP_COMPLETE: %d runs\n',height(summary));
end

function draw_paths(cfg,a,z)
    hold on; theta=linspace(0,2*pi,160); circle=[cos(theta);sin(theta)];
    for j=1:size(cfg.task.obs,1)
        c=cfg.task.obs(j,1:2).'; r=cfg.task.obs(j,3);
        p=c+r*circle; q=c+(r+cfg.task.obs_safe)*circle;
        fill(p(1,:),p(2,:),[.7 .7 .7],'EdgeColor',[.3 .3 .3]);
        plot(q(1,:),q(2,:),'--','Color',[.9 .55 .1]);
    end
    plot(a.X(1,:),a.X(2,:),'b-','LineWidth',1.7);
    plot(z.X(1,:),z.X(2,:),'r-','LineWidth',1.4);
    axis equal; grid on; xlabel('x (m)'); ylabel('y (m)');
end

function write_text(path,content)
    fid=fopen(path,'w','n','UTF-8'); assert(fid>=0);
    cleanup=onCleanup(@() fclose(fid)); %#ok<NASGU>
    fprintf(fid,'%s',content);
end
