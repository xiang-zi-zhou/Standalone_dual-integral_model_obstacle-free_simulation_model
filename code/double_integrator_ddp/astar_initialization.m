function [astar] = astar_initialization(cfg)
% 接收任务信息，传出pathXY

    % 接收任务信息
    x0 = cfg.task.x0;
    x_goal = cfg.task.x_goal;

    % a*初始化世界
    x_world = cfg.world.x;
    y_world = cfg.world.y;
    resolution = cfg.world.resolution;

    map = binaryOccupancyMap(x_world, y_world, resolution);
    % 生成障碍物(圆形)  
    % 这里之后可能需要优化，目前只能识别单个圆形障碍物？
    obstacle3 = cfg.task.obs; % 圆形障碍物，圆心为(15, 14)，半径为3米
    [X, Y] = meshgrid(0:1/resolution:x_world, 0:1/resolution:y_world);
    radius = obstacle3(3);
    mask = (X - obstacle3(1)).^2 + (Y - obstacle3(2)).^2 <= radius^2;
    setOccupancy(map, [X(mask), Y(mask)], 1); % 设置障碍物

    % 膨胀障碍物：示例安全半径为0.5米
    occupiedOriginal = occupancyMatrix(map) > 0.5;
    inflationRadius = cfg.task.obs_safe; % 与 DDP 使用相同的安全距离
    inflate(map, inflationRadius);
    occupiedInflated = occupancyMatrix(map) > 0.5;
    % 仅膨胀新增的区域
    inflationOnly = occupiedInflated & ~occupiedOriginal;

    % 3. 创建 A* 规划器
    planner = plannerAStarGrid(map);
    planner.GCost = 'Euclidean';  % g代表从起点到当前节点已经走过的实际代价
    % 'Euclidean'（默认）：欧几里得距离
    planner.HCost = 'Euclidean';

    % 4. 起点、终点采用栅格坐标：[行, 列]
    startRC = world2grid(map, x0(1:2)');
    goalRC = world2grid(map, x_goal(1:2)');

    % 计时
    tStart = tic;

    % 5. 搜索路径
    [pathRC, debugInfo] = plan(planner, startRC, goalRC);
    searchTime = toc(tStart);

    if isempty(pathRC)
        error('没有找到可行路径，请检查地图和起终点。');
    end

    % 6. 转换成世界坐标：[x, y]，单位米
    pathXY = grid2world(map, pathRC);

    G = debugInfo.GCostMatrix; % 获取A*探索过的路径点
    explored = isfinite(G) & G > 0; % 获取探索过的路径点
    explored(startRC(1), startRC(2)) = true; % 起点代价为0，单独加入
    [row, col] = find(explored);
    exploredXY = grid2world(map, [row, col]); % 转换成世界坐标：[x, y]，单位米

    % 输出粗路径长度
    pathLength = sum(vecnorm(diff(pathXY, 1, 1), 2, 2));

    % 汇总搜索结果
    astar.pathXY = pathXY;
    astar.exploredXY = exploredXY; % 探索过的路径方格中点坐标
    astar.searchTime = searchTime;
    astar.pathLength = pathLength;
    astar.debugInfo = debugInfo;
    % 障碍物（用于画图）
    astar.map = map;
    astar.obs.opt = obstacle3; % 圆形障碍物，圆心为(15, 14)，半径为3米
    astar.obs.inflationOnly = inflationOnly; % 膨胀新增的障碍物区域
    astar.obs.inflationRadius = inflationRadius; % 膨胀半径
end
