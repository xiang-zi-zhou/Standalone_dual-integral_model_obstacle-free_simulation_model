function astar_check(astar)

    pathXY = astar.pathXY;
    exploredXY = astar.exploredXY;
    searchTime = astar.searchTime;
    pathLength = astar.pathLength;
    debugInfo = astar.debugInfo;
    map = astar.map;
    inflationOnly = astar.obs.inflationOnly;
    inflationRadius = astar.obs.inflationRadius;

    % 输出粗路径长度
    fprintf('粗路径长度：%.2f m\n', pathLength);
    fprintf('A*搜索耗时：%.6f s（%.3f ms）\n', searchTime, searchTime*1000);
    fprintf('A*探索节点数：%d\n', debugInfo.NumNodesExplored);

    figure('Name', 'A*粗路径规划（世界坐标）', 'Color', 'w', ...
        'Position', [100, 100, 1000, 780]);
    show(map);
    hold on;
    % 膨胀新增区域：浅灰色
    d = 0.5/map.Resolution; % 每个栅格的半边长，单位m
    [row, col] = find(inflationOnly);
    xy = grid2world(map, [row, col]);
    px0 = xy(:, 1);
    py0 = xy(:, 2);
    hInflation = patch( ...
        [px0-d, px0+d, px0+d, px0-d].', ...
        [py0-d, py0-d, py0+d, py0+d].', ...
        [0.80, 0.80, 0.80], ...
        'EdgeColor', 'none', ...
        'DisplayName', '膨胀区域');
    % G矩阵中具有有限代价的节点；不将其全部称为已扩展节点
    px = exploredXY(:, 1);
    py = exploredXY(:, 2);
    patch([px-d, px+d, px+d, px-d].', [py-d, py-d, py+d, py+d].',...
        [1, 0.90, 0.65],'EdgeColor', 'none'); % A*探索过的路径点;.' 是非共轭转置

    % A*搜索路径
    hPath = plot(pathXY(:, 1), pathXY(:, 2), 'b-', 'LineWidth', 2); % A*搜索路径

    % 起点终点
    hStart = plot(pathXY(1, 1), pathXY(1, 2), 'go', 'MarkerSize', 7, 'MarkerFaceColor', 'g'); % 起点
    hGoal = plot(pathXY(end, 1), pathXY(end, 2), 'ro', 'MarkerSize', 7, 'MarkerFaceColor', 'r'); % 终点

    % 使用图例代理，避免每个栅格生成一个图例条目
    hOriginalKey = patch(NaN, NaN, [0, 0, 0], 'EdgeColor', 'none');
    hInflationKey = patch(NaN, NaN, [0.80, 0.80, 0.80], 'EdgeColor', 'none');
    hSearchKey = patch(NaN, NaN, [1, 0.90, 0.65], 'EdgeColor', 'none');
    legend([hOriginalKey, hInflationKey, hSearchKey, hPath, hStart, hGoal], ...
        {'原始障碍物', '膨胀新增区域', '搜索节点（有限G值）', ...
        'A*粗路径', '起点', '终点'}, ...
        'Location', 'eastoutside', 'FontSize', 11, 'AutoUpdate', 'off');

    title({'二维障碍环境下的A*粗路径规划', ...
        sprintf('路径长度 %.2f m | 搜索耗时 %.2f ms | 扩展节点数 %d', ...
        pathLength, searchTime*1000, debugInfo.NumNodesExplored)}, ...
        'Interpreter', 'none', 'FontSize', 13);
    xlabel('x / m', 'FontSize', 12);
    ylabel('y / m', 'FontSize', 12);
    text(pathXY(1, 1)+0.5, pathXY(1, 2), '起点', ...
        'FontSize', 11, 'VerticalAlignment', 'middle');
    text(pathXY(end, 1)+0.5, pathXY(end, 2), '终点', ...
        'FontSize', 11, 'VerticalAlignment', 'middle');

    annotation('textbox', [0.08, 0.01, 0.84, 0.06], ...
        'String', sprintf(['图：A*粗路径与搜索节点分布。栅格边长 %.2f m，障碍物膨胀距离 %.2f m。\n' ...
        '灰色表示新增安全区域；黄色表示G矩阵中记录有限代价的节点。'], ...
        1/map.Resolution, inflationRadius), ...
        'EdgeColor', 'none', 'HorizontalAlignment', 'center', ...
        'FontSize', 10, 'Interpreter', 'none');

    axis equal;
    xlim(map.XWorldLimits);
    ylim(map.YWorldLimits);
    set(gca, 'FontSize', 11, 'LineWidth', 1, 'GridAlpha', 0.12);
    box on;
    grid on;

end