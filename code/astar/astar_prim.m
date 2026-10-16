clear; clc; close all;


% 1. 创建地图；每米4个格子，即每格0.25米
map = binaryOccupancyMap(30, 30, 4);


%% ====障碍物1
% [X, Y] = meshgrid(20:0.125:28, 10:0.125:18);
% % 设置矩形障碍物，左下角坐标为(12, 8)，右上角坐标为(18, 20)
% % 对应的起点为：[12,48]，终点为：[108,60]
% [X, Y] = meshgrid(12:0.125:18, 8:0.125:20);
% setOccupancy(map, [X(:), Y(:)], 1); % 设置障碍物

%% ====障碍物2
% 原始正方形：[15,23]×[10,18]，绕世界原点逆时针旋转9°
% 对应的起点为：[4,60]，终点为：[108,60]
% [X, Y] = meshgrid(15:0.125:23, 10:0.125:18);
% theta = pi/20; % 旋转角度
% R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
% points = [X(:), Y(:)] * R.';
% setOccupancy(map, points, 1); % 设置障碍物

%% ====障碍物3：圆形障碍物
[X, Y] = meshgrid(0:0.125:120, 0:0.125:120);
% 设置圆形障碍物，圆心为(15, 14)，半径为3米
radius = 3;
mask = (X - 15).^2 + (Y - 14).^2 <= radius^2;
setOccupancy(map, [X(mask), Y(mask)], 1); % 设置障碍物


% 2. 膨胀障碍物：示例安全半径为0.5米
occupiedOriginal = occupancyMatrix(map) > 0.5;
inflationRadius = 0.5; % 膨胀距离，单位m；同时用于图中说明
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
startRC = [4, 60];
goalRC  = [108, 60];

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

% 7. 显示搜索结果
figure('Name', 'A*搜索结果（栅格坐标）', 'Color', 'w');
show(planner);
title('A*粗路径搜索（膨胀后的地图）', 'Interpreter', 'none');
xlabel('列索引');
ylabel('行索引');

% 输出粗路径长度
pathLength = sum(vecnorm(diff(pathXY, 1, 1), 2, 2));
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
