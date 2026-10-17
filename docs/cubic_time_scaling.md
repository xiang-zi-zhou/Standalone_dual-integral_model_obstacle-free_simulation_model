# A* 粗路径的三次时间分配

## 1. 目的与符号

A* 给出按起点到终点排列的位置点 `pathXY`（M×2，单位 m），没有给出到达各点的时间。时间分配用于生成带时间的位置参考，并不改变折线路径的几何形状。

- $L$：折线总长，单位 m。
- $T=N\Delta t$：总运动时间，单位 s。
- $s(t)$：时刻 t 沿路径已走过的弧长，单位 m。
- $\tau=t/T$：归一化时间。
- $q(\tau)=s(t)/L$：归一化路径进度。

本方法用于静止出发、静止到达的参考进度，不要求起终点加速度为零。

## 2. 三次多项式的推导

要求满足四个边界条件：

$$s(0)=0,\quad s(T)=L,\quad \dot s(0)=0,\quad \dot s(T)=0.$$

因为 $\dot s=(L/T)q'(\tau)$，等价条件为：

$$q(0)=0,\quad q(1)=1,\quad q'(0)=0,\quad q'(1)=0.$$

选择含四个待定系数的三次多项式：

$$q(\tau)=a_0+a_1\tau+a_2\tau^2+a_3\tau^3.$$

由起点条件得到 $a_0=a_1=0$；由终点条件得到：

$$a_2+a_3=1,\qquad 2a_2+3a_3=0.$$

解得 $a_2=3,\ a_3=-2$，所以：

$$\boxed{s(t)=L(3\tau^2-2\tau^3),\qquad \tau=t/T.}$$

它表示先慢后快再慢地走完整条路径，并不是对路径坐标直接做三次曲线拟合。

## 3. 速度、加速度与时间选择

$$\dot s(t)=\frac{6L}{T}\tau(1-\tau),\qquad
\ddot s(t)=\frac{6L}{T^2}(1-2\tau).$$

在 $0\le\tau\le1$ 内，$\dot s\ge0$，路径进度不会倒退。

$$\dot s_{\max}=\frac{3L}{2T},\qquad
\max|\ddot s|=\frac{6L}{T^2}.$$

起终点加速度分别为 $6L/T^2$ 和 $-6L/T^2$，并不为零。如果运动前后保持静止，加速度在连接处发生跳变；这是三次时间分配的特点。

仅从标量进度的速度、加速度限制出发，可作初步检查：

$$T\ge\max\left(\frac{1.5L}{v_{\max}},\sqrt{\frac{6L}{a_{\max}}}\right).$$

该式不保证折线路径的空间加速度满足限制。平滑曲线路径 $\mathbf p(s)$ 的实际加速度为：

$$\ddot{\mathbf p}=\mathbf p''(s)\dot s^2+\mathbf p'(s)\ddot s.$$

第一项包含转弯影响。A* 折点处切向不连续，全局三次时间分配不会自动让运动在折点停下，也不会消除方向突变。

## 4. MATLAB：生成带时间的位置参考

下面代码放在 A* 已经生成 `pathXY` 之后。`cfg` 应已通过 `make_config()` 创建，且 `N`、`dt` 与 DDP 使用值一致。

```matlab
% pathXY: M×2，米制坐标，按起点到终点排列
assert(size(pathXY,2) == 2 && ~isempty(pathXY));
assert(all(isfinite(pathXY(:))));

% 删除连续重复点，避免 interp1 的弧长坐标重复
keep = [true; vecnorm(diff(pathXY,1,1),2,2) > 1e-10];
P = pathXY(keep,:);
assert(size(P,1) >= 2, '路径至少需要两个不同的点。');

% 累计弧长
sNodes = [0; cumsum(vecnorm(diff(P,1,1),2,2))];
L = sNodes(end);

% N 个控制区间对应 N+1 个状态时刻
N = cfg.model.N;
dt = cfg.model.dt;
assert(N >= 1 && N == floor(N) && dt > 0);
T = N*dt;
t = (0:N).'*dt;
tau = t/T;

% 三次时间分配
progress = 3*tau.^2 - 2*tau.^3;
sQuery = L*progress;
sQuery = min(L, max(0, sQuery)); % 防止浮点误差超出插值范围

% 沿原折线按弧长插值；输出尺寸为 2×(N+1)
Pref = interp1(sNodes, P, sQuery, 'linear').';

% 这些是标量路径进度的导数，不是完整的二维速度/加速度
sDot = (6*L/T)*tau.*(1-tau);
sDDot = (6*L/T^2)*(1-2*tau);

fprintf('路径长度：%.3f m，总时间：%.3f s\n', L, T);
fprintf('参考进度峰值速度：%.3f m/s\n', 1.5*L/T);
fprintf('参考进度峰值加速度绝对值：%.3f m/s^2\n', 6*L/T^2);
```

线性插值在空间上沿原折线取点。但若一个时间步跨越多个折点，连接相邻参考采样点的直线可能偏离原折线；由控制前向仿真得到的轨迹也可能偏离原路径。因此仍需检查实际运动，包括采样间隔内的碰撞。

## 5. 如何接上控制初值与 DDP

时间分配的输出是 `Pref`，不是加速度控制 `U0`。后续继续使用主对话提出的动力学最小二乘拟合，求得 `2×N` 的 `U0`，再前向仿真得到 `X0`。

如果已经采用主对话中的 `path_to_u0` 示例，只需替换一行：

```matlab
% 原来的五次时间分配
% progress = 10*tau.^3 - 15*tau.^4 + 6*tau.^5;

% 改为三次时间分配
progress = 3*tau.^2 - 2*tau.^3;
```

其余弧长插值、控制拟合和 rollout 逻辑不变。调用结构仍为：

```matlab
cfg = make_config();
cfg.task.x0 = [pathXY(1,:).'; 0; 0];
cfg.task.x_goal = [pathXY(end,:).'; 0; 0];

% path_to_u0 是需要自行保存的自定义函数，不是 MATLAB 内置函数
[U0, X0, Pref] = path_to_u0(pathXY, cfg);
sol = solve_ddp(cfg.task.x0, U0, @dynamics, ...
    @stage_cost, @terminal_cost, @backward_pass, cfg);
```

参考进度起终点速度为零，不代表最小二乘拟合后的终端速度严格为零。如果要求严格静止到达，需要对控制拟合加入等式约束：

$$\Delta t\sum_{k=0}^{N-1}u_k=v_{\mathrm{goal}}-v_0.$$

同时检查初值轨迹的速度、加速度、终端误差和碰撞。时间分配本身不提供这些保证，也不替代正式 DDP 的避障代价或走廊约束。

## 6. 论文表述与参考

可表述为：

> 为将 A* 几何路径转换为带时间的位置参考，首先计算路径点的累计弧长，再采用满足起终点进度速度为零的三次多项式时间标度，按统一时间步长生成参考位置序列。随后结合离散双积分模型拟合初始控制序列，并检验所得初始轨迹的运动约束及避障情况。

参考：Kevin M. Lynch and Frank C. Park, *Modern Robotics: Mechanics, Planning, and Control*, Cambridge University Press, 2017, Chapter 9（Trajectory Generation）。

- [教材第 9 章资源](https://modernrobotics.northwestern.edu/chapters/chapter9/)
- [教材配套 CubicTimeScaling MATLAB 实现](https://github.com/NxRLab/ModernRobotics/blob/master/packages/MATLAB/mr/CubicTimeScaling.m)

本文提供推导和实现说明；未修改现有 A* 或 DDP 源文件，示例代码未在用户 MATLAB 环境中运行验证。
