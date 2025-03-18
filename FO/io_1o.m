% 定义微分方程 dy/dt = -k * y% 其中 y 是未知函数，k 是常数
% 定义常数
k = 0.1;
% 定义初始条件
y0 = 1;
% 定义仿真时间范围
tspan = [0 10]; % 从0到10
% 调用 ode45 函数求解微分方程
[t, y] = ode45(@(t,y) -k*y, tspan, y0);

% 绘图
plot(t, y);
xlabel('Time');
ylabel('y(t)');
title('Solution of the ODE dy/dt = -k*y');
grid on;
