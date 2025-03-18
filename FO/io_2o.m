% 定义二阶微分方程的函数
% y(1) = y, y(2) = dy/dt;  y2''=-2y1-2y
%f匿名函数必须同时接受两个输入 (t,y),y是两维向量时[]内，y1';y2'
f = @(t, y) [y(2); -2*y(2) - 2*y(1)];
% 也可单独创建函数m文件
% function dydt = odefun(t,y)
% dydt = zeros(2,1);
% dydt(1) = y(1)+2*y(2);
% dydt(2) = 3*y(1)+2*y(2);
% end
% 定义初始条件
y0 = [1; 0]; % 初始值 y(0) = 1, dy/dt(0) = 0

% 定义仿真时间范围
tspan = [0 10];

% 调用 ode45 函数求解微分方程;ode45 仅适用于使用两个输入参数（t 和 y）的函数
[t, y] = ode45(f, tspan, y0);

% 绘图
plot(t, y(:, 1)); % 绘制 y(t)
xlabel('Time');
ylabel('y(t)');
title('Solution of the Second Order ODE');
grid on;
