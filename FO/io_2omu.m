A = [0 1; -1 -1];
B = [0; 1];
% 定义控制输入;向量m维，就是矩阵多维
u = @(t) sin(t); % 控制输入
% 定义一组一阶微分方程
f = @(t, y) [y(1);  y(2) - u(t)];%多维2*1

% 定义初始条件
y0 = [0; 0]; % 初始状态和初始速度

% 定义仿真时间范围
tspan = [0 10];

% 调用 ode45 函数求解微分方程
[t, y] = ode45(f, tspan, y0);

% 绘图
plot(t, y(:, 1)); % 绘制状态变量 x1(t)
hold on;
plot(t, y(:, 2)); % 绘制状态变量 x2(t)
xlabel('Time');
ylabel('State Variables');
title('Solution of the Second Order System with Control Input');
legend('x1(t)', 'x2(t)');
grid on;
