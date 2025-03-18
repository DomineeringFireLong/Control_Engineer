% 定义初始条件和仿真时间范围
x0 = [10; 0]; % 初始状态
tspan = [0 10]; % 仿真时间范围

% 控制器设计
alpha = 0.8; % 分数阶
lambda = 5; % 控制增益
%x1'=x2;x2'=u;
% 求解微分方程s=x(2) + x(1)^a, s'=u+a*x1^(a-1)*x2=-lsign(s)
[t, x] = ode45(@(t,x) [x(2); -lambda*sign(x(2) + x(1)^0.5)-alpha*x(1)^(alpha-1)*x(2)], tspan, x0);

% 绘图
plot(t, x(:, 1));
xlabel('Time');
ylabel('x1(t)');
title('State Response');
grid on;
