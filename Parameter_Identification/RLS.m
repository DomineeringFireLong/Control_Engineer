clc; clear

% 构建三个输入项，一共101组
t = 0 : 1 : 100;
x1 = t;
x2 = (t - 60).^2;
x3 = cos(pi.*t / 10);

% 假设系统真实系数
c1 = 2.22;
c2 = 0.05;
c3 = 23.1;

% 得到系统输出，得到对应的101个输出（人为加入了一些噪声）
Y = c1 * x1 + c2 * x2 + c3 * x3  + randn(1,101);%系统方程，根据输入和输出来拟合其参数

% 绘制一下采样数据
plot(t, Y,'o')
hold on

% 设置初始值，执行递推过程
theta = [0;0;0];
Pk_ = 1e6 * eye(3);%初始值可I为单位矩
lambda = 0.995;
for i = 1 : 1 : 100
    x = [x1(i); x2(i); x3(i)];%系统输入
    y = Y(i);%系统输出
    Kk = Pk_ * x / (lambda + x' * Pk_ * x);
    theta = theta + Kk * (y - x'*theta);
    Pk_ = (1/lambda)*(eye(3) - Kk * x') * Pk_;
end

% 基于最小二乘的结果绘制曲线，并于原始数据做对比
Y_Est = theta(1) * x1 + theta(2) * x2 + theta(3) * x3;

% 绘制出来拟合的曲线，并于采样点进行比较
plot(t, Y,'r')
