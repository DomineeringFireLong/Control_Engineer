%%
clc; clear;

% 生成一组模拟数据
x = linspace(-10, 10, 100);
y = 2*x.^2 - 3*sin(x) + 0.5*randn(size(x));

% 定义要拟合的函数形式，包括二次项和三角函数项
f = @(p,x) p(1)*x.^2 + p(2)*sin(p(3)*x) + p(4);

% 定义初始参数矩阵，其中包含二次项系数、三角函数的振幅、频率和常数项
p0 = [1, 1, 1, 1]; % 对应真实值为[2, -3, 1, 0]

% 使用最小二乘法拟合曲线，并返回最优参数和拟合误差
[p, resnorm] = lsqcurvefit(f, p0, x, y);

% 生成拟合曲线并绘制
y_fit = f(p, x);
plot(x, y, 'o', x, y_fit, '-')
legend('原始数据', '拟合曲线')
%%
clc; clear;

% Generate simulated data
x = linspace(0, 2*pi, 50)';
y = 2*sin(2*x) + 0.5*x.^2 + randn(size(x));

% Define error function
fun = @(c) sum((y - (c(1)*sin(c(2)*x) + c(3)*x.^2 + c(4))).^2);

% Find least-squares solution
c0 = [1 2 1 1]; % 对应真实值为[2 2 0.5 0]
c = fminsearch(fun, c0);

% Plot results
xfit = linspace(0, 2*pi, 100)';
yfit = c(1)*sin(c(2)*xfit) + c(3)*xfit.^2 + c(4);
plot(x, y, 'ko', xfit, yfit, 'b-');
legend('Data', 'Fit');
