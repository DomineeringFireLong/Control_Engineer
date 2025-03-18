% 定义系统参数
n = 100;  % 数据点数量
a1_true = 0.5;  % 真实值
a2_true = 0.3;  % 真实值
b1_true = 1.0;  % 真实值
b2_true = 0.8;  % 真实值

% 生成输入信号 (单位脉冲输入)
u = [1; zeros(n-1, 1)];

% 生成系统输出数据（带噪声）
y = zeros(n, 1);
noise = 0.1 * randn(n, 1);  % 添加噪声

for k = 3:n
    y(k) = a1_true * y(k-1) + a2_true * y(k-2) + b1_true * u(k-1) + b2_true * u(k-2) + noise(k);
end

% 卡尔曼滤波器的参数初始化
P = eye(4);  % 误差协方差矩阵 (4x4，因为我们有4个参数)
Q = 1e-5 * eye(2);  % 过程噪声协方差矩阵
R = 0.1;  % 观测噪声协方差
theta_hat = zeros(4, 1);  % 参数估计值

% 记录估计值
theta_estimates = zeros(n, 4);

% 卡尔曼滤波循环
for k = 3:n
    % 构建观测向量和矩阵
    H = [y(k-1); y(k-2);u(k-1); u(k-2)]';  % 观测矩阵 H 需要是列向量
    z = [y(k); u(k-1); u(k-2); y(k-1)];  % 观测向量
    
    % 预测步骤
    theta_hat_pred = theta_hat;  % 参数预测
    P_pred = P + Q;  % 协方差预测
    
    % 计算卡尔曼增益
    K = P_pred * H / (H' * P_pred * H + R);  % 卡尔曼增益
    
    % 更新步骤
    theta_hat = theta_hat_pred + K * (z - H' * theta_hat_pred);  % 更新参数估计
    P = (eye(2) - K * H') * P_pred;  % 更新误差协方差矩阵
    
    % 保存估计值
    theta_estimates(k, :) = theta_hat';
end

% 绘制结果
figure;
subplot(2,1,1);
plot(theta_estimates(:,1), 'r', 'DisplayName', 'Estimated a1');
hold on;
plot(a1_true * ones(n, 1), 'k--', 'DisplayName', 'True a1');
legend;
title('Parameter Estimation for a1');

subplot(2,1,2);
plot(theta_estimates(:,2), 'r', 'DisplayName', 'Estimated a2');
hold on;
plot(a2_true * ones(n, 1), 'k--', 'DisplayName', 'True a2');
legend;
title('Parameter Estimation for a2');
