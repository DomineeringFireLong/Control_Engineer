clear ; 
close all; 
clc;
%此处以控制到原点为例，故e=y-0=x,对误差的反馈矫正体现在在性能指标函数中
%第一步，定义状态空间矩阵
%定义状态矩阵 A, n x n 矩阵
A = [1 0.1; -1 2];
n= size (A,1);%仅查询 A 的第1个维度的长度 矩阵1行2列
%定义输入矩阵 B, n x p 矩阵
B = [ 0.2 1; 0.5 2];
p = size(B,2);
% 定义Q矩阵，n x n 矩阵
Q=[100 0;0 1];
% 定义F矩阵，n x n 矩阵
F=[100 0;0 1];
% 定义R矩阵，p x p 矩阵
R=[1 0 ;0 0.1];%J=eqe+uru+e(k_steps)Fe(k_steps)
% 定义step数量k
k_steps=100; 
% 定义矩阵 X_K， n x k 矩 阵
X_K = zeros(n,k_steps);
% 初始状态变量值， n x 1 向量
X_K(:,1) =[20;-20];
% 定义输入矩阵 U_K， p x k 矩阵
U_K=zeros(p,k_steps);
% 定义预测区间K
N=5;
% Call MPC_Matrices 函数 求得 E,H矩阵 

%其实就是求出k步长的转移矩阵，然后带入求出E H两个矩阵，把性能指标只与 初始状态和控制量有关）
%对性能指标进行求解简化
[E,H]=MPC_Matrices(A,B,Q,R,F,N);%模型和性能指标参数以及预测步长


% 计算每一步的状态变量的值，用二次规划，优化每次的性能指标，求出每一步的控制量，只取第一个控制量，不断在线优化求解
for k = 1 : k_steps 
% 求得U_K(:,k)
U_K(:,k) = Prediction(X_K(:,k),E,H,N,p);
% 计算第k+1步时状态变量的值
X_K(:,k+1)=(A*X_K(:,k)+B*U_K(:,k));
end
% 绘制状态变量和输入的变化
subplot  (2, 1, 1);
hold;
for i =1 :size (X_K,1)
plot (X_K(i,:));
end
legend("x1","x2")
hold off;
subplot (2, 1, 2);
hold;
for i =1 : size (U_K,1)
plot (U_K(i,:));
end
legend("u1","u2")
