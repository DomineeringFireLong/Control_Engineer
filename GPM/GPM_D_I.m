%思想：通过数值方法，插值拟合原来的函数，然后用插值函数的求导和数值积分来把连续、含微积分问题变成离散的代数问题
clear all
clc
close all
N = 10;                     % 不算t0时刻的配点个数，实际标号t0,t1,...,tN

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%--------------计算积分权重系数和微分估计矩阵------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%1.区间离散化
syms tau
PN = 1./(2^N*factorial(N))*diff((tau^2-1)^N,tau,N);         % Legendre 多项式
tau_collocpoints = [sort(double(solve(PN==0)))]';           % Legendre 多项式零点，也就是高斯伪谱法的配点
tau_interppoints = [-1 tau_collocpoints];                   % 拉格朗日多项式插值点
%2.积分权重系数
Astarsym = 2/((1-tau^2)*(diff(PN,tau,1))^2);                % 积分权重系数的表达式
Astar = double(subs(Astarsym,tau,tau_collocpoints));        % 配点上的积分权重系数值

% 计算微分矩阵
for k = 1:length(tau_collocpoints)
    for i = 1:length(tau_interppoints)
        for m = 1:length(tau_interppoints)
            if m == i
                summed(m) = 0;                                          % 被求和项，要求m≠i，可转化为m=i时，该项为0
            else
                fenzilist = tau_collocpoints(k) - tau_interppoints;     
                fenzilist([i,m]) = [];%把第i和m个去掉，不是置0
                fenzi = prod(fenzilist);                                % 被求和项的分子
                fenmulist = tau_interppoints(i) - tau_interppoints;
                fenmulist([i]) = [];
                fenmu = prod(fenmulist);                                % 被求和项的分母
                summed(m) = fenzi./fenmu;                               % 被求和项
            end
        end
        D(k,i) = sum(summed);                                           % 微分估计矩阵(k,i)的值
    end
end


%上述都是与模型无关的插值项

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%--------------积分权重系数和微分估计矩阵验证------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% 以函数f(x)=3x^3 + 2x^2 + x为例子进行验证,x定义在[-1,1]
syms x
f = 3*x^3 + 2*x^2 + x;

% 微分验证
f_diff_real = double(subs(diff(f,x,1),tau_collocpoints));

f_tau_interp = double(subs(f,x,tau_interppoints));
f_diff_appr = D*f_tau_interp';

figure(1)
plot(tau_collocpoints,f_diff_appr,'or');
hold on
plot(tau_collocpoints,f_diff_real,'-b');
xlabel('x');
ylabel('df/dx');
legend({'配点上的估计值','实际导数值'})
grid on

% 积分验证，计算[-1,1]的积分值
fun = @(x) 3*x.^3 + 2*x.^2 + x;
fint_real = integral(fun,-1,1);

f_tau_colloc = double(subs(f,x,tau_collocpoints));
fint_GL = sum(Astar.*f_tau_colloc);%积分权重*函数在插值点的函数值

disp('积分的真值为')
disp([num2str(fint_real),'']);
disp('高斯积分的值为')
disp([num2str(fint_GL),'']);