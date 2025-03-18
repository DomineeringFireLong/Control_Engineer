function u_k= Prediction(x_k,E,H,N,p)%滚动优化，控制问题转化成优化（二次规划）问题并求解

U_k = zeros(N*p,1); % NP x 1
%把代价函数转化成只跟 初始条件 和 输入量有关的函数
U_k = quadprog(H,E*x_k);%二次规划 min 0.5*u'Hu+(E*x_k)'u

u_k = U_k(1:p,1); % 取第一个结果

end 