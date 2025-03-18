function y=adapt_fun(x)

y=sin(10*pi*x)/x;%适应度函数本质就是期望求最值的那个性能指标
% 变成多维需要对多个自变量同时求