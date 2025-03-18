function dy = glfdiff(y,t,gam)
 
if strcmp(class(y),'function_handel')
    y = y(t);
end
 
h = t(2)-t(1);
w = 1;
y = y(:);
t = t(:);
 
for j = 2:length(t)
    w(j) = w(j-1)*(1-(gam+1)/(j-1));
end
for i = 1:length(t)
    dy(i) = w(1:i)*[y(i:-1:1)]/h^gam;
end



% 
% function dy = glfdiff(f,t,alpha)
%  
% if strcmp(class(f),'function_handel')
%     f = f(t);
% end
%  
% h = t(2)-t(1);
% n=length(t);
% J=[0:(n-1)]
% f = f(:);
% a0=f(1);
% 
% if a0~=0 && alpha>0
%     dy(1)=sign(a0)*Inf;
% end
% 
% w=gamma(alpha+1)*(-1).^J./gamma(J+1)./gamma(alpha-J+1)/h^alpha;
% % for j = 2:n
% %     w(j) = w(j-1)*(1-(gam+1)/(j-1));
% % end
% for i = 2:length(t)
%     dy(i) = w(1:i)*[f(i:-1:1)];
% end
% %通过3.2-8 求极限公式，精度不够
% % 这样的算法有明显的缺陷，因为其中直接涉及大数的Gamma函数值，由于精度影响，计算结果出现错误。