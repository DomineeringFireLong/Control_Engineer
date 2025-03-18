%模拟退火算法求解TSP问题完整源代码
clear,clc;
close all;
% 读取个各城市的坐标
X=importdata('ts.xlsx');
data=X(:,2:3);   %各城市坐标
num_city = size(data,1);
Initial_temp = 1000;
res = 1e-3;       % 最低温限制
ratio = 0.9;       % 降温参数，控制温度的下降
temperature = Initial_temp;
Markov_length = 3000;                % 改变解的次数
Distance_matrix = pdist2(data,data); % 矩阵的形式存储城市之间的距离
route_new = randperm(num_city);      % 新产生的解路线
Energy_current = inf;                % 当前解的能量
Energy_best = inf;                   % 最优解的能量

route_current = route_new;           % 当前解路线
route_best = route_new;              % 最优解路线
pic_num = 1;

% 外层while循环控制降温过程，内层for循环控制新解的产生。
while temperature > res
    Energy1=Energy_best;                             % 用于控制循环的结束条件
    for i = 1: Markov_length
        
        % 产生新解(对当前解添加扰动)
        if rand >0.5
            % 两点交换
            a = 0;
            b = 0;
            while (a==b)
                a = ceil(rand*num_city);
                b = ceil(rand*num_city);
            end
            temp = route_new(a);
            route_new(a) = route_new(b);
            route_new(b) = temp;
        else
            % 三变化
            factor = randperm(num_city,3);
            factor = sort(factor);      % 对三个元素排序，a<b<c
            a = factor(1);
            b = factor(2);
            c = factor(3);
            temp = route_new(a:b);
            route_new(a:a+c-b-1) = route_new(b+1:c);
            route_new(a+c-b:c) = temp;
        end
        
        Energy_new = 0;   %计算该条路线的总距离
        for j=1:num_city-1
            Energy_new = Energy_new + Distance_matrix(route_new(j),route_new(j+1));
        end
        % 回到起始点，加上首尾两个城市的距离
        Energy_new = Energy_new+Distance_matrix(route_new(1),route_new(num_city));
        
        
        % 按照Metroplis准则接收新解
        if Energy_new<Energy_current
            % 更新局部最优
            Energy_current = Energy_new;
            route_current = route_new;
            
            % 更新全局最优
            if Energy_new<Energy_best
                Energy_best=Energy_new;
                route_best = route_new;
            end
            
        else
            if rand<exp(-(Energy_new-Energy_current)/temperature)
                Energy_current = Energy_new;
                route_current = route_new;
            else
                route_new = route_current; % 否则路线不更新，保存更改之前的路线
            end
            
        end
        
    end
    
    %画图
    items = [route_new,route_new(1)];                % 从最后一个城市回到起始城市

    scatter(data(route_new,1),data(route_new,2))
    for i=1:length(route_new)
        text(data(route_new(i),1),data(route_new(i),2),num2str(items(i)))
    end
    
    hold on
    plot(data(items,1),data(items,2),'b-')
    text(1.5,3.5,['温度=',num2str(temperature)]);
    text(1.5,3.3,['总距离=',num2str(Energy_best)]);

    drawnow;
    F=getframe(gcf);
    I=frame2im(F);
    [I,map]=rgb2ind(I,256);
    hold off
    title('模拟退火算法求解TSP问题')
    xlabel('横坐标')
    ylabel('纵坐标')
    pic_num = pic_num + 1;
    if Energy1==Energy_best&&Energy_current==Energy_best
        break
    else
        temperature = temperature*ratio; %降温过程
    end
    
end

%Energy_best:最短路径；route_best:最短路径对应的最优距离（不唯一）
disp('最优解:');
R = [route_best, route_best(1)];
N = length(R);
p = num2str(R(1));
for i = 2: N
    p = [p, '->', num2str(R(i))];
end
disp(p)
disp(['总距离:',num2str(round(Energy_best,8))]);
