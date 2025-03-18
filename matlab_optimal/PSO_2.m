%思想利用群体智能(多个数据)来避免求导，采用迭代收敛的方法进行求最优值
%在可行解中初始化一群粒子，每个粒子代表可能的最优解(用位置、速度、适应度三项指标表示其特征)
% 粒子在解空间中运动，跟踪个体极值Pbest和群体极值Gbest更新个体位置(适应度反应最优性)
% 伪代码：
%初始化粒子的位置和速度
%适应度函数值计算
%个体极值和群体极值计算
%while:
% 速度、位置更新->适应度函数值计算->个体极值和群体极值计算->判断
% 最优解
%%
clc
clear all
%%
x,y=meshgrid(-5:0.1:5,-5:0.1:5)
z=x.^2+y.^2-10*cos(2*pi*x)-10*cos(2*pi*y)+20
figure
mesh(x,y,z)
hold on
%%  参数初始化
c1=1.49445;
c2=1.49445;
episode=50;%迭代次数
sizepop=10;%粒子规模，10个例子
Vmax=0.5;%速度限定
Vmin=-0.5;
popmax=2;%根据X范围定义1-2之间
popmin=1;
ws=0.9;
we=0.2;

%% 产生初始化位置和速度
for i=1:sizepop
    %随机产生一个种群
    pop(i,:)=(rands(1)+1)/2+1;%rands是-1到1，变换1到2 初始化位置.位置可能是多维的，不一定一维
    v(i,:)=0.5*rands(1);%初始化速度,速度可能是多维的，不一定一维
    %计算适应度函数
    fitness(i)=adapt_fun(pop(i,:));

end
%%  个体极值和群体极值
[bestfitness bestindex]=max(fitness);
gbest=pop(bestindex,:)%全体最佳,pop是一个数组，gbest是一个值,位置
pbest=pop;%个体最佳,数组,位置
fitnesspbest=fitness%数组,适应度值
fitnessgbest=bestfitness%数组,适应度值

%% 迭代寻优
for i=1:episode
    %w速度更新上一时刻值的权重，进行线性递减惯性
    w=ws-(ws-we)*(i/episode)
    for j=1:sizepop
        %速度更新,唯一的迭代公式,其实也是靠反馈控制来收敛
        v(j,:)=w*v(j,:)+c1*rand*(pbest(j,:)-pop(j,:))+c2*rand*(gbest-pop(j,:));%原来的速度、(该粒子的个体位置-个体最优)、(该粒子的个体位置-全局最优)
        %边界限幅约束 
        v(j,find(v(j,:)>Vmax))=Vmax;%对其分量进行限制
        v(j,find(v(j,:)<Vmin))=Vmin;%对其分量进行限制
        %种群(位置)更新
        pop(j,:)=pop(j,:)+v(j,:);
        pop(j,find(pop(j,:)>popmax))=popmax;
        pop(j,find(pop(j,:)<popmin))=popmin;
        %适应度更新(全部粒子更新)
        fitness(j)=adapt_fun(pop(j,:));
    end
    for j=1:sizepop
        %个体最优
        if fitness(j)>fitnesspbest(j)%更新后的适应度比之间好，则个体最优更新
            pbest(j,:)=pop(j,:);%个体的位置更新
            fitnesspbest(j)=fitness(j);%个体的适应度更新
        end
        %群体最优更新
        if fitness(j)>fitnessgbest%更新后的适应度比之间好，则个体最优更新
            gbest=pop(j,:);%个体的位置更新
            fitnessgbest=fitness(j);%个体的适应度更新
        end
    end
    yy(i)=fitnessgbest;
end

%%
[fitnessgbest gbest]%输出结果
plot(gbest,fitnessgbest,'r*')

figure
plot(yy)
title("最优个体适应度",'FontSize',12);
xlabel('进化代数','FontSize',12);
ylabel('适应度','FontSize',12);



















