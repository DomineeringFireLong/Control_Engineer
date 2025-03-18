clc;clear;close all;
tic;
%% 01.初始参数设置
%-------------------------------------------------------------------------%
%----------------------- 设置问题的求解边界 ------------------------------%
%-------------------------------------------------------------------------%
% 设置时间
t0 = 0;
tf = 2;
% 设置状态量初值
x10 = -2;
x20 = 1;
% 设置状态量边界条件
x1Min = -5;
x1Max = 5;
x2Min = -5;
x2Max = 5;
% 设置控制量边界条件
uMin  = -1.5;
uMax  = 1.5;

%% 02.边界条件设置
%-------------------------------------------------------------------------%
%------------------------ 将求解边界设置于问题中 -------------------------%
%-------------------------------------------------------------------------%
mesh(1).colpoints=10;
bounds.phase.initialtime.lower  = t0; 
bounds.phase.initialtime.upper  = t0;
bounds.phase.finaltime.lower    = tf-1; 
bounds.phase.finaltime.upper    = tf+1;
bounds.phase.initialstate.lower = [x10 x20]; 
bounds.phase.initialstate.upper = [x10 x20];
bounds.phase.state.lower        = [x1Min x2Min]; 
bounds.phase.state.upper        = [x1Max x2Max];
bounds.phase.finalstate.lower   = [0 0];
bounds.phase.finalstate.upper   = [0 0];%末端状态约束
bounds.phase.control.lower      = uMin; 
bounds.phase.control.upper      = uMax;
bounds.phase.integral.lower     = 0; 
bounds.phase.integral.upper     = 1000;

%% 03.结果猜测
%-------------------------------------------------------------------------%
%------------------------------- 初值猜想 --------------------------------%
%-------------------------------------------------------------------------%
guess.phase.time     = [t0; tf+1]; 
guess.phase.state    = [[x10; 0],[x20; 0]];
guess.phase.control  = [uMin;uMax ];
guess.phase.integral = 100;

%% 04.设置GPOPS求解器参数
%-------------------------------------------------------------------------%
%---------------------------- 设置求解器参数 -----------------------------%        
%-------------------------------------------------------------------------%
setup.name = 'Vehicle-Stopping-OCP';
setup.functions.continuous  = @vsopcContinuous;
setup.functions.endpoint   	= @vsopcEndpoint;
setup.bounds                = bounds;
setup.guess                 = guess;
setup.nlp.solver            = 'snopt';
setup.derivatives.supplier  = 'sparseCD';
setup.derivatives.derivativelevel = 'second';
setup.mesh.method           = 'hp1';
setup.mesh.tolerance        = 1e-6;
setup.mesh.maxiteration     = 45;
setup.mesh.colpointsmax     = 4;
setup.mesh.colpointsmin     = 10;
setup.mesh.phase.fraction   = 0.1*ones(1,10);
setup.mesh.phase.colpoints  = 4*ones(1,10);
setup.method = 'RPMintegration';

%% 05.求解
%-------------------------------------------------------------------------%
%----------------------- 使用 GPOPS2 求解最优控制问题 --------------------%
%-------------------------------------------------------------------------%
output = gpops2(setup);
solution = output.result.solution;
toc;

%% 06.画图
figure('Color',[1,1,1]);
plot(solution.phase.time(:,1),solution.phase.state(:,1),'-','LineWidth',1.5);hold on;
plot(solution.phase.time(:,1),solution.phase.state(:,2),'-.','LineWidth',1.5);
plot(solution.phase.time(:,1),solution.phase.control(:,1),'--','LineWidth',1.5);
axis([0 2 -2.5 2]);
xlabel('Time',...
       'FontWeight','bold');
ylabel('States',...
       'FontWeight','bold');
legend('Pos','Vel','Acc',...
       'LineWidth',1,...
       'EdgeColor',[1,1,1],...
       'Orientation','horizontal',...
       'Position',[0.5,0.93,0.40,0.055]);
set(gca,'FontName','Times New Roman',...
        'FontSize',15,...
        'LineWidth',1.3);
print -dpng Result.png

%% 函数模块部分
% ----------------------------------------------------------------------- %
% ------------------------- BEGIN: vsopcContinuous.m -------------------- %
% ----------------------------------------------------------------------- %
function phaseout = vsopcContinuous(input)
t  = input.phase.time;
x1  = input.phase.state(:,1);
x2  = input.phase.state(:,2);
u   = input.phase.control(:,1);

dx1 = x2;
dx2 = u;
%动力学模型
phaseout.dynamics = [dx1, dx2];
phaseout.integrand =0.5*u.^2;%积分型性能指标
end
% ----------------------------------------------------------------------- %
% -------------------------- END: vsopcContinuous.m --------------------- %
% ----------------------------------------------------------------------- %

% ----------------------------------------------------------------------- %
% -------------------------- BEGIN: vsopcEndpoint.m --------------------- %
% ----------------------------------------------------------------------- %
function output = vsopcEndpoint(input)

J  = input.phase.integral;
output.objective = 0.2*J+0.8*input.phase.finaltime;%积分型性能指标+末端型
end
% ----------------------------------------------------------------------- %
% --------------------------- END: vsopcEndpoint.m ---------------------- %
% ----------------------------------------------------------------------- %
