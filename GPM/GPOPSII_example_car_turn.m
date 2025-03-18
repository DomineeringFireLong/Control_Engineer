%% 01.初始参数设置
clc;clear;close all;
tic;
 
%-------------------------------------------------------------------------%
%-----------------------根据问题，进行状态量边界条件约束的求解边界 -----------%
%-------------------------------------------------------------------------%
% 设置时间
t0 = 0;
tmax = 30;
% 设置状态量初值
 
x11_0 =0;x12_0 =5;%x13_0=;      xij_0表示状态变量xi在第j阶段的初值，若某阶段初值被注释，则说明该阶段没有初值不确定，根据优化目标自动选取，只给定范围
x11_f=5;%x12_f=;x13_f=;         xij_f表示状态变量xi在第j阶段的终值
x21_0 =0.2;x23_0 =10;%x22_0 =;
x22_f =10; x23_f =15;%x21_f =;
x30 =0;x3f= pi/2;
 
% 设置状态量边界条件
x11_Min = 0;x12_Min = 5;x13_Min = 14;
x11_Max = 5;x12_Max = 16;x13_Max = 16;
 
x21_Min = -1;x22_Min = -1;x23_Min = 10;
x21_Max = 1;x22_Max = 10;x23_Max = 15;
 
x3Min = -pi/2;
x3Max = pi/2;
%设置road约束宽度，即与目标路径中心相距的距离限制，本文设计道路宽两米，所以限幅（-1，1）
c10 = 0;
c1Min = -1;
c1Max = 1;
% 设置控制量边界条件
 
 
 
u1Min  = -pi*12/180;  %车辆前轮转向角限幅 rad 此处选择正负12°是与本例子弯道设计有关，
%该弯道不够急，若转向角限幅过大，车辆转向能力很足，可以直接贴着道路内边缘转向，可能得不到期望的轨迹。
u1Max  = pi*12/180;
u2Min  = 0.001;%车辆速度限幅 m/s
u2Max  = 20;
%% 02.边界条件设置
%-------------------------------------------------------------------------%
%------------------------ 将求解边界设置于问题中 -------------------------%
%-------------------------------------------------------------------------%
iphase=1;
bounds.phase(iphase).initialtime.lower  = t0; 
bounds.phase(iphase).initialtime.upper  = t0;
bounds.phase(iphase).finaltime.lower    = t0; 
bounds.phase(iphase).finaltime.upper    = tmax;
bounds.phase(iphase).initialstate.lower = [x11_0 x21_0 x30]; 
bounds.phase(iphase).initialstate.upper = [x11_0 x21_0 x30];
bounds.phase(iphase).state.lower        = [x11_Min x21_Min x3Min]; 
bounds.phase(iphase).state.upper        = [x11_Max x21_Max x3Max];
bounds.phase(iphase).finalstate.lower   = [x11_f x21_Min x3Min];
bounds.phase(iphase).finalstate.upper   = [x11_f x21_Max x3Max];
bounds.phase(iphase).control.lower      = [u1Min u2Min]; 
bounds.phase(iphase).control.upper      = [u1Max u2Max];
bounds.phase(iphase).path.lower        = c1Min;
bounds.phase(iphase).path.upper        = c1Max;
 
guess.phase(iphase).time     = [t0; tmax]; 
guess.phase(iphase).state    = [[x11_0; x11_Max],[x21_Min; x21_Max],[x3Min; x3Max]];
guess.phase(iphase).control  = [[u1Min; u1Max],[u2Min; u2Max]];
 
 
iphase=2;
bounds.phase(iphase).initialtime.lower  = t0; 
bounds.phase(iphase).initialtime.upper  = tmax;
bounds.phase(iphase).finaltime.lower    = t0; 
bounds.phase(iphase).finaltime.upper    = tmax;
bounds.phase(iphase).initialstate.lower = [x12_0 x22_Min x3Min]; 
bounds.phase(iphase).initialstate.upper = [x12_0 x22_Max x3Max];
bounds.phase(iphase).state.lower        = [x12_Min x22_Min x3Min]; 
bounds.phase(iphase).state.upper        = [x12_Max x22_Max x3Max];
bounds.phase(iphase).finalstate.lower   = [x12_Min x22_f x3Min];
bounds.phase(iphase).finalstate.upper   = [x12_Max x22_f x3Max];
bounds.phase(iphase).control.lower      = [u1Min u2Min]; 
bounds.phase(iphase).control.upper      = [u1Max u2Max];
bounds.phase(iphase).path.lower        = c1Min;
bounds.phase(iphase).path.upper        = c1Max;
 
guess.phase(iphase).time     = [t0; tmax]; 
guess.phase(iphase).state    = [[x12_0; x12_Max],[x21_Min; x22_f],[x3Min; x3Max]];
guess.phase(iphase).control  = [[u1Min; u1Max],[u2Min; u2Max]];
 
iphase=3;
bounds.phase(iphase).initialtime.lower  = t0; 
bounds.phase(iphase).initialtime.upper  = tmax;
bounds.phase(iphase).finaltime.lower    = t0; 
bounds.phase(iphase).finaltime.upper    = tmax;
bounds.phase(iphase).initialstate.lower = [x13_Min x23_0 x3Min]; 
bounds.phase(iphase).initialstate.upper = [x13_Max x23_0 x3Max];
bounds.phase(iphase).state.lower        = [x13_Min x23_Min x3Min]; 
bounds.phase(iphase).state.upper        = [x13_Max x23_Max x3Max];
bounds.phase(iphase).finalstate.lower   = [x13_Min x23_f x3f];
bounds.phase(iphase).finalstate.upper   = [x13_Max x23_f x3f];
bounds.phase(iphase).control.lower      = [u1Min u2Min]; 
bounds.phase(iphase).control.upper      = [u1Max u2Max];
bounds.phase(iphase).path.lower        = c1Min;
bounds.phase(iphase).path.upper        = c1Max;
 
guess.phase(iphase).time     = [t0; tmax]; 
guess.phase(iphase).state    = [[x13_Min; x13_Max],[x23_0; x23_f],[x3Min; x3Max]];
guess.phase(iphase).control  = [[u1Min; u1Max],[u2Min; u2Max]];

%% 03.连接点结构体设置
%-------------------------------------------------------------------------%
%------------- Set up Event Constraints That Link Phases -----------------%
%-------------------------------------------------------------------------%
bounds.eventgroup(1).lower = [zeros(1,3),0];
bounds.eventgroup(1).upper = [zeros(1,3),0];
bounds.eventgroup(2).lower = [zeros(1,3),0];
bounds.eventgroup(2).upper = [zeros(1,3),0];
%eventgroup用与后续存储每段之间连接点的差值，
%每有n段路径，就有n-1个eventgroup。结构里的zeros对于状态量数量，若有n个状态量，写为zeros(1,n)
 
 
for i=1:3    %此处1：3对应三段，即有n段 此处i=1:n
    meshphase(i).colpoints = 4*ones(1,10);
    meshphase(i).fraction = 0.1*ones(1,10);
end
 
%% 04.求解器参数设置
%-------------------------------------------------------------------------%
%---------------------------- 设置求解器参数 -----------------------------%        
%-------------------------------------------------------------------------%
setup.name = 'Vehicle-Conering';
setup.functions.continuous  = @vsopcContinuous;
setup.functions.endpoint   	= @vsopcEndpoint;
setup.bounds                = bounds;
setup.guess                 = guess;
setup.nlp.solver            = 'snopt';
setup.derivatives.supplier  = 'sparseCD';
setup.derivatives.derivativelevel = 'second';
setup.mesh.method           = 'hp1';
setup.mesh.tolerance        = 1e-6;
setup.mesh.maxiteration     = 10;
setup.mesh.colpointsmax     = 4;
setup.mesh.colpointsmin     = 10;
setup.mesh.phase = meshphase;
setup.method = 'RPMintegration';
 
%% 05.求解
%-------------------------------------------------------------------------%
%----------------------- 使用 GPOPS2 求解最优控制问题 --------------------%
%-------------------------------------------------------------------------%
output = gpops2(setup);
solution = output.result.solution;
toc;
 
%% 06.画图
%%绘制道路边缘线(红)以及实际轨迹(绿)
figure('Color',[1,1,1]);
iphase=1;
plot(solution.phase(iphase).state(:,1),solution.phase(iphase).state(:,2),'g');hold on;
 
  x_in0=linspace(0,5,10);
  y_in0=x_in0-x_in0+1;
  x_in1=linspace(5,14,100);
  y_in1=-sqrt(81-(x_in1-5).^2)+10;
  x_in=[x_in0 x_in1];
  y_in=[y_in0 y_in1];
  line([14 14],[10 15],'Color','r');hold on;
  plot(x_in, y_in,'r-'); hold on;
  
 
iphase=2;
plot(solution.phase(iphase).state(:,1),solution.phase(iphase).state(:,2),'g');
 
iphase=3;
plot(solution.phase(iphase).state(:,1),solution.phase(iphase).state(:,2),'g');
axis equal;
 
  x_out0=linspace(0,5,10);
  y_out0=x_out0-x_out0-1;
  x_out1=linspace(5,16,100);
  y_out1=-sqrt(121-(x_out1-5).^2)+10;
  x_out=[x_out0 x_out1];
  y_out=[y_out0 y_out1];
  line([16 16],[10 15],'Color','r');hold on;
  plot(x_out, y_out,'r-'); hold on;
xlabel('X(m)'); ylabel('Y(m)');
legend('实际轨迹','道路边缘线');
 
%%根据需要，绘制各状态量随时间的变化曲线
figure('Color',[1,1,1]);
plot(solution.phase(iphase).time,solution.phase(iphase).control(:,2),'g');hold on;
xlabel('t(s)'); ylabel('V(m/s)');
figure('Color',[1,1,1]);
plot(solution.phase(iphase).time,solution.phase(iphase).control(:,1),'g');hold on;
xlabel('t(s)'); ylabel('转向角(m/s)');
 
 
 
 
 