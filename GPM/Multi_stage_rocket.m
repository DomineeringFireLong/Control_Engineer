%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 功能描述：多级火箭上升问题
% 作者：Lei Lie
% 时间：2024/06/26
% 版本：1.0
% - 完成了代码框架的初始搭建
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc;
clear;
close all;

%% 01.初始参数设置
%----------------------------------------------------%
%------------------- 辅助参数设置 -------------------%
%----------------------------------------------------%
earthRadius         = 6378145;
gravParam           = 3.986012e14;
initialMass         = 301454;
earthRotRate        = 7.29211585e-5;
seaLevelDensity     = 1.225;
densityScaleHeight  = 7200;
g0                  = 9.80665;

scales.length       = earthRadius;
scales.speed        = sqrt(gravParam/scales.length);
scales.time         = scales.length/scales.speed;
scales.acceleration = scales.speed/scales.time;
scales.mass         = initialMass;
scales.force        = scales.mass*scales.acceleration;
scales.area         = scales.length^2;
scales.volume       = scales.area.*scales.length;
scales.density      = scales.mass/scales.volume;
scales.gravparam    = scales.acceleration*scales.length^2;

omega               = earthRotRate*scales.time;
auxdata.omegaMatrix = omega*[0 -1 0;1 0 0;0 0 0];
auxdata.mu          = gravParam/scales.gravparam;
auxdata.cd          = 0.5;
auxdata.sa          = 4*pi/scales.area;
auxdata.rho0        = seaLevelDensity/scales.density;
auxdata.H           = densityScaleHeight/scales.length;
auxdata.Re          = earthRadius/scales.length;
auxdata.g0          = g0/scales.acceleration;

lat0                = 28.5*pi/180;         
x0                  = auxdata.Re*cos(lat0);
y0                  = 0;                   
z0                  = auxdata.Re*sin(lat0);
r0                  = [x0 y0 z0];
v0                  = r0*auxdata.omegaMatrix.';

btSrb = 75.2/scales.time;
btFirst = 261/scales.time;
btSecond = 700/scales.time;

t0 = 0/scales.time;
t1 = 75.2/scales.time;
t2 = 150.4/scales.time;
t3 = 261/scales.time;
t4 = 961/scales.time;

mTotSrb      = 19290/scales.mass;
mPropSrb     = 17010/scales.mass;
mDrySrb      = mTotSrb-mPropSrb;
mTotFirst    = 104380/scales.mass;
mPropFirst   = 95550/scales.mass;
mDryFirst    = mTotFirst-mPropFirst;
mTotSecond   = 19300/scales.mass;
mPropSecond  = 16820/scales.mass;
mDrySecond   = mTotSecond-mPropSecond;
mPayload     = 4164/scales.mass;
thrustSrb    = 628500/scales.force;
thrustFirst  = 1083100/scales.force;
thrustSecond = 110094/scales.force;
mdotSrb      = mPropSrb/btSrb;
ispSrb       = thrustSrb/(auxdata.g0*mdotSrb);
mdotFirst    = mPropFirst/btFirst;
ispFirst     = thrustFirst/(auxdata.g0*mdotFirst);
mdotSecond   = mPropSecond/btSecond;
ispSecond    = thrustSecond/(auxdata.g0*mdotSecond);

af = 24361140/scales.length;
ef = 0.7308;
incf = 28.5*pi/180;
Omf = 269.8*pi/180;
omf = 130.5*pi/180;
nuguess = 0;
cosincf = cos(incf);
cosOmf = cos(Omf);
cosomf = cos(omf);
oe = [af ef incf Omf omf nuguess];
[rout,vout] = launchoe2rv(oe,auxdata.mu);
rout = rout';
vout = vout';

m10 = mPayload+mTotSecond+mTotFirst+9*mTotSrb;
m1f = m10-(6*mdotSrb+mdotFirst)*t1;
m20 = m1f-6*mDrySrb;
m2f = m20-(3*mdotSrb+mdotFirst)*(t2-t1);
m30 = m2f-3*mDrySrb;
m3f = m30-mdotFirst*(t3-t2);
m40 = m3f-mDryFirst;
m4f = mPayload;

auxdata.thrustSrb    = thrustSrb;
auxdata.thrustFirst  = thrustFirst;
auxdata.thrustSecond = thrustSecond;
auxdata.ispSrb       = ispSrb;
auxdata.ispFirst     = ispFirst;
auxdata.ispSecond    = ispSecond;

rmin = -2*auxdata.Re;
rmax = -rmin;
vmin = -10000/scales.speed;
vmax = -vmin;

%% 02.边界条件设置
%----------------------------------------------------%
%---------------- 第一阶段参数设置 ------------------%
%----------------------------------------------------%
iphase = 1;
% 边界条件
bounds.phase(iphase).initialtime.lower = [t0]; 
bounds.phase(iphase).initialtime.upper = [t0]; 
bounds.phase(iphase).finaltime.lower = [t1]; 
bounds.phase(iphase).finaltime.upper = [t1]; 

bounds.phase(iphase).initialstate.lower = [r0(1:3),v0(1:3),m10];    
bounds.phase(iphase).initialstate.upper = [r0(1:3),v0(1:3),m10];  

bounds.phase(iphase).state.lower = [rmin*ones(1,3),vmin*ones(1,3),m1f];
bounds.phase(iphase).state.upper = [rmax*ones(1,3),vmax*ones(1,3),m10];

bounds.phase(iphase).finalstate.lower = [rmin*ones(1,3),vmin*ones(1,3),m1f]; 
bounds.phase(iphase).finalstate.upper = [rmax*ones(1,3),vmax*ones(1,3),m10]; 

bounds.phase(iphase).control.lower = -ones(1,3);
bounds.phase(iphase).control.upper = +ones(1,3);

bounds.phase(iphase).path.lower  = 1;
bounds.phase(iphase).path.upper  = 1;

% path
% 通常是所求解问题中的路径约束，也就是等式约束，用于描述整个求解时间历程里状态量或控制量必须满足的条件。
% 这个部分其实也体现了GPOPS-II的部分局限性，它没有很好地办法处理不等式约束或者非线性约束，只能通过强硬地约束上下界来控制变量的边界。
% 这和估计的精度有很大关联，如果估计的约束上下限不准确，会造成求解难度加大。


%----------------------------------------------------%
%---------------- 第二阶段参数设置 ------------------%
%----------------------------------------------------%
iphase = 2;
% 边界条件
bounds.phase(iphase).initialtime.lower = [t1]; 
bounds.phase(iphase).initialtime.upper = [t1]; 
bounds.phase(iphase).finaltime.lower = [t2]; 
bounds.phase(iphase).finaltime.upper = [t2]; 
bounds.phase(iphase).initialstate.lower = [rmin*ones(1,3),vmin*ones(1,3),m2f];
bounds.phase(iphase).initialstate.upper = [rmax*ones(1,3),vmax*ones(1,3),m20];
bounds.phase(iphase).state.lower = [rmin*ones(1,3),vmin*ones(1,3),m2f];
bounds.phase(iphase).state.upper = [rmax*ones(1,3),vmax*ones(1,3),m20];
bounds.phase(iphase).finalstate.lower = [rmin*ones(1,3),vmin*ones(1,3),m2f]; 
bounds.phase(iphase).finalstate.upper = [rmax*ones(1,3),vmax*ones(1,3),m20]; 
bounds.phase(iphase).control.lower = -ones(1,3);
bounds.phase(iphase).control.upper = +ones(1,3);

bounds.phase(iphase).path.lower  = 1;
bounds.phase(iphase).path.upper  = 1;

%----------------------------------------------------%
%---------------- 第三阶段参数设置 ------------------%
%----------------------------------------------------%
iphase = 3;
% 边界条件
bounds.phase(iphase).initialtime.lower = [t2]; 
bounds.phase(iphase).initialtime.upper = [t2]; 
bounds.phase(iphase).finaltime.lower = [t3]; 
bounds.phase(iphase).finaltime.upper = [t3]; 
bounds.phase(iphase).initialstate.lower = [rmin*ones(1,3),vmin*ones(1,3),m3f];
bounds.phase(iphase).initialstate.upper = [rmax*ones(1,3),vmax*ones(1,3),m30];
bounds.phase(iphase).state.lower = [rmin*ones(1,3),vmin*ones(1,3),m3f];
bounds.phase(iphase).state.upper = [rmax*ones(1,3),vmax*ones(1,3),m30];
bounds.phase(iphase).finalstate.lower = [rmin*ones(1,3),vmin*ones(1,3),m3f]; 
bounds.phase(iphase).finalstate.upper = [rmax*ones(1,3),vmax*ones(1,3),m30]; 
bounds.phase(iphase).control.lower = -ones(1,3);
bounds.phase(iphase).control.upper = +ones(1,3);
bounds.phase(iphase).path.lower  = 1;
bounds.phase(iphase).path.upper  = 1;

%----------------------------------------------------%
%---------------- 第四阶段参数设置 ------------------%
%----------------------------------------------------%
iphase = 4;
% 边界条件
bounds.phase(iphase).initialtime.lower = [t3]; 
bounds.phase(iphase).initialtime.upper = [t3]; 
bounds.phase(iphase).finaltime.lower = [t3]; 
bounds.phase(iphase).finaltime.upper = [t4]; 

bounds.phase(iphase).initialstate.lower = [rmin*ones(1,3),vmin*ones(1,3),m4f];
bounds.phase(iphase).initialstate.upper = [rmax*ones(1,3),vmax*ones(1,3),m40];
bounds.phase(iphase).state.lower = [rmin*ones(1,3),vmin*ones(1,3),m4f];
bounds.phase(iphase).state.upper = [rmax*ones(1,3),vmax*ones(1,3),m40];
bounds.phase(iphase).finalstate.lower = [rmin*ones(1,3),vmin*ones(1,3),m4f]; 
bounds.phase(iphase).finalstate.upper = [rmax*ones(1,3),vmax*ones(1,3),m40]; 

bounds.phase(iphase).control.lower = -ones(1,3);
bounds.phase(iphase).control.upper = +ones(1,3);

bounds.phase(iphase).path.lower  = 1;
bounds.phase(iphase).path.upper  = 1;

%----------------------------------------------------%
%------------- 阶段切换的连续性保证条件 -------------%
% eventgroup是用来管理GPOPS-II中事件触发的集合，一般多用于解决多阶段的最优控制中多阶段之间的变量连续性问题。主要有以下几个作用：
%     触发事件的处理。
%     约束条件的激活。
%     控制参数的切换。
%----------------------------------------------------%
bounds.eventgroup(1).lower = [zeros(1,6), -6*mDrySrb, 0];%七个状态变量分界点的变化，分级火箭质量变化是突变的所以不为0，最后一个是时间差
bounds.eventgroup(1).upper = [zeros(1,6), -6*mDrySrb, 0];
bounds.eventgroup(2).lower = [zeros(1,6), -3*mDrySrb, 0];
bounds.eventgroup(2).upper = [zeros(1,6), -3*mDrySrb, 0];
bounds.eventgroup(3).lower = [zeros(1,6), -mDryFirst, 0];
bounds.eventgroup(3).upper = [zeros(1,6), -mDryFirst, 0];

%----------------------------------------------------%
%-------------- 第四阶段的终端条件约束 --------------%
%----------------------------------------------------%
bounds.eventgroup(4).lower = [af, ef, incf, Omf, omf];
bounds.eventgroup(4).upper = [af, ef, incf, Omf, omf];

%% 03.初值猜测
%----------------------------------------------------%
%---------------- 第一阶段参数设置 ------------------%
%----------------------------------------------------%
iphase = 1;
% 初值猜测
guess.phase(iphase).time = [t0; t1];
guess.phase(iphase).state(:,1) = [r0(1); r0(1)];
guess.phase(iphase).state(:,2) = [r0(2); r0(2)];
guess.phase(iphase).state(:,3) = [r0(3); r0(3)];
guess.phase(iphase).state(:,4) = [v0(1); v0(1)];
guess.phase(iphase).state(:,5) = [v0(2); v0(2)];
guess.phase(iphase).state(:,6) = [v0(3); v0(3)];
guess.phase(iphase).state(:,7) = [m10; m1f];
guess.phase(iphase).control(:,1) = [0; 0];
guess.phase(iphase).control(:,2) = [1; 1];
guess.phase(iphase).control(:,3) = [0; 0];

%----------------------------------------------------%
%---------------- 第二阶段参数设置 ------------------%
%----------------------------------------------------%
iphase = 2;
% 初值猜测
guess.phase(iphase).time = [t1; t2];
guess.phase(iphase).state(:,1) = [r0(1); r0(1)];
guess.phase(iphase).state(:,2) = [r0(2); r0(2)];
guess.phase(iphase).state(:,3) = [r0(3); r0(3)];
guess.phase(iphase).state(:,4) = [v0(1); v0(1)];
guess.phase(iphase).state(:,5) = [v0(2); v0(2)];
guess.phase(iphase).state(:,6) = [v0(3); v0(3)];
guess.phase(iphase).state(:,7) = [m10; m1f];
guess.phase(iphase).control(:,1) = [0; 0];
guess.phase(iphase).control(:,2) = [1; 1];
guess.phase(iphase).control(:,3) = [0; 0];

%----------------------------------------------------%
%---------------- 第一阶段参数设置 ------------------%
%----------------------------------------------------%
iphase = 3;
% 初值猜测
guess.phase(iphase).time       = [t2; t3];
guess.phase(iphase).state(:,1) = [rout(1); rout(1)];
guess.phase(iphase).state(:,2) = [rout(2); rout(2)];
guess.phase(iphase).state(:,3) = [rout(3); rout(3)];
guess.phase(iphase).state(:,4) = [vout(1); vout(1)];
guess.phase(iphase).state(:,5) = [vout(2); vout(2)];
guess.phase(iphase).state(:,6) = [vout(3); vout(3)];
guess.phase(iphase).state(:,7) = [m30; m3f];
guess.phase(iphase).control(:,1) = [0; 0];
guess.phase(iphase).control(:,2) = [1; 1];
guess.phase(iphase).control(:,3) = [0; 0];

%----------------------------------------------------%
%---------------- 第一阶段参数设置 ------------------%
%----------------------------------------------------%
iphase = 4;
% 初值猜测
guess.phase(iphase).time    = [t3; t4];
guess.phase(iphase).state(:,1) = [rout(1) rout(1)];
guess.phase(iphase).state(:,2) = [rout(2) rout(2)];
guess.phase(iphase).state(:,3) = [rout(3) rout(3)];
guess.phase(iphase).state(:,4) = [vout(1) vout(1)];
guess.phase(iphase).state(:,5) = [vout(2) vout(2)];
guess.phase(iphase).state(:,6) = [vout(3) vout(3)];
guess.phase(iphase).state(:,7) = [m40; m4f];
guess.phase(iphase).control(:,1) = [0; 0];
guess.phase(iphase).control(:,2) = [1; 1];
guess.phase(iphase).control(:,3) = [0; 0];

%% 04.设置GPOPS求解器参数
%----------------------------------------------------%
%-------------- 每个阶段的网格细化条件 --------------%
%----------------------------------------------------%
for i=1:4
  meshphase(i).colpoints = 4*ones(1,10);
  meshphase(i).fraction = 0.1*ones(1,10);
end

setup.name = 'Launch-Vehicle-Ascent-Problem';
setup.functions.continuous = @launchContinuous;
setup.functions.endpoint = @launchEndpoint;
setup.mesh.phase = meshphase;
setup.nlp.solver = 'snopt';
setup.bounds = bounds;
setup.guess = guess;
setup.auxdata = auxdata;
setup.derivatives.supplier = 'sparseFD';
setup.derivatives.derivativelevel = 'second';
setup.derivatives.dependencies = 'sparseNaN';
setup.scales.method = 'automatic-bounds';
setup.mesh.method = 'hp1';
setup.mesh.tolerance = 1e-6;
setup.method = 'RPMintegration';

%% 05.求解
%----------------------------------------------------%
%----------- 使用 GPOPS2 求解最优控制问题 -----------%
%----------------------------------------------------%
totaltime = tic;
output = gpops2(setup);
totaltime = toc(totaltime);

%% 06.画图
% 数据处理
solution    = output.result.solution;
%----------------------------------------------------%
%-------------------- 第一阶段 ----------------------%
%----------------------------------------------------%
t{1}        = solution.phase(1).time*1e3;
rad1        = solution.phase(1).state(:,1:3);
alt{1}      = (sqrt(dot(rad1,rad1,2))-auxdata.Re)*scales.length/1000;
velocity1   = solution.phase(1).state(:,4:6);
speed{1}    = sqrt(dot(velocity1,velocity1,2));
mass{1}     = solution.phase(1).state(:,7);
control{1}  = solution.phase(1).control;

%----------------------------------------------------%
%-------------------- 第二阶段 ----------------------%
%----------------------------------------------------%
t{2}        = solution.phase(2).time*1e3;
rad2        = solution.phase(2).state(:,1:3);
alt{2}      = (sqrt(dot(rad2,rad2,2))-auxdata.Re)*scales.length/1000;
velocity2   = solution.phase(2).state(:,4:6);
speed{2}    = sqrt(dot(velocity2,velocity2,2));
mass{2}     = solution.phase(2).state(:,7);
control{2}  = solution.phase(2).control;

%----------------------------------------------------%
%-------------------- 第三阶段 ----------------------%
%----------------------------------------------------%
t{3}        = solution.phase(3).time*1e3;
rad3        = solution.phase(3).state(:,1:3);
alt{3}      = (sqrt(dot(rad3,rad3,2))-auxdata.Re)*scales.length/1000;
velocity3   = solution.phase(3).state(:,4:6);
speed{3}    = sqrt(dot(velocity3,velocity3,2));
mass{3}     = solution.phase(3).state(:,7);
control{3}  = solution.phase(3).control;

%----------------------------------------------------%
%-------------------- 第四阶段 ----------------------%
%----------------------------------------------------%
t{4}        = solution.phase(4).time*1e3;
rad4        = solution.phase(4).state(:,1:3);
alt{4}      = (sqrt(dot(rad4,rad4,2))-auxdata.Re)*scales.length/1000;
speed{4}    = solution.phase(4).state(:,4:6);
velocity4   = solution.phase(4).state(:,4:6);
speed{4}    = sqrt(dot(velocity4,velocity4,2));
mass{4}     = solution.phase(4).state(:,7);
control{4}  = solution.phase(4).control;

% 第一张图：高度
figure('Color',[1,1,1])
pp = plot(t{1},alt{1},'-o',t{2},alt{2},'-o',t{3},alt{3},'-o',t{4},alt{4},'-o');
xl = xlabel('time (s)');
yl = ylabel('altitude (km)');
ll = legend('Phase 1','Phase 2','Phase 3','Phase 4','Location','SouthEast');
set(xl,'FontSize',14);
set(yl,'FontSize',14);
set(ll,'FontSize',14);
set(pp,'LineWidth',1.25);
set(gca,'FontSize',14,'FontName','Times New Roman');
print -dpng launchAltitude.png

% 第二张图：速度
figure('Color',[1,1,1])
pp = plot(t{1},speed{1},'-o',t{2},speed{2},'-o',t{3},speed{3},'-o',t{4},speed{4},'-o');
xl = xlabel('time (s)');
yl = ylabel('speed (m/s)');
ll = legend('Phase 1','Phase 2','Phase 3','Phase 4','Location','SouthEast');
set(xl,'FontSize',14);
set(yl,'FontSize',14);
set(ll,'FontSize',14);
set(pp,'LineWidth',1.5);
set(gca,'FontSize',14,'FontName','Times New Roman');
print -dpng launchSpeed.png

% 第三张图：控制量
figure('Color',[1,1,1])
pp = plot(t{1},control{1},'-o',t{2},control{2},'-o',t{3},control{3},'-o',t{4},control{4},'-o');
xl = xlabel('time (s)');
yl = ylabel('control');
ll = legend('Phase 1','Phase 2','Phase 3','Phase 4','Location','SouthWest');
set(xl,'FontSize',14);
set(yl,'FontSize',14);
set(ll,'FontSize',14);
set(pp,'LineWidth',1.5);
set(gca,'FontSize',14,'FontName','Times New Roman');
print -dpng launchControl.png

%-------------------------------------------------------------------------%
%------------------- BEGIN Function launchContinuous.m -------------------%
%-------------------------------------------------------------------------%
function phaseout = launchContinuous(input)
%----------------------------------------------------%
%--------------- 第一阶段动力学方程 -----------------%
%----------------------------------------------------%
t1 = input.phase(1).time;
x1 = input.phase(1).state;
u1 = input.phase(1).control;
r1 = x1(:,1:3);
v1 = x1(:,4:6);
m1 = x1(:,7);

rad1                = sqrt(sum(r1.*r1,2));
omegaMatrix         = input.auxdata.omegaMatrix;
omegacrossr         = r1*omegaMatrix.';
vrel1               = v1-omegacrossr;
speedrel1           = sqrt(sum(vrel1.*vrel1,2));
h1                  = rad1-input.auxdata.Re;
rho1                = input.auxdata.rho0*exp(-h1/input.auxdata.H);
bc1                 = (rho1./(2*m1)).*input.auxdata.sa*input.auxdata.cd;
bcspeed1            = bc1.*speedrel1;
bcspeedmat1         = repmat(bcspeed1,1,3);
Drag1               = -bcspeedmat1.*vrel1;
muoverradcubed1     = input.auxdata.mu./rad1.^3;
muoverradcubedmat1  = [muoverradcubed1 muoverradcubed1 muoverradcubed1];
grav1               = -muoverradcubedmat1.*r1;

TSrb1   = 6*input.auxdata.thrustSrb*ones(size(t1));
TFirst1 = input.auxdata.thrustFirst*ones(size(t1));
TTot1   = TSrb1+TFirst1;
m1dot1  = -TSrb1./(input.auxdata.g0*input.auxdata.ispSrb);
m2dot1  = -TFirst1./(input.auxdata.g0*input.auxdata.ispFirst);

% 第一阶段：动力学约束
Toverm1     = TTot1./m1;
Tovermmat1  = [Toverm1 Toverm1 Toverm1];
thrust1     = Tovermmat1.*u1;
rdot1       = v1;
vdot1       = thrust1+Drag1+grav1;
mdot1       = m1dot1+m2dot1;
phaseout(1).dynamics = [rdot1 vdot1 mdot1];

% 第一阶段：路径约束
path1   = [sum(u1.*u1,2)];%让u^2=1
phaseout(1).path = path1;

%----------------------------------------------------%
%--------------- 第二阶段动力学方程 -----------------%
%----------------------------------------------------%
t2 = input.phase(2).time;
x2 = input.phase(2).state;
u2 = input.phase(2).control;
r2 = x2(:,1:3);
v2 = x2(:,4:6);
m2 = x2(:,7);

rad2                = sqrt(sum(r2.*r2,2));
omegaMatrix         = input.auxdata.omegaMatrix;
omegacrossr         = r2*omegaMatrix.';
vrel2               = v2-omegacrossr;
speedrel2           = sqrt(sum(vrel2.*vrel2,2));
h2                  = rad2-input.auxdata.Re;
rho2                = input.auxdata.rho0*exp(-h2/input.auxdata.H);
bc2                 = (rho2./(2*m2)).*input.auxdata.sa*input.auxdata.cd;
bcspeed2            = bc2.*speedrel2;
bcspeedmat2         = repmat(bcspeed2,1,3);
Drag2               = -bcspeedmat2.*vrel2;
muoverradcubed2     = input.auxdata.mu./rad2.^3;
muoverradcubedmat2  = [muoverradcubed2 muoverradcubed2 muoverradcubed2];
grav2               = -muoverradcubedmat2.*r2;

TSrb2   = 3*input.auxdata.thrustSrb*ones(size(t2));
TFirst2 = input.auxdata.thrustFirst*ones(size(t2));
TTot2   = TSrb2+TFirst2;
m1dot2  = -TSrb2./(input.auxdata.g0*input.auxdata.ispSrb);
m2dot2  = -TFirst2./(input.auxdata.g0*input.auxdata.ispFirst);

% 第二阶段：动力学约束
Toverm2     = TTot2./m2;
Tovermmat2  = [Toverm2 Toverm2 Toverm2];
thrust2     = Tovermmat2.*u2;
rdot2       = v2;
vdot2       = thrust2+Drag2+grav2;
mdot2       = m1dot2+m2dot2;    
phaseout(2).dynamics = [rdot2 vdot2 mdot2];

% 第二阶段：路径约束
path2   = [sum(u2.*u2,2)];
phaseout(2).path = path2;

%----------------------------------------------------%
%--------------- 第三阶段动力学方程 -----------------%
%----------------------------------------------------%
t3 = input.phase(3).time;
x3 = input.phase(3).state;
u3 = input.phase(3).control;
r3 = x3(:,1:3);
v3 = x3(:,4:6);
m3 = x3(:,7);

rad3 = sqrt(sum(r3.*r3,2));
omegaMatrix         = input.auxdata.omegaMatrix;
omegacrossr         = r3*omegaMatrix.';
vrel3               = v3-omegacrossr;
speedrel3           = sqrt(sum(vrel3.*vrel3,2));
h3                  = rad3-input.auxdata.Re;
rho3                = input.auxdata.rho0*exp(-h3/input.auxdata.H);
bc3                 = (rho3./(2*m3)).*input.auxdata.sa*input.auxdata.cd;
bcspeed3            = bc3.*speedrel3;
bcspeedmat3         = repmat(bcspeed3,1,3);
Drag3               = -bcspeedmat3.*vrel3;
muoverradcubed3     = input.auxdata.mu./rad3.^3;
muoverradcubedmat3  = [muoverradcubed3 muoverradcubed3 muoverradcubed3];
grav3               = -muoverradcubedmat3.*r3;

% 第三阶段：动力学约束
TTot3       = input.auxdata.thrustFirst*ones(size(t3));
Toverm3     = TTot3./m3;
Tovermmat3  = [Toverm3 Toverm3 Toverm3];
thrust3     = Tovermmat3.*u3;
rdot3       = v3;
vdot3       = thrust3+Drag3+grav3;
mdot3       = -TTot3./(input.auxdata.g0*input.auxdata.ispFirst);
phaseout(3).dynamics = [rdot3 vdot3 mdot3];

% 第三阶段：路径约束
path3 = [sum(u3.*u3,2)];
phaseout(3).path = path3;

%----------------------------------------------------%
%--------------- 第四阶段动力学方程 -----------------%
%----------------------------------------------------%
t4 = input.phase(4).time;
x4 = input.phase(4).state;
u4 = input.phase(4).control;
r4 = x4(:,1:3);
v4 = x4(:,4:6);
m4 = x4(:,7);

rad4                = sqrt(sum(r4.*r4,2));
omegacrossr         = r4*input.auxdata.omegaMatrix.';
vrel4               = v4-omegacrossr;
speedrel4           = sqrt(sum(vrel4.*vrel4,2));
h4                  = rad4-input.auxdata.Re;
rho4                = input.auxdata.rho0*exp(-h4/input.auxdata.H);
bc4                 = (rho4./(2*m4)).*input.auxdata.sa*input.auxdata.cd;
bcspeed4            = bc4.*speedrel4;
bcspeedmat4         = repmat(bcspeed4,1,3);
Drag4               = -bcspeedmat4.*vrel4;
muoverradcubed4     = input.auxdata.mu./rad4.^3;
muoverradcubedmat4  = [muoverradcubed4 muoverradcubed4 muoverradcubed4];
grav4               = -muoverradcubedmat4.*r4;

% 第四阶段：动力学约束
TTot4       = input.auxdata.thrustSecond*ones(size(t4));
Toverm4     = TTot4./m4;
Tovermmat4  = [Toverm4 Toverm4 Toverm4];
thrust4     = Tovermmat4.*u4;
rdot4       = v4;
vdot4       = thrust4+Drag4+grav4;
mdot4       = -TTot4/(input.auxdata.g0*input.auxdata.ispSecond);
phaseout(4).dynamics = [rdot4 vdot4 mdot4];

% 第四阶段：路径约束
path4 = [sum(u4.*u4,2)];
phaseout(4).path = path4;
end
%-------------------------------------------------------------------------%
%--------------------- END Function launchContinuous.m -------------------%
%-------------------------------------------------------------------------%

%-------------------------------------------------------------------------%
%--------------------- BEGIN Function launchEndpoint.m -------------------%
%-------------------------------------------------------------------------%
function output = launchEndpoint(input)
%----------------------------------------------------%
%-------------------- 第一阶段 ----------------------%
%----------------------------------------------------%
t0{1} = input.phase(1).initialtime;
tf{1} = input.phase(1).finaltime;
x0{1} = input.phase(1).initialstate;
xf{1} = input.phase(1).finalstate;

%----------------------------------------------------%
%-------------------- 第二阶段 ----------------------%
%----------------------------------------------------%
t0{2} = input.phase(2).initialtime;
tf{2} = input.phase(2).finaltime;
x0{2} = input.phase(2).initialstate;
xf{2} = input.phase(2).finalstate;

%----------------------------------------------------%
%-------------------- 第三阶段 ----------------------%
%----------------------------------------------------%
t0{3} = input.phase(3).initialtime;
tf{3} = input.phase(3).finaltime;
x0{3} = input.phase(3).initialstate;
xf{3} = input.phase(3).finalstate;

%----------------------------------------------------%
%-------------------- 第四阶段 ----------------------%
%----------------------------------------------------%
t0{4} = input.phase(4).initialtime;
tf{4} = input.phase(4).finaltime;
x0{4} = input.phase(4).initialstate;
xf{4} = input.phase(4).finalstate;

% Event Group 1: 连结第一阶段和第二阶段
output.eventgroup(1).event = [x0{2}(1:7)-xf{1}(1:7), t0{2}-tf{1}];
% Event Group 2: 连结第二阶段和第三阶段
output.eventgroup(2).event = [x0{3}(1:7)-xf{2}(1:7), t0{3}-tf{2}];
% Event Group 3: 连结第三阶段和第四阶段
output.eventgroup(3).event = [x0{4}(1:7)-xf{3}(1:7), t0{4}-tf{3}];
% Event Group 4: 进入至GTO轨道的终端约束
orbitalElements = launchrv2oe(xf{4}(1:3).',xf{4}(4:6).',input.auxdata.mu);
output.eventgroup(4).event = orbitalElements(1:5).';

% 性能指标
output.objective = -xf{4}(7);
end
%-------------------------------------------------------------------------%
%--------------------- END Function launchEndpoint.m ---------------------%
%-------------------------------------------------------------------------%
%-------------------------------------------------------------------------%
%------------------- BEGIN Function launchrv2oe.m ------------------------%
%-------------------------------------------------------------------------%
function oe = launchrv2oe(rv,vv,mu)
K  = [0;0;1];
hv = cross(rv,vv);
nv = cross(K,hv);
n  = sqrt(nv.'*nv);
h2 = (hv.'*hv);
v2 = (vv.'*vv);
r  = sqrt(rv.'*rv);
ev = 1/mu *( (v2-mu/r)*rv - (rv.'*vv)*vv );
p  = h2/mu;
e  = sqrt(ev.'*ev);
a  = p/(1-e*e);
i  = acos(hv(3)/sqrt(h2));
Om = acos(nv(1)/n);
if (nv(2)<0-eps)
  Om = 2*pi-Om;
end
om = acos(nv.'*ev/n/e);
if (ev(3)<0)
  om = 2*pi-om;
end
nu = acos(ev.'*rv/e/r);
if (rv.'*vv<0)
  nu = 2*pi-nu;
end
oe = [a; e; i; Om; om; nu];
end
%-------------------------------------------------------------------------%
%---------------------- END Function launchrv2oe.m -----------------------%
%-------------------------------------------------------------------------%
%-------------------------------------------------------------------------%
%------------------- BEGIN Function launchoe2rv.m ------------------------%
%-------------------------------------------------------------------------%
function [ri,vi] = launchoe2rv(oe,mu)
a=oe(1); e=oe(2); i=oe(3); Om=oe(4); om=oe(5); nu=oe(6);
p = a*(1-e*e);
r = p/(1+e*cos(nu));
rv = [r*cos(nu); r*sin(nu); 0];
vv = sqrt(mu/p)*[-sin(nu); e+cos(nu); 0];
cO = cos(Om);  sO = sin(Om);
co = cos(om);  so = sin(om);
ci = cos(i);   si = sin(i);
R  = [cO*co-sO*so*ci  -cO*so-sO*co*ci  sO*si;
      sO*co+cO*so*ci  -sO*so+cO*co*ci -cO*si;
      so*si            co*si           ci];
ri = R*rv;
vi = R*vv;
end
%-------------------------------------------------------------------------%
%---------------------- END Function launchoe2rv.m -----------------------%
%-------------------------------------------------------------------------%
