%% 07函数模块部分
 
% ----------------------------------------------------------------------- %
% ------------------------- BEGIN: vsopcContinuous.m -------------------- %
% ----------------------------------------------------------------------- %
function phaseout = vsopcContinuous(input)
iphase=1;
%% 先获取当前状态量的值
%t  = input.phase.time;
%x11  = input.phase(iphase).state(:,1);
x21  = input.phase(iphase).state(:,2);
x31  = input.phase(iphase).state(:,3);
 
u11   = input.phase(iphase).control(:,1);
u21   = input.phase(iphase).control(:,2);
%% 根据模型,写状态量的微分方程,获得状态量变化率
dx11 = cos(x31).*u21;
dx21 = sin(x31).*u21;
dx31 = tan(u11).*u21;
 
%编写道路约束
%% 将状态量变化率幅值给dynamics结构体变量，用于计算下一周期状态量的值
phaseout(1).dynamics = [dx11,dx21,dx31];
%% 编写该段路径约束，x21及第一段的y坐标值，即在第一阶段限制了y在(-1,1),该限幅大小在设置边界条件时的路径约束限幅中设置
phaseout(1).path=x21;
 
%%同理编写其他段微分方程即路径约束
iphase=2;
%t  = input.phase.time;
x12  = input.phase(iphase).state(:,1);
x22  = input.phase(iphase).state(:,2);
x32  = input.phase(iphase).state(:,3);
 
 
 
u12   = input.phase(iphase).control(:,1);
u22   = input.phase(iphase).control(:,2);
 
dx12 = cos(x32).*u22;
dx22 = sin(x32).*u22;
dx32 = (tan(u12).*u22)/3;
 
%编写道路约束
 
phaseout(2).dynamics = [dx12,dx22,dx32];
phaseout(2).path=sqrt((x22-10).^2+(x12-5).^2)-10;
 
iphase=3;
%t  = input.phase.time;
x13  = input.phase(iphase).state(:,1);
%x23  = input.phase(iphase).state(:,2);
x33  = input.phase(iphase).state(:,3);
 
u13   = input.phase(iphase).control(:,1);
u23   = input.phase(iphase).control(:,2);
 
dx13 = cos(x33).*u23;
dx23 = sin(x33).*u23;
dx33 = (tan(u13).*u23);
 
%编写道路约束
 
phaseout(3).dynamics = [dx13,dx23,dx33];
phaseout(3).path=x13-15;
 
 
end
% ----------------------------------------------------------------------- %
% -------------------------- END: vsopcContinuous.m --------------------- %
% ----------------------------------------------------------------------- %