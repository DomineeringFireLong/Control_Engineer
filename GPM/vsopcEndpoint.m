%% 08目标函数以及段之间连接
% ----------------------------------------------------------------------- %
% -------------------------- BEGIN: vsopcEndpoint.m --------------------- %
% ----------------------------------------------------------------------- %
function output = vsopcEndpoint(input)
 
%%获取第一段各状态量的终值，第二阶段的初值，终值，第三阶段的初值
% Variables at Start and Terminus of Phase 1
 
tf1 = input.phase(1).finaltime;
 
xf1 = input.phase(1).finalstate;  
% Variables at Start and Terminus of Phase 2
t02 = input.phase(2).initialtime;
tf2 = input.phase(2).finaltime;
x02 = input.phase(2).initialstate;
xf2 = input.phase(2).finalstate;
% Variables at Start and Terminus of Phase 3
t03 = input.phase(3).initialtime;
x03 = input.phase(3).initialstate;
 
%%将相邻段的初值、终值进行比较，确保第一段终值和第二阶段的初值相等，从而将不同段之间连接起来
 
% Event Group 1: Linkage Constraints Between Phases 1 and 2
% 本例子有三个状态量，所以将三个状态量都进行对比即x02(1:3)和xf1(1:3).
output.eventgroup(1).event = [x02(1:3)-xf1(1:3), t02-tf1];
% Event Group 2: Linkage Constraints Between Phases 2 and 3
output.eventgroup(2).event = [x03(1:3)-xf2(1:3), t03-tf2];
% Event Group 3: Linkage Constraints Between Phases 3 and 4
 
%%目标函数即为第三阶段的结束时间，即优化目标为时间最优
J  = input.phase(3).finaltime;
 
output.objective = J;
end
% ----------------------------------------------------------------------- %
% --------------------------- END: vsopcEndpoint.m ---------------------- %
% ----------------------------------------------------------------------- %