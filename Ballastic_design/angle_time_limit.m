%具有角度和时间约束的导弹最优全弹道设计
%根据论文，初制导功能在于控制导弹快速爬升
%指令高度：Hc=1000+35*t，以35m/s快速爬升
%指令偏航角为-45deg
clear
clc
%---------------------导弹参数设置-----------------------
V=260; % 单位：m/s，导弹飞行速度
theta=45*pi/180;%单位：rad，初始弹道倾角
fea=-45*pi/180;%单位：rad，初始弹道偏角
T_1=14.2;%单位：s，初制导时间
H=1000;%单位：m，飞行高度
g=9.6;%单位：m/s^2，重力加速度
X=0; %单位：m，拦截弹位置坐标，X轴
Y=1000; %单位：m，拦截弹位置坐标，Y轴
Z=0; %单位：m，拦截弹位置坐标，Z轴
 
%------------------变结构控制律参数设置-----------------------
c1=30;   %纵向通道
epu1=.1;%纵向通道
k1=.1;%纵向通道
c2=30;%偏航通道
epu2=.0005;%偏航通道
k2=.5;%偏航通道
 
dH=0;
dfea=0;
t=0; %单位：s，仿真时刻
dt=0.001;%单位：s，仿真时间步长
n=1;%计数用
 
for i=1:14200   %循环14200次，对应初制导时间14.2s
    
    %-------------------------纵向通道------------------------------
    Hc=1000+35*t;  %高度指令
    dHc=35;%高度指令的导数
    ddHc=0;%高度指令的二阶导数
    
    e1=Hc-H;  %高度误差
    de1=dHc-dH; %高度误差的导数
    S1=c1*e1+de1;  %切换函数（论文中式（4））
    dS1=-epu1*sign(S1)-k1*S1;%指数趋近律（论文中式（5））
    ny=1/g*(c1*(dHc-dH)+ddHc+(epu1+10)*sign(S1)+k1*S1);  %控制输入（论文中式（6））
    
    dH=V*sin(theta); %纵向通道：高度变化函数（论文中式（1））
    dtheta=g/V*(ny-cos(theta)); %纵向通道：弹道倾角变化函数（论文中式（1））
    H=H+dH*dt;
    theta=theta+dtheta*dt;
    
    %-------------------------偏航通道------------------------------
    feac=-45*pi/180;  %弹道偏角指令
    dfeac=0;%弹道偏角指令的导数
    
    e2=feac-fea;  %弹道偏角误差
    de2=dfeac-dfea; %弹道偏角误差的导数
    S2=c2*e2+de2;  %切换函数
    dS2=-epu2*sign(S2)-k2*S2;%指数趋近律
    nz=-V/g*(c2*(feac-fea)+dfeac+(epu2)*sign(S2)+k2*S2);  %控制输入（论文中式（8））
    
    dfea=-g/(V*cos(theta))*nz;%偏航通道：弹道偏角变化函数（论文中式（7））
    fea=fea+dfea*dt;
    
    %----------------------------计算坐标----------------------------
    X=X+V*cos(theta)*cos(fea)*dt;
    Y=Y+V*sin(theta)*dt;
    Z=Z-V*cos(theta)*sin(fea)*dt;
    
    %-------------------------保存结果------------------------------
    H_store(:,n)=[H;Hc]; %第一行为实际飞行高度，第二行为指令高度   
    theta_store(n)=theta;  %保存弹道倾角
    fea_store(:,n)=[fea;feac];   %第一行为实际弹道偏角，第二行为指令弹道偏角   
    ny_store(n)=ny;   %保存纵向过载
    nz_store(n)=nz;   %保存偏航过载
    P_store(:,n)=[X;Y;Z];
    n=n+1;  %每循环一次，计数加一
    t=t+dt; %往前推进一个仿真步长
end
 
figure(1)
plot((1:n-1)*dt,H_store(1,:),(1:500:n-1)*dt,H_store(2,1:500:end),'+')
xlabel('time/s')
ylabel('Altitude/m')
legend('实际飞行高度','指令高度')
title('初制导：高度变化情况')
 
figure(2)
plot((1:n-1)*dt,theta_store*180/pi)
xlabel('time/s')
ylabel('弹道倾角/deg')
title('初制导：弹道倾角变化情况')
 
figure(3)
plot((1:n-1)*dt,ny_store)
xlabel('time/s')
ylabel('纵向过载ny')
title('初制导：纵向过载变化情况')
 
figure(4)
plot((1:n-1)*dt,fea_store(1,:)*180/pi,(1:500:n-1)*dt,fea_store(2,1:500:end)*180/pi,'+')
xlabel('time/s')
ylabel('弹道偏角/deg')
legend('实际弹道偏角','指令弹道偏角')
title('初制导：弹道偏角变化情况')
 
figure(5)
plot((1:n-1)*dt,nz_store)
xlabel('time/s')
ylabel('偏航过载nz')
title('初制导：偏航过载变化情况')
 
figure(6)%注意matlab三维坐标画图X、Y、Z轴方向与导弹飞行力学坐标系不同
plot3(P_store(1,:),P_store(3,:),P_store(2,:))
xlabel('X/m')
ylabel('-Z/m')
zlabel('Y/m')
title('拦截弹位置坐标')