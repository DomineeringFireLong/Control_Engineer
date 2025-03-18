t=0:0.001:1;
u=2*t-1;
y1=glfdiff(u,t,0.5);
y2=glfdiff(u,t,-0.5);
plot(t,y1)
% plot(t,y1,t,y2)
