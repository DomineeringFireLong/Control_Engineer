% sys = tf([133],[1,25,0]);
% dsys = c2d(sys,0.01,'z');
% [num,den]=tfdata(dsys,'v');
b=1;%b小了容易震荡，大了稳定