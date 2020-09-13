clear all
close all
clc



%% premi�re partie
format long

freq=1000;

x0=0 ; y0=0 ; z0=1 ; yaw0=0;
x1=6 ; y1=0 ; z1=1 ; yaw1=0;
x2=1 ; y2=0 ; z2=1 ; yaw2=0;
x3=1 ; y3=1 ; z3=1 ; yaw3=0;
x4=0 ; y4=1 ; z4=1 ; yaw4=0;
x5=0 ; y5=0 ; z5=1 ; yaw5=0;


Z = [x0; y0; z0; yaw0];
A = [x1; y1; z1; yaw1];
B = [x2; y2; z2; yaw2];
C = [x3; y3; z3; yaw3];
D = [x4; y4; z4; yaw4];
E = [x5; y5; z5; yaw5];

th_point = [Z, A, Z];
th_time  = [6, 6, 6];


[m n] = size(th_point);
x = [];
y = [];
z = [];
yaw = [];

xd = [];
yd = [];
zd = [];
yawd = [];

xdd = [];
ydd = [];
zdd = [];
yawdd = [];

P1x = [];
P1y = [];
P1z = [];
P1yaw = [];
%% Getting trajectory
for i = 2:n
[t,xnew,ynew,znew, yawnew,xdnew,ydnew,zdnew,yawdnew,xddnew,yddnew,zddnew,yawddnew, P1xnew, P1ynew, P1znew, P1yawnew] = Trajectory_Raf_Poly7(th_point(:,i-1),th_point(:,i),th_time(i-1),freq);
x = [x, xnew];
y = [y, ynew];
z = [z, znew];
yaw = [yaw, yawnew];

xd = [xd, xdnew];
yd = [yd, ydnew];
zd = [zd, zdnew];
yawd = [yawd, yawdnew];

xdd = [xdd, xddnew];
ydd = [ydd, yddnew];
zdd = [zdd, zddnew];
yawdd = [yawdd, yawddnew];

P1x = [P1x, P1xnew];
P1y = [P1y, P1ynew];
P1z = [P1z, P1znew];
P1yaw = [P1yaw, P1yawnew];
end


%% plot des diff�rentes courbes
figure
subplot(1,3,1)
plot(x);
title(' position x');
subplot(1,3,2)
plot(y);
title(' position y');
subplot(1,3,3)
plot(z);
title(' position z');


figure
plot3(x,y,z);
title(' position 3D');
xlabel('x');
ylabel('y');
zlabel('z');
grid

figure
plot(xd); 
title(' vitesse xd');

figure
plot(yd);
title(' vitesse yd');

figure
plot(zd);
title(' vitesse zd');

figure
plot(xdd); 
title(' acceleration xd');

figure
plot(ydd);
title(' acceleration yd');

figure
plot(zdd);
title(' acceleration zd');

