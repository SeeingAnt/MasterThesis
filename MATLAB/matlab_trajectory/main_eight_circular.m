%% Script for generating an eight trajectory or a circular trajectory 
%% through Bezier curves
close all
clear all

%% getting coefficients

filename= 'eight.csv';
Coeff = csvread(filename, 1,0);

%% Poly Seventh
format long 
res = 200; % resolution
freq = 200;

[m, n] = size(Coeff);

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

for i = 1:m
    
[xnew,ynew,znew, yawnew,xdnew,ydnew,zdnew,yawdnew,xddnew,yddnew,zddnew,yawddnew] = FromCoeffToTraj(Coeff(i,:),res);

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

end

plot3(x,y,z);
% roundn = @(x,n) round(x*10^n)./10^n;
% x_traj = roundn(x,10);
T = [x', y', z', yaw', yaw', yaw', xd', yd', zd', xdd', ydd', zdd', zeros(size(yaw')), zeros(size(yaw')),zeros(size(yaw'))];
%T(T<1e-4) = 0;

address=pwd;
ind=find(address=='/');
writematrix(T,strcat(address(1:ind(end)),'ROS/src/rotors_gazebo/src/eight_traj.txt'),'Delimiter','tab');





