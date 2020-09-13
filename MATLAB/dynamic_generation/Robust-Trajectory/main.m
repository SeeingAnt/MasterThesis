
clear all; clc; close all;
addpath('tool')
addpath('../traj_sim')
addpath('polynomial_chaos')

traj_name = '../traj_sim/RobustFlip';
%% Initial parameters

par=InitParameter();   

x0 = [reshape(ones(40,3).*[reshape(ones(12,3).*[0;0;1;0;0;0;0;0;0;0;0;0],1,3*12)';par.m*par.g;0;0;0],1,40*3)';par.m*par.g;0;0;0];

% % flip
trajectory(1).xf = [NaN; NaN; NaN; par.flips*2*pi; 0; 0];
trajectory(1).p = [];
trajectory(1).initu = [0.002;0.0015;0.0000;0.00];
trajectory(1).initTime = 0.8;

% rectangle
% trajectory(1).xf = [0; 1; 1; 0; 0; 0];
% trajectory(2).xf = [1; 1; 1; 0; 0; 0];
% trajectory(3).xf = [1; 0; 1; 0; 0; 0];
% trajectory(4).xf = [0; 0; 1; 0; 0; 0];
% 
% trajectory(1).p = [];
% trajectory(2).p = [];
% trajectory(3).p = [];
% trajectory(4).p = [];
% 
% trajectory(1).initu = [0.002; 0; 0; 0];
%   trajectory(1).initTime = 0.3;
% trajectory(2).initu = [0.002; 0; 0; 0];
%   trajectory(2).initTime = 0.3;
% trajectory(3).initu = [0.002; 0; 0; 0];
%   trajectory(3).initTime = 0.3;
% trajectory(4).initu = [0.002; 0; 0; 0];
%   trajectory(4).initTime = 0.3;  


% rectangle and flips

% trajectory(1).xf = [0; 1; 1; 0; 0; 0];
% trajectory(2).xf = [0.5; 0.5; NaN; pi*2; 0; 0];
% trajectory(3).xf = [1; 0; 1; 0; 0; 0];
% trajectory(4).xf = [1; 1; 1; 0; 0; 0];
% trajectory(5).xf = [0.5; 0.5; NaN; 0; pi*2; 0];
% trajectory(6).xf = [0; 0; 1; 0; 0; 0];
% 
% 
% trajectory(1).initu = [0.002; 0; 0; 0];
%   trajectory(1).initTime = 0.3;
% trajectory(2).initu = [0.002;0.001;0.001;0.000];
%   trajectory(2).initTime = 1.3;
% trajectory(3).initu = [0.002; 0; 0; 0];
%   trajectory(3).initTime = 0.3;
% trajectory(4).initu = [0.002; 0; 0; 0];
%   trajectory(4).initTime = 0.3;
% trajectory(5).initu = [0.002;0.001;0.001;0.000];
%   trajectory(5).initTime = 0.6;  
% trajectory(6).initu = [0.002; 0; 0; 0];
%   trajectory(6).initTime = 0.3;
% 
% trajectory(1).p = [];
% trajectory(2).p = [];%[1;0;2];
% trajectory(3).p = [];
% trajectory(4).p = [];%[1;0;2];


u0 = [0;0;0;0];
numberOftrajectory = length(trajectory);

save x0 x0
save u0 u0

xplot = [];
uplot = [];
tplot = 0;

%%  definition of trajectory

for i=1:numberOftrajectory-1
   
    [trajectory(i).xplot,trajectory(i).uplot,trajectory(i).tplot,trajectory(i).p]  = trajectory_generation(trajectory(i),true);
    xplot = [xplot;trajectory(i).xplot];
    uplot = [uplot;trajectory(i).uplot];
    tplot = [tplot;tplot(end)+trajectory(i).tplot];
end    

[trajectory(numberOftrajectory).xplot,trajectory(numberOftrajectory).uplot, trajectory(numberOftrajectory).tplot, ...
    trajectory(numberOftrajectory).p] = trajectory_generation(trajectory(numberOftrajectory),true);
    
    xplot = [xplot;trajectory(numberOftrajectory).xplot];
    uplot = [uplot;trajectory(numberOftrajectory).uplot];
    tplot = [tplot;tplot(end)+trajectory(numberOftrajectory).tplot];
    tplot = tplot(2:end);

save(traj_name,'tplot','uplot','xplot')

%% Visualize results

generate_acceleration;
flag=false;
visualize_result;

%% compute gains 
K = computeLQRGains([theta; phi; psy],omega,f1);

%% Write trajectory and Gains to file

A = [x' y' z' theta' phi' psy' xd' yd' zd' omega' xdd' ydd' zdd' omegad' jerk'];
address=pwd;
ind=find(address=='/');
writematrix(A,strcat(address(1:ind(end-2)),'ROS/src/rotors_gazebo/src/traj.txt'),'Delimiter','tab');
writematrix(K,strcat(address(1:ind(end-2)),'ROS/src/rotors_gazebo/src/Gains.txt'),'Delimiter','tab');