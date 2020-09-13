%close all;
clc

load orientation.csv
load orientation1.csv
load orientation2.csv
load orientation3.csv
load rotation_des.csv
load rotation_des1.csv
load rotation_des2.csv
load rotation_des3.csv

figure

plot(rotation_des(:,1)/1e9,rotation_des(:,2)); hold on
plot(orientation(:,1)/1e9,orientation(:,2)); hold on

legend('$qx_{des}$','$qx_{real}$','Interpreter','latex','FontSize',13)
% xline(8.215, '-.k', 'Linewidth', 1.0, 'DisplayName', ['Start' 10 'Aggressive'], 'FontSize', 12)
% xline(8.415, '-.g', 'Linewidth', 1.0, 'DisplayName', ['End' 10 'Aggressive'], 'FontSize', 12)

xlabel("time(s)")
ylabel("qx")

xlim([5 10]);
ylim([-31.2 31.2]);
grid on

figure
plot(rotation_des(:,1)/1e9,rotation_des1(:,2)); hold on
plot(orientation(:,1)/1e9,orientation1(:,2)); hold on

legend('$qy_{des}$','$qy_{real}$','Interpreter','latex','FontSize',13)
% xline(8.215, '-.k', 'Linewidth', 1.0, 'DisplayName', ['Start' 10 'Aggressive'], 'FontSize', 12)
% xline(8.415, '-.g', 'Linewidth', 1.0, 'DisplayName', ['End' 10 'Aggressive'], 'FontSize', 12)

xlabel("time(s)")
ylabel("qy")

xlim([5 10]);
ylim([-1.2 1.2]);
grid on

figure
plot(rotation_des(:,1)/1e9,rotation_des2(:,2)); hold on
plot(orientation(:,1)/1e9,orientation2(:,2)); hold on

legend('$qz_{des}$','$qz_{real}$','Interpreter','latex','FontSize',13)
% xline(8.215, '-.k', 'Linewidth', 1.0, 'DisplayName', ['Start' 10 'Aggressive'], 'FontSize', 12)
% xline(8.415, '-.g', 'Linewidth', 1.0, 'DisplayName', ['End' 10 'Aggressive'], 'FontSize', 12)

xlabel("time(s)")
ylabel("qz")

xlim([5 10]);
ylim([-1.2 1.2]);
grid on

figure
plot(rotation_des(:,1)/1e9,rotation_des3(:,2)); hold on
plot(orientation(:,1)/1e9,orientation3(:,2)); hold on

legend('$qw_{des}$','$qw_{real}$','Interpreter','latex','FontSize',13)
% xline(8.215, '-.k', 'Linewidth', 1.0, 'DisplayName', ['Start' 10 'Aggressive'], 'FontSize', 12)
% xline(8.465, '-.g', 'Linewidth', 1.0, 'DisplayName', ['End' 10 'Aggressive'], 'FontSize', 12)

xlabel("time(s)")
ylabel("qw")

xlim([5 10]);
ylim([-1.2 1.2]);
grid on
%% angular rotation
rostopic echo /crazyflie2/command/trajectory/points[0]/transforms[0]/rotation/x -b rotation_des.bag -p > rotation_des.csv
rostopic echo gazebo/model_states/pose[1]/orientation/x -b orientation.bag -p > orientation.csv
rostopic echo /crazyflie2/command/trajectory/points[0]/transforms[0]/rotation/y -b rotation_des.bag -p > rotation_des1.csv
rostopic echo gazebo/model_states/pose[1]/orientation/y -b orientation.bag -p > orientation1.csv
rostopic echo /crazyflie2/command/trajectory/points[0]/transforms[0]/rotation/z -b rotation_des.bag -p > rotation_des2.csv
rostopic echo gazebo/model_states/pose[1]/orientation/z -b orientation.bag -p > orientation2.csv
rostopic echo /crazyflie2/command/trajectory/points[0]/transforms[0]/rotation/w -b rotation_des.bag -p > rotation_des3.csv
rostopic echo gazebo/model_states/pose[1]/orientation/w -b orientation.bag -p > orientation3.csv
%% velocity
 rostopic echo /crazyflie2/command/trajectory/points[0]/velocities[0]/angular/x -b rotation_des.bag -p > rotation_des.csv
 rostopic echo gazebo/model_states/twist[1]/angular/x -b orientation.bag -p > orientation.csv
%% motors
 rostopic echo /crazyflie2/motor_speed/angular_velocities[0] -b motors.bag -p > motor1.csv
 rostopic echo /crazyflie2/motor_speed/angular_velocities[1] -b motors.bag -p > motor2.csv
 rostopic echo /crazyflie2/motor_speed/angular_velocities[2] -b motors.bag -p > motor3.csv
 rostopic echo /crazyflie2/motor_speed/angular_velocities[3] -b motors.bag -p > motor4.csv
%% 

 load motor1.csv
 load motor2.csv
 load motor3.csv
 load motor4.csv

 figure
 plot(motor1(:,1)/1e9,motor1(:,2)); hold on
 
 figure
 plot(motor2(:,1)/1e9,motor2(:,2)); hold on
 
 figure
 plot(motor3(:,1)/1e9,motor3(:,2)); hold on
 
 figure
 plot(motor4(:,1)/1e9,motor4(:,2)); hold on
 
 
%%
load orientation.csv
load orientation1.csv
load orientation2.csv
load orientation3.csv

figure(5)

plot(orientation(:,1)/1e9,orientation(:,2)); hold on

xlabel("time")
ylabel("qx")

xlim([5 10]);
ylim([-1.2 1.2]);
grid on


figure(6)
plot(orientation(:,1)/1e9,orientation1(:,2)); hold on

xlabel("time")
ylabel("qy")

xlim([5 10]);
ylim([-1.2 1.2]);
grid on

figure(7)
plot(orientation(:,1)/1e9,orientation2(:,2)); hold on

xlabel("time")
ylabel("qz")

xlim([5 10]);
ylim([-1.2 1.2]);
grid on

figure(8)
plot(orientation(:,1)/1e9,orientation3(:,2)); hold on

xlabel("time(s)")
ylabel("qw")

xlim([5 10]);
ylim([-1.2 1.2]);
grid on
%%

% close all;
clc

load x_des.csv x_des
load y_des.csv y_des
load z_des.csv z_des
load x_real.csv x_real
load y_real.csv y_real
load z_real.csv z_real

figure(1);
plot3(x_des(:,2),y_des(:,2),z_des(:,2)); hold on
plot3(x_real(:,2),y_real(:,2),z_real(:,2));
xlabel("x(m)");
ylabel("y(m)");
zlabel("z(m)");

daspect([1 1 1]), grid on;

figure(2);
plot(x_des(:,1)/1e9,x_des(:,2)); hold on
plot(x_real(:,1)/1e9,x_real(:,2));
xlabel("time(s)");
ylabel("x(m)");
legend('$x_{des}$','$x_{real}$','Interpreter','latex','FontSize',13)
% xline(8.215, '-.k', 'Linewidth', 1.0, 'DisplayName', ['Start' 10 'Aggressive'], 'FontSize', 12)
% xline(8.415, '-.g', 'Linewidth', 1.0, 'DisplayName', ['End' 10 'Aggressive'], 'FontSize', 12)

xlim([5 10]);
ylim([-0.2 0.2]);
grid on

figure(3);
plot(y_des(:,1)/1e9,y_des(:,2)); hold on
plot(y_real(:,1)/1e9,y_real(:,2));
xlabel("time(s)");
ylabel("y(m)");
legend('$y_{des}$','$y_{real}$','Interpreter','latex','FontSize',13)
% xline(8.215, '-.k', 'Linewidth', 1.0, 'DisplayName', ['Start' 10 'Aggressive'], 'FontSize', 12)
% xline(8.415, '-.g', 'Linewidth', 1.0, 'DisplayName', ['End' 10 'Aggressive'], 'FontSize', 12)

xlim([5 10]);
ylim([-0.2 0.2]);
grid on

figure(4);
plot(z_des(:,1)/1e9,z_des(:,2)); hold on
plot(z_real(:,1)/1e9,z_real(:,2));
xlabel("time(s)");
ylabel("z(m)");
legend('$z_{des}$','$z_{real}$','Interpreter','latex','FontSize',13)
% xline(8.215, '-.k', 'Linewidth', 1.0, 'DisplayName', ['Start' 10 'Aggressive'], 'FontSize', 12)
% xline(8.415, '-.g', 'Linewidth', 1.0, 'DisplayName', ['End' 10 'Aggressive'], 'FontSize', 12)

xlim([5 10]);
ylim([0.2 2.5]);
grid on
%% 
    rostopic echo /crazyflie2/command/trajectory/points[0]/transforms[0]/translation/x -b rotation_des.bag -p > x_des.csv
    rostopic echo /crazyflie2/command/trajectory/points[0]/transforms[0]/translation/y -b rotation_des.bag -p > y_des.csv
    rostopic echo /crazyflie2/command/trajectory/points[0]/transforms[0]/translation/z -b rotation_des.bag -p > z_des.csv
    rostopic echo gazebo/model_states/pose[1]/position/x -b orientation.bag -p > x_real.csv
    rostopic echo gazebo/model_states/pose[1]/position/y -b orientation.bag -p > y_real.csv
    rostopic echo gazebo/model_states/pose[1]/position/z -b orientation.bag -p > z_real.csv
%%

load x_real.csv x_real
load y_real.csv y_real
load z_real.csv z_real

figure(1);
plot3(x_real(:,2),y_real(:,2),z_real(:,2));

daspect([1 1 1]), grid on;

figure(2);
plot(x_real(:,1)/1e9,x_real(:,2));
xlabel("time");
ylabel("x");

xlim([5 10]);
ylim([-0.2 0.2]);
grid on

figure(3);
plot(y_real(:,1)/1e9,y_real(:,2));
xlabel("time");
ylabel("y");

xlim([5 10]);
ylim([-0.2 0.2]);
grid on

figure(4);
plot(z_real(:,1)/1e9,z_real(:,2));
xlabel("time");
ylabel("z");

xlim([5 10]);
ylim([0.2 2.5]);
grid on

