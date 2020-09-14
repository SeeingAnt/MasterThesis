%% Reaching phase

z1_start = [z_hover1 0 0 0 0];
z1_end = [z_start ((z_start-z_end)/t2 + par.g*t2/2) -par.g 0 0];
z1 = trajectory(z1_start,z1_end,t1,true,par.step);

theta1_start = [0 0 0 0 0];
theta1_end = [theta_start (theta_end-theta_start)/t2 0 0 0];
theta1 = trajectory(theta1_start,theta1_end,t1,true,par.step);

%% Flip phase

coeff_z2 = [-par.g/2 ((z_end-z_start)/t2+par.g*t2/2) z_start];
coeff_zd2 = polyder(coeff_z2);
coeff_zdd2 = polyder(coeff_zd2);
z2 = polyval(coeff_z2,par.step:par.step:t2);
zd2 = polyval(coeff_zd2,par.step:par.step:t2);
zdd2 = polyval(coeff_zdd2,par.step:par.step:t2);

coeff_theta2 = [(theta_end-theta_start)/t2 theta_start];
coeff_thetad2 = polyder(coeff_theta2);
coeff_thetadd2 = polyder(coeff_thetad2);
theta2 = polyval(coeff_theta2,par.step:par.step:t2);
thetad2 = polyval(coeff_thetad2,par.step:par.step:t2);
thetadd2 = polyval(coeff_thetadd2,par.step:par.step:t2);


%% Recovery phase

z3_start = [z_end ((z_start-z_end)/t2-par.g*t2/2) -par.g 0 0];
z3_end = [z_hover2 0 0 0 0];
z3 = trajectory(z3_start,z3_end,t3,false,par.step);

theta3_start = [theta_end (theta_end-theta_start)/t2 0 0 0];
theta3_end = [2*par.flips*pi 0 0 0 0];
theta3 = trajectory(theta3_start,theta3_end,t3,false,par.step);

%% Global trajectory

z = [z1(1,:) z2 z3(1,:)];
zd = [z1(2,:) zd2 z3(2,:)];
zdd = [z1(3,:) zdd2 z3(3,:)];

theta = [theta1(1,:) theta2 theta3(1,:)];
thetad = [theta1(2,:) thetad2 theta3(2,:)];
thetadd = [theta1(3,:) thetadd2 theta3(3,:)];

gravity = par.g*ones(size(z));

ydd = -tan(theta).*(zdd + gravity);
yd = integrate(ydd,0,t1+t2+t3);
y = integrate(yd,optimout.p(1),t1+t2+t3);

    