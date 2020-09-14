
r = 0.02;
l = 0.046;

%%
figure;
scatter3(x(1),y(1),z(1),'g')
hold on; text(x(1),y(1),z(1),'\leftarrow start point','FontSize',14)
hold on; scatter3(x(end),y(end),z(end),'r')
text(x(end)+0.2,y(end),z(end),' end point','FontSize',14)
plot3(x,y,z)

for i = 1:40:length(y)
    
       cy = cos(psy(i)/2);
       sy = sin(psy(i)/2);
       cp = cos(phi(i)/2);
       sp = sin(phi(i)/2);
       cr = cos(theta(i)/2);
       sr = sin(theta(i)/2);      
       
       qw = cr * cp * cy + sr * sp * sy;
       qx = sr * cp * cy - cr * sp * sy;
       qy = cr * sp * cy + sr * cp * sy;
       qz = cr * cp * sy - sr * sp * cy;

       R = [1-2*qy^2-2*qz^2    2*qx*qy-2*qz*qw     2*qx*qz+2*qy*qw;
            2*qx*qy+2*qz*qw 	1-2*qx^2-2*qz^2 	2*qy*qz-2*qx*qw;
            2*qx*qz-2*qy*qw 	2*qy*qz+2*qx*qw 	1-2*qx^2-2*qy^2];


       c1 = [x(i);y(i);z(i)] + R * [r;0;0];
       q1 = [x(i);y(i);z(i)] + R * [l;l;0];
       q2 = [x(i);y(i);z(i)] + R * [-l;-l;0];
       q3 = [x(i);y(i);z(i)] + R * [l;-l;0];
       q4 = [x(i);y(i);z(i)] + R * [-l;l;0];


       r1 = q1 + R * [0;0;r];
       r2 = q2 + R * [0;0;r];
       r3 = q3 + R * [0;0;r];
       r4 = q4 + R * [0;0;r];

       plot3([q1(1) q2(1)],[q1(2) q2(2)],[q1(3) q2(3)] ,'k','LineWidth',1.5), hold on 
       plot3([q3(1) q4(1)],[q3(2) q4(2)],[q3(3) q4(3)] ,'k','LineWidth',1.5), hold on 
       plot3([q1(1) r1(1)],[q1(2) r1(2)],[q1(3) r1(3)], 'k','LineWidth',1.5), hold on
       plot3([q2(1) r2(1)],[q2(2) r2(2)],[q2(3) r2(3)], 'k','LineWidth',1.5), hold on
       plot3([q3(1) r3(1)],[q3(2) r3(2)],[q3(3) r3(3)], 'k','LineWidth',1.5), hold on
       plot3([q4(1) r4(1)],[q4(2) r4(2)],[q4(3) r4(3)], 'k','LineWidth',1.5), hold on
       plot3([x(i) c1(1)],[y(i) c1(2)],[z(i) c1(3)], 'r','LineWidth',1.5), hold on
       
       xlabel('x(m)');
       ylabel('y(m)');
       zlabel('z(m)');
   
end
title('3D trajectory'),daspect([1 1 1]), grid on;


%% plot states

if(flag)

figure;
plot(time,x); hold on;
xlabel('time (s)');
ylabel('x (m)');
grid on;
saveas(gcf,'figures/x.eps')



figure;
plot(time,y);hold on;
xlabel('time (s)');
ylabel('y (m)');
grid on;
saveas(gcf,'figures/y.eps')


figure;
plot(time,z);hold on;
xlabel('time (s)');
ylabel('z (m)');
grid on;
saveas(gcf,'figures/z.eps')


figure;
plot(time,xd);hold on;
xlabel('time (s)');
ylabel('$\dot{x} (\frac{m}{s})$','Interpreter','latex','FontSize',15);
grid on;
saveas(gcf,'figures/xd.eps')


figure;
plot(time,yd);hold on;
xlabel('time (s)');
ylabel('$\dot{y} (\frac{m}{s})$','Interpreter','latex','FontSize',15);
grid on;
saveas(gcf,'figures/yd.eps')


figure;
plot(time,zd);hold on;
xlabel('time (s)');
ylabel('$\dot{z} (\frac{m}{s})$','Interpreter','latex','FontSize',15);
grid on;
saveas(gcf,'figures/zd.eps')


figure;
plot(time,xdd);hold on;
xlabel('time (s)');
ylabel('$\ddot{x} (\frac{m}{s^2})$','Interpreter','latex','FontSize',15);
grid on;
saveas(gcf,'figures/xdd.eps')


figure;
plot(time,ydd);hold on;
xlabel('time (s)');
ylabel('$\ddot{y} (\frac{m}{s^2})$','Interpreter','latex','FontSize',15);
grid on;
saveas(gcf,'figures/ydd.eps')

figure;
plot(time,zdd);hold on;
xlabel('time (s)');
ylabel('$\ddot{z} (\frac{m}{s^2})$','Interpreter','latex','FontSize',15);
grid on;
saveas(gcf,'figures/zdd.eps')


figure;
plot(time,jerk(1,:));hold on;
xlabel('time (s)');
ylabel('$x^{(3)} (\frac{m}{s^3})$','Interpreter','latex','FontSize',15);
grid on;
saveas(gcf,'figures/xdddd.eps')


figure;
plot(time,jerk(2,:));hold on;
xlabel('time (s)');
ylabel('$y^{(3)} (\frac{m}{s^3})$','Interpreter','latex','FontSize',15);
grid on;
saveas(gcf,'figures/yddd.eps')

figure;
plot(time,jerk(3,:));hold on;
xlabel('time (s)');
ylabel('$z^{(3)} (\frac{m}{s^3})$','Interpreter','latex','FontSize',15);
grid on;
saveas(gcf,'figures/zddd.eps')


figure;
plot(time,theta);hold on;
xlabel('time (s)');
ylabel('$\theta$ (rad)','Interpreter','latex','FontSize',15);
grid on;
saveas(gcf,'figures/theta.eps')


figure;
plot(time,phi);hold on;
xlabel('time (s)');
ylabel('$\phi$ (rad)','Interpreter','latex','FontSize',15);
grid on;
saveas(gcf,'figures/phi.eps')


figure;
plot(time,psy);hold on;
xlabel('time (s)');
ylabel('$\psi$ (rad)','Interpreter','latex','FontSize',15);
grid on;
saveas(gcf,'figures/psi.eps')


figure;
plot(time,omega(1,:));hold on;
xlabel('time (s)');
ylabel('$\omega_1 (\frac{rad}{s})$','Interpreter','latex','FontSize',15);
grid on;
saveas(gcf,'figures/om1.eps')


figure;
plot(time,omega(2,:));hold on;
xlabel('time (s)');
ylabel('$\omega_2 (\frac{rad}{s})$','Interpreter','latex','FontSize',15);
grid on;
saveas(gcf,'figures/om2.eps')


figure;
plot(time,omega(3,:));hold on;
xlabel('time (s)');
ylabel('$\omega_3 (\frac{rad}{s})$','Interpreter','latex','FontSize',15);
grid on;
saveas(gcf,'figures/om3.eps')


figure;
plot(time,omegad(1,:));hold on;
xlabel('time (s)');
ylabel('$\dot{\omega}_1 (\frac{rad}{s^2})$','Interpreter','latex','FontSize',15);
grid on;
saveas(gcf,'figures/dom1.eps')


figure;
plot(time,omegad(2,:));hold on;
xlabel('time (s)');
ylabel('$\dot{\omega}_2 (\frac{rad}{s^2})$','Interpreter','latex','FontSize',15);
grid on;
saveas(gcf,'figures/dom2.eps')


figure;
plot(time,omegad(3,:));hold on;
xlabel('time (s)');
ylabel('$\dot{\omega}_3 (\frac{rad}{s^2})$','Interpreter','latex','FontSize',15);
grid on;
saveas(gcf,'figures/dom3.eps')

end
