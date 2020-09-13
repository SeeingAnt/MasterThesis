r = 0.02;
l = 0.046;

x0=10;
y0=10;
width=1250;
height=800;

%%
figure;
scatter3(x(1),y(1),z(1),'g')
hold on; text(x(1),y(1),z(1),'\leftarrow start point','FontSize',14)
scatter3(x(end),y(end),z(end),'r')
text(x(end),y(end),z(end),'\leftarrow end point','FontSize',14)
plot3(x,y,z)

for i = 1:20:length(y)
    
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
       

   
end
xlabel('x (m)');
ylabel('y (m)');
zlabel('z (m)');
plot3(x+position_std(1,:),y+position_std(2,:),z+position_std(3,:),'--','Color','b');
plot3(x-position_std(1,:),y-position_std(2,:),z-position_std(3,:),'--','Color','g');
title('3D trajectory'),daspect([1 1 1]), grid on;

%% plot states
if flag==true
    figure;
    plot(time,x); hold on;
    plot(time,x+position_std(1,:),'--');
    plot(time,x-position_std(1,:),'--');
    xlabel('time (s)','FontSize',20);
    ylabel('x (m)','FontSize',20);
    legend('x','$x+\sigma$','$x-\sigma$','Interpreter','latex','FontSize',15)
    set(gcf,'position',[x0,y0,width,height])
    grid on;
    saveas(gcf,'figures/x.eps')


    figure;
    plot(time,y);hold on;
    plot(time,y+position_std(2,:),'--');
    plot(time,y-position_std(2,:),'--');
    xlabel('time (s)','FontSize',20);
    ylabel('y (m)','FontSize',20);
    legend('y','$y+\sigma$','$y-\sigma$','Interpreter','latex','FontSize',15)
    set(gcf,'position',[x0,y0,width,height])
    grid on;
    saveas(gcf,'figures/y.eps')

    figure;
    plot(time,z);hold on;
    plot(time,z+position_std(3,:),'--');
    plot(time,z-position_std(3,:),'--');
    xlabel('time (s)','FontSize',20);
    ylabel('z (m)','FontSize',20);
    legend('z','$z+\sigma$','$z-\sigma$','Interpreter','latex','FontSize',15)
    set(gcf,'position',[x0,y0,width,height])
    grid on;
    saveas(gcf,'figures/z.eps')

    figure;
    plot(time,xd);hold on;
    plot(time,xd+velocity_std(1,:),'--');
    plot(time,xd-velocity_std(1,:),'--');
    xlabel('time (s)','FontSize',20);
    ylabel('$\dot{x} (\frac{m}{s})$','Interpreter','latex','FontSize',20);
    legend('$\dot{x}$','$\dot{x}+\sigma$','$\dot{x}-\sigma$','Interpreter','latex','FontSize',15)
    set(gcf,'position',[x0,y0,width,height])
    grid on;
    saveas(gcf,'figures/xd.eps')

    figure;
    plot(time,yd);hold on;
    plot(time,yd+velocity_std(2,:),'--');
    plot(time,yd-velocity_std(2,:),'--');
    xlabel('time (s)','FontSize',20);
    ylabel('$\dot{y} (\frac{m}{s})$','Interpreter','latex','FontSize',20);
    legend('$\dot{y}$','$\dot{y}+\sigma$','$\dot{y}-\sigma$','Interpreter','latex','FontSize',15)
    set(gcf,'position',[x0,y0,width,height])
    grid on;
    saveas(gcf,'figures/yd.eps')

    figure;
    plot(time,zd);hold on;
    plot(time,zd+velocity_std(3,:),'--');
    plot(time,zd-velocity_std(3,:),'--');
    xlabel('time (s)','FontSize',20);
    ylabel('$\dot{z} (\frac{m}{s})$','Interpreter','latex','FontSize',20);
    legend('$\dot{z}$','$\dot{z}+\sigma$','$\dot{z}-\sigma$','Interpreter','latex','FontSize',15)
    set(gcf,'position',[x0,y0,width,height])
    grid on;
    saveas(gcf,'figures/zd.eps')

    figure;
    plot(time,xdd);hold on;
    plot(time,acc_p_std(1,:),'--');
    plot(time,acc_m_std(1,:),'--');
    xlabel('time (s)','FontSize',20);
    ylabel('$\ddot{x} (\frac{m}{s^2})$','Interpreter','latex','FontSize',20);
    legend('$\ddot{x}$','$\ddot{x}+\sigma$','$\ddot{x}-\sigma$','Interpreter','latex','FontSize',15)
    set(gcf,'position',[x0,y0,width,height])
    grid on;
    saveas(gcf,'figures/xdd.eps')

    figure;
    plot(time,ydd);hold on;
    plot(time,acc_p_std(2,:),'--');
    plot(time,acc_m_std(2,:),'--');
    xlabel('time (s)','FontSize',20);
    ylabel('$\ddot{y} (\frac{m}{s^2})$','Interpreter','latex','FontSize',20);
    legend('$\ddot{y}$','$\ddot{y}+\sigma$','$\ddot{y}-\sigma$','Interpreter','latex','FontSize',15)
    set(gcf,'position',[x0,y0,width,height])
    grid on;
    saveas(gcf,'figures/ydd.eps')

    figure;
    plot(time,zdd);hold on;
    plot(time,acc_p_std(3,:),'--');
    plot(time,acc_m_std(3,:),'--');
    xlabel('time (s)','FontSize',20);
    ylabel('$\ddot{z} (\frac{m}{s^2})$','Interpreter','latex','FontSize',20);
    legend('$\ddot{z}$','$\ddot{z}+\sigma$','$\ddot{z}-\sigma$','Interpreter','latex','FontSize',15)
    set(gcf,'position',[x0,y0,width,height])
    grid on;
    saveas(gcf,'figures/zdd.eps')

    figure;
    plot(time,jerk(1,:));hold on;
    plot(time,jerk_p_std(1,:),'--');
    plot(time,jerk_m_std(1,:),'--');
    xlabel('time (s)','FontSize',20);
    ylabel('$x^{(3)} (\frac{m}{s^3})$','Interpreter','latex','FontSize',20);
    legend('$x^{(3)}$','$x^{(3)}+\sigma$','$x^{(3)}-\sigma$','Interpreter','latex','FontSize',15)
    set(gcf,'position',[x0,y0,width,height])
    grid on;
    saveas(gcf,'figures/xdddd.eps')

    figure;
    plot(time,jerk(2,:));hold on;
    plot(time,jerk_p_std(2,:),'--');
    plot(time,jerk_m_std(2,:),'--');
    xlabel('time (s)','FontSize',20);
    ylabel('$y^{(3)} (\frac{m}{s^3})$','Interpreter','latex','FontSize',20);
    legend('$y^{(3)}$','$y^{(3)}+\sigma$','$y^{(3)}-\sigma$','Interpreter','latex','FontSize',15)
    set(gcf,'position',[x0,y0,width,height])
    grid on;
    saveas(gcf,'figures/yddd.eps')

    figure;
    plot(time,jerk(3,:));hold on;
    plot(time,jerk_p_std(3,:),'--');
    plot(time,jerk_m_std(3,:),'--');
    xlabel('time (s)','FontSize',20);
    ylabel('$z^{(3)} (\frac{m}{s^3})$','Interpreter','latex','FontSize',20);
    legend('$z^{(3)}$','$z^{(3)}+\sigma$','$z^{(3)}-\sigma$','Interpreter','latex','FontSize',15)
    set(gcf,'position',[x0,y0,width,height])
    grid on;
    saveas(gcf,'figures/zddd.eps')


    figure;
    plot(time,theta);hold on;
    plot(time,theta+theta_std,'--');
    plot(time,theta-theta_std,'--');
    xlabel('time (s)','FontSize',20);
    ylabel('$\theta$ (rad)','Interpreter','latex','FontSize',20);
    legend('$\theta$','$\theta+\sigma$','$\theta-\sigma$','Interpreter','latex','FontSize',15)
    set(gcf,'position',[x0,y0,width,height])
    grid on;
    saveas(gcf,'figures/theta.eps')

    figure;
    plot(time,phi);hold on;
    plot(time,phi+phi_std,'--');
    plot(time,phi-phi_std,'--');
    xlabel('time (s)','FontSize',20);
    ylabel('$\phi$ (rad)','Interpreter','latex','FontSize',20);
    legend('$\phi$','$\phi+\sigma$','$\phi-\sigma$','Interpreter','latex','FontSize',15)
    set(gcf,'position',[x0,y0,width,height])
    grid on;
    saveas(gcf,'figures/phi.eps')

    figure;
    plot(time,psy);hold on;
    plot(time,psy+psy_std,'--');
    plot(time,psy-psy_std,'--');
    xlabel('time (s)','FontSize',20);
    ylabel('$\psi$ (rad)','Interpreter','latex','FontSize',20);
    legend('$\psi$','$\psi+\sigma$','$\psi-\sigma$','Interpreter','latex','FontSize',15)
    set(gcf,'position',[x0,y0,width,height])
    grid on;
    saveas(gcf,'figures/psi.eps')

    figure;
    plot(time,omega(1,:));hold on;
    plot(time,omega(1,:)+omega_std(1,:),'--');
    plot(time,omega(1,:)-omega_std(1,:),'--');
    xlabel('time (s)','FontSize',20);
    ylabel('$\omega_1 (\frac{rad}{s})$','Interpreter','latex','FontSize',20);
    legend('$\omega_1$','$\omega_1+\sigma$','$\omega_1-\sigma$','Interpreter','latex','FontSize',15)
    set(gcf,'position',[x0,y0,width,height])
    grid on;
    saveas(gcf,'figures/om1.eps')

    figure;
    plot(time,omega(2,:));hold on;
    plot(time,omega(2,:)+omega_std(2,:),'--');
    plot(time,omega(2,:)-omega_std(2,:),'--');
    xlabel('time (s)','FontSize',20);
    ylabel('$\omega_2 (\frac{rad}{s})$','Interpreter','latex','FontSize',20);
    legend('$\omega_2$','$\omega_2+\sigma$','$\omega_2-\sigma$','Interpreter','latex','FontSize',15)
    set(gcf,'position',[x0,y0,width,height])
    grid on;
    saveas(gcf,'figures/om2.eps')

    figure;
    plot(time,omega(3,:));hold on;
    plot(time,omega(3,:)+omega_std(3,:),'--');
    plot(time,omega(3,:)-omega_std(3,:),'--');
    xlabel('time (s)','FontSize',20);
    ylabel('$\omega_3 (\frac{rad}{s})$','Interpreter','latex','FontSize',20);
    legend('$\omega_3$','$\omega_3+\sigma$','$\omega_3-\sigma$','Interpreter','latex','FontSize',15)
    set(gcf,'position',[x0,y0,width,height])
    grid on;
    saveas(gcf,'figures/om3.eps')

    figure;
    plot(time,omegad(1,:));hold on;
    plot(time,omegad_p_std(1,:),'--');
    plot(time,omegad_m_std(1,:),'--');
    xlabel('time (s)','FontSize',20);
    ylabel('$\dot{\omega}_1 (\frac{rad}{s^2})$','Interpreter','latex','FontSize',20);
    legend('$\dot{\omega}_1$','$\dot{\omega}_1+\sigma$','$\dot{\omega}_1-\sigma$','Interpreter','latex','FontSize',15)
    grid on;
    set(gcf,'position',[x0,y0,width,height])
    saveas(gcf,'figures/dom1.eps')


    figure;
    plot(time,omegad(2,:));hold on;
    plot(time,omegad_p_std(2,:),'--');
    plot(time,omegad_m_std(2,:),'--');
    xlabel('time (s)','FontSize',20);
    ylabel('$\dot{\omega}_2 (\frac{rad}{s^2})$','Interpreter','latex','FontSize',20);
    legend('$\dot{\omega}_2$','$\dot{\omega}_2+\sigma$','$\dot{\omega}_2-\sigma$','Interpreter','latex','FontSize',15)
    set(gcf,'position',[x0,y0,width,height])
    grid on;
    saveas(gcf,'figures/dom2.eps')
    grid on;

    figure;
    plot(time,omegad(3,:));hold on;
    plot(time,omegad_p_std(3,:),'--');
    plot(time,omegad_m_std(3,:),'--');
    xlabel('time (s)','FontSize',20);
    ylabel('$\dot{\omega}_3 (\frac{rad}{s^2})$','Interpreter','latex','FontSize',20);
    legend('$\dot{\omega}_3$','$\dot{\omega}_3+\sigma$','$\dot{\omega}_3-\sigma$','Interpreter','latex','FontSize',15)
    set(gcf,'position',[x0,y0,width,height])
    grid on;
    saveas(gcf,'figures/dom3.eps')
end