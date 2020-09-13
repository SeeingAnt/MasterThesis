tc = 0:par.step:t1;
tf = (t1+par.step):par.step:(t1+t2);
tr = (t1+t2+par.step):par.step:(t1+t2+t3);

%% command trajectory
y1 = y(1,1:size(z1,2));
y2 = y(1,size(z1,2)+1:size(z1,2)+size(z2,2));
y3 = y(1,size(z1,2)+size(z2,2)+1:size(z1,2)+size(z2,2)+size(z3,2));

r = 0.01;
l = 0.046;

disp('The error on the criterion is: ');
err = norm(ydd.*cos(theta)+(zdd+gravity).*sin(theta));
disp(err);

figure, subplot(2,2,1), plot(tc,y1,'LineWidth',1.5,'Color','b'),
hold on, plot(tf,y2,'LineWidth',1.5,'Color','r'), 
hold on, plot(tr,y3,'LineWidth',1.5,'Color','y'),
hold on, plot(0,0,'*g'), hold on, plot(t1,y1(length(y1)),'*g'),
hold on, plot(t1+t2,y2(length(y2)),'*g'),
hold on, plot(t1+t2+t3,y3(length(y3)),'*g'),
title('Position y(t)'), grid,
subplot(2,2,2), plot(tc,z1(1,:),'LineWidth',1.5,'Color','b'),
hold on, plot(tf,z2,'LineWidth',1.5,'Color','r'), 
hold on, plot(tr,z3(1,:),'LineWidth',1.5,'Color','y'),
hold on, plot(0,z_hover1,'*g'), hold on, plot(t1,z_start,'*g'),
hold on, plot(t1+t2,z_end,'*g'),
hold on, plot(t1+t2+t3,z_hover2,'*g'),
title('Position z(t)'), grid,
subplot(2,2,3), plot(tc,theta1(1,:),'LineWidth',1.5,'Color','b'),
hold on, plot(tf,theta2,'LineWidth',1.5,'Color','r'), 
hold on, plot(tr,theta3(1,:),'LineWidth',1.5,'Color','y'), 
hold on, plot(0,0,'*g'), hold on, plot(t1,theta_start,'*g'),
hold on, plot(t1+t2,theta_end,'*g'),
hold on, plot(t1+t2+t3,2*par.flips*pi,'*g'),
title('Angular position \theta(t)'), grid,
%subplot(2,2,4), 
figure, plot(y1,z1(1,:),'LineWidth',1.5,'Color','b'), 
hold on, plot(y2,z2(1,:),'LineWidth',1.5,'Color','r'),
hold on, plot(y3,z3(1,:),'LineWidth',1.5,'Color','y'), 
hold on, plot(y1(1),z_hover1,'*g'), hold on, plot(y1(length(y1)),z_start,'*g'),
hold on, plot(y2(length(y2)),z_end,'*g'),
hold on, plot(y3(length(y3)),z_hover2,'*g'),
for i = 1:20:length(y)
    
   t = theta(i);
   
   q1 = y(i)-l*cos(t);
   q2 = y(i)+l*cos(t);
   w1 = z(i)-l*sin(t);
   w2 = z(i)+l*sin(t);
   
   r1y = q1+r*cos(t+pi/2);
   r1z = w1+r*sin(t+pi/2);
   r2y = q2+r*cos(t+pi/2);
   r2z = w2+r*sin(t+pi/2);
   
   plot([q1 q2],[w1 w2],'k','LineWidth',1.5), hold on 
   plot([q1 r1y],[w1 r1z],'k','LineWidth',1.5), hold on
   plot([q2 r2y],[w2 r2z],'k','LineWidth',1.5), hold on
   
end
title('Planar flipping trajectory'), daspect([1 1 1]), grid;

%% true trajectory

% figure, hold on, plot(y_new,z_new);
% for i = 1:20:length(y_new)
%     
%    t = theta_new(i);
%    
%    q1 = y_new(i)-l*cos(t);
%    q2 = y_new(i)+l*cos(t);
%    w1 = z_new(i)-l*sin(t);
%    w2 = z_new(i)+l*sin(t);
%    
%    r1y = q1+r*cos(t+pi/2);
%    r1z = w1+r*sin(t+pi/2);
%    r2y = q2+r*cos(t+pi/2);
%    r2z = w2+r*sin(t+pi/2);
%    
%    plot([q1 q2],[w1 w2],'k','LineWidth',1.5), hold on 
%    plot([q1 r1y],[w1 r1z],'k','LineWidth',1.5), hold on
%    plot([q2 r2y],[w2 r2z],'k','LineWidth',1.5), hold on
%    
% end
% title('True Planar flipping trajectory'), daspect([1 1 1]), grid;
