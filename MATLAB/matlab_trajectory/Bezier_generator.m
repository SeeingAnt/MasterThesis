clear
close all

n=7;

st = 0;
ft = 2; 
f=1/2;

[x,y,z,xd,yd,zd,xdd,ydd,zdd,yaw,yawd] = build_sin(st,ft,n,f);

st = 2;
ft = 4; 
f=1/2;

[x1,y1,z1,xd1,yd1,zd1,xdd1,ydd1,zdd1,yaw1,yawd1] = build_sin(st,ft,n,f);

x=[x,x1];
y=[y,y1];
z=[z,z1];
xd=[xd,xd1];
yd=[yd,yd1];
zd=[zd,zd1];
xdd=[xdd,xdd1];
ydd=[ydd,ydd1];
zdd=[zdd,zdd1];
yaw=[yaw,yaw1];
yawd=[yawd,yawd1];

figure
plot(x,y);
figure
plot(xd,yd);
figure
plot(xdd,ydd);

A = [x' y' z' yaw' xd' yd' zd' yawd' xdd' ydd' zdd'];
address=pwd;
ind=find(address=='/');
writematrix(A,strcat(address(1:ind(end)),'ROS/src/rotors_gazebo/src/prova.txt'),'Delimiter','tab');


