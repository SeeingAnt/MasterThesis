load(traj_name)

addpath('../tool')
par=InitParameter();

acc = [0;0;0];
omegad = [0;0;0];

%%

[u_resamp,ty]=resample(uplot,tplot');
[x_resamp,ty]=resample(xplot,tplot');

step = ty(end)/size(ty,2);
n = 0.002/step;
num = size(ty,2)/n;
index = linspace(1,size(ty,2),ceil(num));


time = ty(round(index)); 

u=[];
acc=[0;0;0];
jerk=[0;0;0];
f1 = [x_resamp(:,13)];
f2 = [x_resamp(:,15)];
f3 = [x_resamp(:,16)];
f4 = [x_resamp(:,17)];
omega = [x_resamp(:,10:12)'];
psy =  [x_resamp(:,9)'];
phi = [x_resamp(:,8)'];
theta = [x_resamp(:,7)'];
velocity = [x_resamp(:,4:6)'];
position = [x_resamp(:,1:3)'];

f = [0;0;0;0];

for i=1:length(ty)

           cy = cos(psy(i));
           sy = sin(psy(i));
           cp = cos(phi(i));
           sp = sin(phi(i));
           cr = cos(theta(i));
           sr = sin(theta(i));
          
          W = [   1     tan(phi(i))*sr    tan(phi(i))*cr; ...
                  0        cr                        -sr; ...
                  0        sr/cp                   cr/cp];
          
          R=eul2rotm([psy(i) phi(i) theta(i)]);    
          
          f(1,i)=par.Finv(1,:)*[f1(i);f2(i);f3(i);f4(i)];
          f(2,i)=par.Finv(2,:)*[f1(i);f2(i);f3(i);f4(i)];
          f(3,i)=par.Finv(3,:)*[f1(i);f2(i);f3(i);f4(i)];
          f(4,i)=par.Finv(4,:)*[f1(i);f2(i);f3(i);f4(i)];
        
          ud = velocity(:,i)*par.kd*([-1 1 -1 1]*real(sqrt(f(:,i)/par.kf)));
          utau = ud*par.l;
          
          acc(:,i)=(-[0;0;par.g]+R*[0;0;f1(i)]/par.m);%ud/par.m);
        
          jerk(:,i) = R*[0;0;x_resamp(i,14)]/par.m + cross(omega(:,i),R*[0;0;f1(i)])/par.m;
        
          omegad(:,i) = par.J\(-cross(omega(:,i),par.J*omega(:,i))+[f2(i);f3(i);f4(i)]);%+utau); 

end  

f1 = f1(round(index));

jerk = jerk(:,round(index));

theta = theta(round(index));
phi = phi(round(index));
psy = psy(round(index));

velocity = velocity(:,round(index));
position = position(:,round(index));

velocity(:,end) = [0;0;0]; 

x = position(1,:);
y = position(2,:);
z = position(3,:);

xd = velocity(1,:);
yd = velocity(2,:);
zd = velocity(3,:);

omega=omega(:,round(index));
omega(:,end)=[0;0;0];

acc=acc(:,round(index));
omegad=omegad(:,round(index));

xdd = acc(1,:);
ydd = acc(2,:);
zdd = acc(3,:);
% 
thetadd= omegad(1,:);
phidd= omegad(2,:);
psydd= omegad(3,:);




