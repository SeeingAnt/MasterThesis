load(traj_name)

addpath('tool')
par=InitParameter();

acc = [0;0;0];
omegad = [0;0;0];

%%

[u_resamp,ty]=resample(uplot,tplot');
[x_resamp,ty]=resample(xplot,tplot');


freq = ty(end)/size(ty,2);
n = 0.002/freq;
num = size(ty,2)/n;
index = linspace(1,size(ty,2),ceil(num));

time = ty(round(index)); 

u=[];
acc=[0;0;0];
jerk=[0;0;0];
f1 = 0;
f1_std = 0;
f2 = 0;
f2_std = 0;
f3 = 0;
f3_std = 0;
f4 = 0;
f4_std = 0;
omega = [0;0;0];
psy =  0;
phi = 0;
theta = 0;
velocity = [0;0;0];
position = [0;0;0];
omega_std = [0;0;0];
psy_std =  0;
phi_std = 0;
theta_std = 0;
velocity_std = [0;0;0];
position_std = [0;0;0];

qw=[0];
qx=[0];
qy=[0];
qz=[0];

f = [0;0;0;0];

for i=1:length(ty)

            kd = par.xi_q;
            kt = par.xi_q;
    
            x_k = zeros(10,1);
            y_k = zeros(10,1);
            z_k = zeros(10,1);
            theta_k = zeros(10,1);
            phi_k = zeros(10,1);
            psy_k = zeros(10,1);
            vx_k = zeros(10,1);
            vy_k = zeros(10,1);
            vz_k = zeros(10,1);
            om1_k = zeros(10,1);
            om2_k = zeros(10,1);
            om3_k = zeros(10,1);
            
            N=3;

            for k = 1:10
                for l = 1:N
                    for j = 1:N
                        phi_xi=1;
                        phi_xi = phi_xi*polyval(par.PsiPol2{par.Alpha(k,1)+1},kd(j));

                        phi_xi = phi_xi*polyval(par.PsiPol2{par.Alpha(k,2)+1},kt(l));

                        x_k(k) = x_k(k)+(x_resamp(i,(l-1)*40+(j-1)*12+1)*...
                                phi_xi*par.w_q(l)*par.w_q(j));
                        y_k(k) = y_k(k)+(x_resamp(i,(l-1)*40+(j-1)*12+2)*...
                                phi_xi*par.w_q(l)*par.w_q(j));
                        z_k(k) = z_k(k)+(x_resamp(i,(l-1)*40+(j-1)*12+3)*...
                                phi_xi*par.w_q(l)*par.w_q(j));
                        vx_k(k) = vx_k(k)+(x_resamp(i,(l-1)*40+(j-1)*12+4)*...
                                phi_xi*par.w_q(l)*par.w_q(j));
                        vy_k(k) = vy_k(k)+(x_resamp(i,(l-1)*40+(j-1)*12+5)*...
                                phi_xi*par.w_q(l)*par.w_q(j));
                        vz_k(k) = vz_k(k)+(x_resamp(i,(l-1)*40+(j-1)*12+6)*...
                                phi_xi*par.w_q(l)*par.w_q(j));
                        theta_k(k) = theta_k(k)+(x_resamp(i,(l-1)*40+(j-1)*12+7)*...
                                phi_xi*par.w_q(l)*par.w_q(j));
                        phi_k(k) = phi_k(k)+(x_resamp(i,(l-1)*40+(j-1)*12+8)*...
                                phi_xi*par.w_q(l)*par.w_q(j));
                        psy_k(k) = psy_k(k)+(x_resamp(i,(l-1)*40+(j-1)*12+9)*...
                                phi_xi*par.w_q(l)*par.w_q(j));
                        om1_k(k) = om1_k(k)+(x_resamp(i,(l-1)*40+(j-1)*12+10)*...
                                phi_xi*par.w_q(l)*par.w_q(j));
                        om2_k(k) = om2_k(k)+(x_resamp(i,(l-1)*40+(j-1)*12+11)*...
                                phi_xi*par.w_q(l)*par.w_q(j));
                        om3_k(k) = om3_k(k)+(x_resamp(i,(l-1)*40+(j-1)*12+12)*...
                                phi_xi*par.w_q(l)*par.w_q(j));    
                    end
                end    
            end

            x_k = x_k./par.PsiSqNorm2;
            y_k = y_k./par.PsiSqNorm2;
            z_k = z_k./par.PsiSqNorm2;
            vx_k = vx_k./par.PsiSqNorm2;
            vy_k = vy_k./par.PsiSqNorm2;
            vz_k = vz_k./par.PsiSqNorm2;
            theta_k = theta_k./par.PsiSqNorm2;
            phi_k = phi_k./par.PsiSqNorm2;
            psy_k = psy_k./par.PsiSqNorm2;
            om1_k = om1_k./par.PsiSqNorm2;
            om2_k = om2_k./par.PsiSqNorm2;
            om3_k = om3_k./par.PsiSqNorm2;

            position(1,i)=x_k(1);
            position_std(1,i)=3*sqrt(x_k(2:end).^2'*par.PsiSqNorm2(2:end));
            position(2,i)=y_k(1);
            position_std(1,i)=3*sqrt(y_k(2:end).^2'*par.PsiSqNorm2(2:end));
            position(3,i)=z_k(1);
            position_std(1,i)=3*sqrt(z_k(2:end).^2'*par.PsiSqNorm2(2:end));
            velocity(1,i)=vx_k(1);
            velocity_std(1,i)=3*sqrt(vx_k(2:end).^2'*par.PsiSqNorm2(2:end));
            velocity(2,i)=vy_k(1);
            velocity_std(2,i)=3*sqrt(vy_k(2:end).^2'*par.PsiSqNorm2(2:end));
            velocity(3,i)=vz_k(1);
            velocity_std(3,i)=3*sqrt(vz_k(2:end).^2'*par.PsiSqNorm2(2:end));
            theta(i)=theta_k(1);
            theta_std(i)=3*sqrt(theta_k(2:end).^2'*par.PsiSqNorm2(2:end));
            phi(i)=phi_k(1);
            phi_std(i)=3*sqrt(phi_k(2:end).^2'*par.PsiSqNorm2(2:end));
            psy(i)=psy_k(1);
            psy_std(i)=3*sqrt(psy_k(2:end).^2'*par.PsiSqNorm2(2:end));
            omega(1,i)=om1_k(1);
            omega_std(1,i)=3*sqrt(om1_k(2:end).^2'*par.PsiSqNorm2(2:end));
            omega(2,i)=om2_k(1);
            omega_std(2,i)=3*sqrt(om2_k(2:end).^2'*par.PsiSqNorm2(2:end));
            omega(3,i)=om3_k(1);
            omega_std(3,i)=3*sqrt(om3_k(2:end).^2'*par.PsiSqNorm2(2:end));
            
            f1_k = zeros(4,1);
            f2_k = zeros(4,1);
            f3_k = zeros(4,1);
            f4_k = zeros(4,1);

            for k = 1:4
                for j = 1:N
                        phi_xi = polyval(par.PsiPol{k},kt(j));

                        f1_k(k) = f1_k(k)+(x_resamp(i,37+(j-1)*40)*...
                                phi_xi*par.w_q(j));
                        f2_k(k) = f2_k(k)+(x_resamp(i,38+(j-1)*40)*...
                                phi_xi*par.w_q(j));
                        f3_k(k) = f3_k(k)+(x_resamp(i,39+(j-1)*40)*...
                                phi_xi*par.w_q(j));
                        f4_k(k) = f4_k(k)+(x_resamp(i,40+(j-1)*40)*...
                                phi_xi*par.w_q(j));    
                end    
            end

            f1_k = f1_k./par.PsiSqNorm;
            f2_k = f2_k./par.PsiSqNorm;
            f3_k = f3_k./par.PsiSqNorm;
            f4_k = f4_k./par.PsiSqNorm;

            f1(i) = f1_k(1);
            f1_std(i) = 3*sqrt(f1_k(2:end).^2'*par.PsiSqNorm(2:end));
            f2(i) = f2_k(1);
            f2_std(i) = 3*sqrt(f2_k(2:end).^2'*par.PsiSqNorm(2:end));
            f3(i) = f3_k(1);
            f3_std(i) = 3*sqrt(f3_k(2:end).^2'*par.PsiSqNorm(2:end));
            f4(i) = f4_k(1);
            f4_std(i) = 3*sqrt(f4_k(2:end).^2'*par.PsiSqNorm(2:end));

          
          R=eul2rotm([psy(i) phi(i) theta(i)]);
          R_p_std=eul2rotm([psy(i)+psy_std(i) phi(i)+phi_std(i) theta(i)+theta_std(i)]);
          R_m_std=eul2rotm([psy(i)-psy_std(i) phi(i)-phi_std(i) theta(i)-theta_std(i)]);
          
          f(1,i)=par.Finv(1,:)*[f1(i);f2(i);f3(i);f4(i)];
          f(2,i)=par.Finv(2,:)*[f1(i);f2(i);f3(i);f4(i)];
          f(3,i)=par.Finv(3,:)*[f1(i);f2(i);f3(i);f4(i)];
          f(4,i)=par.Finv(4,:)*[f1(i);f2(i);f3(i);f4(i)];
          
          f_max(:,i)=par.Finv*([f1(i)+f1_std(i);f2(i)+f2_std(i);f3(i)+f3_std(i);f4(i)+f4_std(i)]);
          
          f_min(:,i)=par.Finv*([f1(i)-f1_std(i);f2(i)-f2_std(i);f3(i)-f3_std(i);f4(i)-f4_std(i)]);
        
          kd = [par.kd-6e-7;par.kd;par.kd+6e-7];
          kt = [250-6e1;250;250+6e1];
          
          
          v_perp = (velocity(:,i)-(R(:,3)'*velocity(:,i))*R(:,3));
          ud = -v_perp*kd(2)*([-1 1 -1 1]*real(sqrt(f(:,i)/par.kf)));
         
          v_perp_p_std = ((velocity(:,i)+velocity_std(:,i))-(R_p_std(:,3)'*(velocity(:,i)+velocity_std(:,i)))*R_p_std(:,3));
          ud_p_std = -(velocity(:,i)+velocity_std(:,i))*kd(3)*([-1 1 -1 1]*real(sqrt(f_max(:,i)/par.kf)));
          v_perp_m_std = ((velocity(:,i)-velocity_std(:,i))-(R_m_std(:,3)'*(velocity(:,i)-velocity_std(:,i)))*R_m_std(:,3));
          ud_m_std = -(velocity(:,i)-velocity_std(:,i))*kd(1)*([-1 1 -1 1]*real(sqrt(f_min(:,i)/par.kf)));
          
          utau = [ud(2);ud(3);0]*par.l;
          utau_p_std = [ud_m_std(2);ud_m_std(3);0]*par.l;
          utau_m_std = [ud_m_std(2);ud_m_std(3);0]*par.l;
          
          acc(:,i)=(-[0;0;par.g]+R*[0;0;f1(i)]/par.m + ud/par.m);
          acc_p_std(:,i)=(-[0;0;par.g]+R*[0;0;f1(i)+f1_std(i)]/par.m + ud_p_std/par.m);
          acc_m_std(:,i)=(-[0;0;par.g]+R*[0;0;f1(i)-f1_std(i)]/par.m + ud_m_std/par.m);
            
          jerk(:,i) = R*[0;0;x_resamp(i,14)]/par.m + cross(omega(:,i),R*[0;0;f1(i)])/par.m;
          jerk_p_std(:,i) = R*[0;0;x_resamp(i,14)]/par.m + cross(omega(:,i),R*[0;0;f1(i)])/par.m;
          jerk_m_std(:,i) = R*[0;0;x_resamp(i,14)]/par.m + cross(omega(:,i),R*[0;0;f1(i)])/par.m;
          
          omegad(:,i) = par.J\(-cross(omega(:,i),par.J*omega(:,i))+[f2(i);f3(i);f4(i)]+utau);
          omegad_p_std(:,i) = par.J\(-cross((omega(:,i)+omega_std(:,i)),par.J*(omega(:,i)+omega_std(:,i)))+[f2(i)+f2_std(i);f3(i)+f3_std(i);f4(i)+f4_std(i)]+utau_p_std);
          omegad_m_std(:,i) = par.J\(-cross((omega(:,i)+omega_std(:,i)),par.J*(omega(:,i)-omega_std(:,i)))+[f2(i)-f2_std(i);f3(i)-f3_std(i);f4(i)-f4_std(i)]+utau_m_std);
          
end  

f1 = f1(round(index));

jerk = jerk(:,round(index));
jerk_p_std = jerk_p_std(:,round(index));
jerk_m_std = jerk_m_std(:,round(index));

theta = theta(round(index));
phi = phi(round(index));
psy = psy(round(index));
theta_std = theta_std(round(index));
phi_std = phi_std(round(index));
psy_std = psy_std(round(index));

velocity = velocity(:,round(index));
velocity_std = velocity_std(:,round(index));
position = position(:,round(index));
position_std = position_std(:,round(index));

velocity(:,end) = [0;0;0]; 

x = position(1,:);
y = position(2,:);
z = position(3,:);

xd = velocity(1,:);
yd = velocity(2,:);
zd = velocity(3,:);

omega=omega(:,round(index));
omega_std=omega_std(:,round(index));
omega(:,end)=[0;0;0];

acc=acc(:,round(index));
acc_p_std=acc_p_std(:,round(index));
acc_m_std=acc_m_std(:,round(index));
omegad=omegad(:,round(index));
omegad_p_std=omegad_p_std(:,round(index));
omegad_m_std=omegad_m_std(:,round(index));

xdd = acc(1,:);
ydd = acc(2,:);
zdd = acc(3,:);
% 
thetadd= omegad(1,:);
phidd= omegad(2,:);
psydd= omegad(3,:);
