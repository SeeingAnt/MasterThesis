function [c,ceq,Dc,Dceq] = confun(t,x,flag,u,p)

    c=[];
    ceq=[];
    
    par =InitParameter(2);
   
    sigp=5e-2;
    siga=1e-2;
    sigv=1e-3;
    N=3;
    
    kd = par.xi_q;
    kt = par.xi_q;
       
    % inputs mean and standard deviation 
    
    f1_k = zeros(4,1);
    f2_k = zeros(4,1);
    f3_k = zeros(4,1);
    f4_k = zeros(4,1);
    
    for k = 1:4
        for j = 1:N
                phi_xi = polyval(par.PsiPol{k},kt(j));
                
                f1_k(k) = f1_k(k)+(x(37+(j-1)*40)*...
                        phi_xi*par.w_q(j));
                f2_k(k) = f2_k(k)+(x(38+(j-1)*40)*...
                        phi_xi*par.w_q(j));
                f3_k(k) = f3_k(k)+(x(39+(j-1)*40)*...
                        phi_xi*par.w_q(j));
                f4_k(k) = f4_k(k)+(x(40+(j-1)*40)*...
                        phi_xi*par.w_q(j));    
        end    
    end
    
    f1_k = f1_k./par.PsiSqNorm;
    f2_k = f2_k./par.PsiSqNorm;
    f3_k = f3_k./par.PsiSqNorm;
    f4_k = f4_k./par.PsiSqNorm;
    
    f1_mean = f1_k(1);
    f1_std = sqrt(f1_k(2:end).^2'*par.PsiSqNorm(2:end));
    f2_mean = f2_k(1);
    f2_std = sqrt(f2_k(2:end).^2'*par.PsiSqNorm(2:end));
    f3_mean = f3_k(1);
    f3_std = sqrt(f3_k(2:end).^2'*par.PsiSqNorm(2:end));
    f4_mean = f4_k(1);
    f4_std = sqrt(f4_k(2:end).^2'*par.PsiSqNorm(2:end));
    
    f_max=par.Finv*([f1_mean+f1_std;f2_mean+f2_std;f3_mean+f3_std;f4_mean+f4_std]);
    
    f_min=par.Finv*([f1_mean-f1_std;f2_mean-f2_std;f3_mean-f3_std;f4_mean-f4_std]);
    
    switch flag
    case 0 % constraints in t0
    % constraints
    load u0 u0
    
    % position mean and standard deviation 
    x_k = zeros(10,1);
    y_k = zeros(10,1);
    z_k = zeros(10,1);
   
    for k = 1:10
        for i = 1:N
            for j = 1:N
                phi_xi=1;
                
                phi_xi = phi_xi*polyval(par.PsiPol2{par.Alpha(k,1)+1},kd(j));
                
                phi_xi = phi_xi*polyval(par.PsiPol2{par.Alpha(k,2)+1},kt(i));
                
                x_k(k) = x_k(k)+(x((i-1)*40+(j-1)*12+1)*...
                        phi_xi*par.w_q(i)*par.w_q(j));
                y_k(k) = y_k(k)+(x((i-1)*40+(j-1)*12+2)*...
                        phi_xi*par.w_q(i)*par.w_q(j));
                z_k(k) = z_k(k)+(x((i-1)*40+(j-1)*12+3)*...
                        phi_xi*par.w_q(i)*par.w_q(j));
            end
        end    
    end
    x_k = x_k./par.PsiSqNorm2;
    y_k = y_k./par.PsiSqNorm2;
    z_k = z_k./par.PsiSqNorm2;
    
    x_mean=x_k(1);
    x_std=sqrt(x_k(2:end).^2'*par.PsiSqNorm2(2:end));
    y_mean=y_k(1);
    y_std=sqrt(y_k(2:end).^2'*par.PsiSqNorm2(2:end));
    z_mean=z_k(1);
    z_std=sqrt(z_k(2:end ).^2'*par.PsiSqNorm2(2:end));
    
    c = [x_mean+x_std-par.x_max;
       -(x_mean-x_std)+par.x_safe;
         y_mean+y_std-par.y_max;
       -(y_mean-y_std)+par.y_safe;
         z_mean+z_std-par.z_max;
       -(z_mean-z_std)+par.z_safe;];
    ceq = [abs(u-u0)];
    
    case 1 % constraints over interval [t0,tf]
  
        x_k = zeros(10,1);
        y_k = zeros(10,1);
        z_k = zeros(10,1);


        for k = 1:10
            for i = 1:N
                for j = 1:N
                    phi_xi=1;
                    phi_xi = phi_xi*polyval(par.PsiPol2{par.Alpha(k,1)+1},kd(j));

                    phi_xi = phi_xi*polyval(par.PsiPol2{par.Alpha(k,2)+1},kt(i));

                    x_k(k) = x_k(k)+(x((i-1)*40+(j-1)*12+1)*...
                            phi_xi*par.w_q(i)*par.w_q(j));
                    y_k(k) = y_k(k)+(x((i-1)*40+(j-1)*12+2)*...
                            phi_xi*par.w_q(i)*par.w_q(j));
                    z_k(k) = z_k(k)+(x((i-1)*40+(j-1)*12+3)*...
                            phi_xi*par.w_q(i)*par.w_q(j));

                end
            end    
        end
        x_k = x_k./par.PsiSqNorm2;
        y_k = y_k./par.PsiSqNorm2;
        z_k = z_k./par.PsiSqNorm2;

        x_mean=x_k(1);
        x_std=sqrt(x_k(2:end).^2'*par.PsiSqNorm2(2:end));
        y_mean=y_k(1);
        y_std=sqrt(y_k(2:end).^2'*par.PsiSqNorm2(2:end));
        z_mean=z_k(1);
        z_std=sqrt(z_k(2:end).^2'*par.PsiSqNorm2(2:end));
        
        if isempty(par.spatial_constraints) ~= true
          [c,ceq] = par.spatial_constraints(x,u,p);
        end
        c = [x_mean+x_std-par.x_max;
           -(x_mean-x_std)+par.x_safe;
             y_mean+y_std-par.y_max;
           -(y_mean-y_std)+par.y_safe;
             z_mean+z_std-par.z_max;
           -(z_mean-z_std)+par.z_safe;
            -f_min;
             f_max-par.f1_max;
             c
            ];
        
            ceq=[ceq];

        
    case 2 % constraints in tf
    % constraints
    load xf xf
    
    c = [];
    ceq = [];
    
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
   
    
    for k = 1:10
        for i = 1:N
            for j = 1:N
                phi_xi=1;
                phi_xi = phi_xi*polyval(par.PsiPol2{par.Alpha(k,1)+1},kd(j));
                
                phi_xi = phi_xi*polyval(par.PsiPol2{par.Alpha(k,2)+1},kt(i));
                
                x_k(k) = x_k(k)+(x((i-1)*40+(j-1)*12+1)*...
                        phi_xi*par.w_q(i)*par.w_q(j));
                y_k(k) = y_k(k)+(x((i-1)*40+(j-1)*12+2)*...
                        phi_xi*par.w_q(i)*par.w_q(j));
                z_k(k) = z_k(k)+(x((i-1)*40+(j-1)*12+3)*...
                        phi_xi*par.w_q(i)*par.w_q(j));
                vx_k(k) = vx_k(k)+(x((i-1)*40+(j-1)*12+4)*...
                        phi_xi*par.w_q(i)*par.w_q(j));
                vy_k(k) = vy_k(k)+(x((i-1)*40+(j-1)*12+5)*...
                        phi_xi*par.w_q(i)*par.w_q(j));
                vz_k(k) = vz_k(k)+(x((i-1)*40+(j-1)*12+6)*...
                        phi_xi*par.w_q(i)*par.w_q(j));
                theta_k(k) = theta_k(k)+(x((i-1)*40+(j-1)*12+7)*...
                        phi_xi*par.w_q(i)*par.w_q(j));
                phi_k(k) = phi_k(k)+(x((i-1)*40+(j-1)*12+8)*...
                        phi_xi*par.w_q(i)*par.w_q(j));
                psy_k(k) = psy_k(k)+(x((i-1)*40+(j-1)*12+9)*...
                        phi_xi*par.w_q(i)*par.w_q(j));
                om1_k(k) = om1_k(k)+(x((i-1)*40+(j-1)*12+10)*...
                        phi_xi*par.w_q(i)*par.w_q(j));
                om2_k(k) = om2_k(k)+(x((i-1)*40+(j-1)*12+11)*...
                        phi_xi*par.w_q(i)*par.w_q(j));
                om3_k(k) = om3_k(k)+(x((i-1)*40+(j-1)*12+12)*...
                        phi_xi*par.w_q(i)*par.w_q(j));    
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
    
    x_mean=x_k(1);
    x_std=sqrt(x_k(2:end).^2'*par.PsiSqNorm2(2:end));
    y_mean=y_k(1);
    y_std=sqrt(y_k(2:end).^2'*par.PsiSqNorm2(2:end));
    z_mean=z_k(1);
    z_std=sqrt(z_k(2:end).^2'*par.PsiSqNorm2(2:end));
    vx_mean=vx_k(1);
    vx_std=sqrt(vx_k(2:end).^2'*par.PsiSqNorm2(2:end));
    vy_mean=vy_k(1);
    vy_std=sqrt(vy_k(2:end).^2'*par.PsiSqNorm2(2:end));
    vz_mean=vz_k(1);
    vz_std=sqrt(vz_k(2:end).^2'*par.PsiSqNorm2(2:end));
    theta_mean=theta_k(1);
    theta_std=sqrt(theta_k(2:end).^2'*par.PsiSqNorm2(2:end));
    phi_mean=phi_k(1);
    phi_std=sqrt(phi_k(2:end).^2'*par.PsiSqNorm2(2:end));
    psy_mean=psy_k(1);
    psy_std=sqrt(psy_k(2:end).^2'*par.PsiSqNorm2(2:end));
    om1_mean=om1_k(1);
    om1_std=sqrt(om1_k(2:end).^2'*par.PsiSqNorm2(2:end));
    om2_mean=om2_k(1);
    om2_std=sqrt(om2_k(2:end).^2'*par.PsiSqNorm2(2:end));
    om3_mean=om3_k(1);
    om3_std=sqrt(om3_k(2:end).^2'*par.PsiSqNorm2(2:end));
    
    % define inequality and equality contraints taking under account the
    % mean and the standard deviation
    
    if isnan(xf(1))
       c = [c;x_mean+x_std-par.x_max;
           -(x_mean-x_std)+par.x_safe;];
    else
        ceq = [ceq;abs(x_mean-xf(1))];
        c=[c;x_std-sigp;];
    end
    if isnan(xf(2))
        c=[c;y_mean+y_std-par.y_max;
           -(y_mean-y_std)+par.y_safe;];
    else
        ceq = [ceq;abs(y_mean-xf(2))];
        c=[c;y_std-sigp;];
    end
    
    if isnan(xf(3))
        c=[c;z_mean+z_std-par.z_max;
           -(z_mean-z_std)+par.z_safe];
    else
        ceq = [ceq;abs(z_mean-xf(3))];
        c=[c;z_std-sigp;];
    end
    
    if length(xf)>6  
    
    if isnan(xf(9)) == false
        ceq = [ceq;abs(psy_mean-xf(9))];
        c=[c;psy_std-siga;];
    end   
        
    ceq = [ceq;abs(theta_mean-xf(7));
           abs(f1_mean-par.m*par.g);
           abs(x(121)-par.m*par.g);
           abs(phi_mean-xf(8));
           abs(x(122:124));
           abs(f2_mean);
           abs(f3_mean);
           abs(f4_mean);
           abs(vx_mean);
           abs(vy_mean);
           abs(vz_mean);
           abs(om1_mean);
           abs(om2_mean);
           abs(om3_mean);
           abs(u);
           ];     
       
    c= [c;theta_std-siga;
        phi_std-siga;
        f1_std-sigv;
        f2_std-sigv;
        f3_std-sigv;
        f4_std-sigv;
        vx_std-siga;
        vy_std-siga;
        vz_std-siga;
        om1_std-siga;
        om2_std-siga;
        om3_std-siga];  
       
    else    
    
    if isnan(xf(6)) == false
        ceq = [ceq;abs(x(9)-xf(6))];
    end   
        
    c = [c;
        -f(1);
        -f(2);
        -f(3);
        -f(4);
         f(1)-par.f1_max;
         f(2)-par.f1_max;
         f(3)-par.f1_max;
         f(4)-par.f1_max];
    ceq = [ceq;
           abs(x(7)-xf(4));
           abs(x(8)-xf(5));
           abs(x(4:6));
           abs(x(10:12));
           ];
       

    end
    
    
    end   
      
end

