function sys = process(t,x,flag,f,p)
par = InitParameter(1);
switch flag
case 0 % f(x,u,p,t)

        load xf xf 
        sys =[];
        
        kd = sqrt(2)*par.xi_q*2e-7+par.kd;
        kt = sqrt(2)*par.xi_q*2e1+250;
        
        for j=1:3
            for i=1:3  
             cy = cos(x(9+(i-1)*12+(j-1)*40));
             sy = sin(x(9+(i-1)*12+(j-1)*40));
             cp = cos(x(8+(i-1)*12+(j-1)*40));
             sp = sin(x(8+(i-1)*12+(j-1)*40));
             cr = cos(x(7+(i-1)*12+(j-1)*40));
             sr = sin(x(7+(i-1)*12+(j-1)*40)); 
          
             W = [ 1        tan(x(8+(i-1)*12+(j-1)*40))*sr    tan(x(8+(i-1)*12+(j-1)*40))*cr; ...
                   0        cr                       -sr; ...
                   0        sr/cp                  cr/cp];
              
             R=eul2rotm([x(9+(i-1)*12+(j-1)*40) x(8+(i-1)*12+(j-1)*40) x(7+(i-1)*12+(j-1)*40)]);
          
             u=[0;0;0;0];
                   
              u(1)=par.Finv(1,:)*([x(37+(j-1)*40:40+(j-1)*40)]);
              u(2)=par.Finv(2,:)*([x(37+(j-1)*40:40+(j-1)*40)]);
              u(3)=par.Finv(3,:)*([x(37+(j-1)*40:40+(j-1)*40)]);
              u(4)=par.Finv(4,:)*([x(37+(j-1)*40:40+(j-1)*40)]);

              v_perp = (x(4+(i-1)*12+(j-1)*40:6+(i-1)*12+(j-1)*40)-(R(:,3)'*x(4+(i-1)*12+(j-1)*40:6+(i-1)*12+(j-1)*40))*R(:,3));
              ud = -v_perp*kd(i)*([-1 1 -1 1]*real(sqrt(u/par.kf)));
              utau = par.l*[ud(2);ud(1);ud(3)];
          
              omega=[x(10+(i-1)*12+(j-1)*40);x(11+(i-1)*12+(j-1)*40);x(12+(i-1)*12+(j-1)*40)];    
              sys = [sys; x(4+(i-1)*12+(j-1)*40);x(5+(i-1)*12+(j-1)*40);x(6+(i-1)*12+(j-1)*40);
                      [0;0;-par.g]+R*[0;0;x(37+(j-1)*40)]/par.m + ud/par.m;
                      W*omega;
                      par.J\(-cross(omega,par.J*omega)+x(38+(j-1)*40:40+(j-1)*40) +utau);
                     ];   
            end
           sys = [sys;-kt(j)*x(37+(j-1)*40)+kt(j)*x(121);
                  -kt(j)*x(38+(j-1)*40:40+(j-1)*40)+kt(j)*x(122:124);];
        end    
           sys=[sys;f(1:4);                   
                norm([f;x(121:124)])^2]; 
             
case 1 % df/dx
        
        if x(9) == pi/2
            cy = cos(x(9)+1e-15);
        else
            cy = cos(x(9));
        end
        
        if x(9) == 0
            sy = sin(x(9)+1e-15);
        else
            sy = sin(x(9));
        end
        
        if x(8) == pi/2
            cp = cos(x(8)+1e-15);
        else
            cp = cos(x(8));
        end
        
        if x(8) == 0
            sp = sin(x(9)+1e-15);
        else
            sp = sin(x(9));
        end
        
        if x(7) == pi/2
            cr = cos(x(7)+1e-15);
        else
            cr = cos(x(7));
        end
        
        if x(7) == 0
            sr = sin(x(7)+1e-15);
        else
            sr = sin(x(7));
        end
    
    
        sys = [ 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
                0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
                0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
                0, 0, 0, 0, 0, 0, x(13)*(cr*sy-cy*sp*sr)/par.m, ...
                   x(13)*cp*cy*cr/par.m, x(13)*(cy*sr-cr*sp*sy)/par.m, ...
                   0, 0, 0, sp*sr/par.m + cy*cr*sp/par.m, 0, 0, 0, 0;
                0, 0, 0, 0, 0, 0, -x(13)*(cy*cr+sp*sy*sr)/par.m, ...
                   x(13)*cp*cr*sy/par.m, x(13)*(sy*sr+cy*cr*sp)/par.m, ...
                   0, 0, 0, cr*sp*sy/par.m-cy*sr/par.m, 0, 0, 0, 0;
                0, 0, 0, 0, 0, 0, -x(13)*cp*sr/par.m, -x(13)*cr*sp/par.m, ...
                   0, 0, 0, 0, cp*cr/par.m, 0, 0, 0, 0;
                0, 0, 0, 0, 0, 0, x(11)*cr*tan(x(8))-x(12)*tan(x(8))*sr, ...
                   x(12)*cr*(tan(x(8))^2 + 1)+x(11)*sr*(tan(x(8))^2 + 1), ...
                   0, 1, tan(x(8))*sr, cr*tan(x(8)), 0, 0, 0, 0, 0 ;
                0, 0, 0, 0, 0, 0,-x(12)*cr-x(11)*sr, 0, 0, 0, cr, ...
                   -sr, 0, 0, 0, 0, 0];
          sys= [sys;0, 0, 0, 0, 0, 0, (x(11)*cr)/cp-(x(12)*sr)/cp, ...
                   (x(12)*cr*sp)/cp^2+(x(11)*sp*sr)/cp^2, 0, 0, ...
                   sr/cp, cr/sp, 0, 0, 0, 0, 0];
          sys = [sys; 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -(3745409798147487*x(12))/4891105491739733, ...
                   -(3745409798147487*x(11))/4891105491739733, 0, ...
                     295147905179352825856/4891105491739733, 0, 0, 0;
                0, 0, 0, 0, 0, 0, 0, 0, 0, (3745409798147487*x(12))/4891105491739733, ...
                   0, (3745409798147487*x(10))/4891105491739733, 0, 0, ...
                   295147905179352825856/4891105491739733, 0, 0;
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 73786976294838206464/2159128822471805, 0;
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]';
               
case 2 % df/du
        sys = [0, 0, 0, 0;
               0, 0, 0, 0;
               0, 0, 0, 0;
               0, 0, 0, 0;
               0, 0, 0, 0;
               0, 0, 0, 0;
               0, 0, 0, 0;
               0, 0, 0, 0;
               0, 0, 0, 0;
               0, 0, 0, 0;
               0, 0, 0, 0;
               0, 0, 0, 0;
               1, 0, 0, 0;
               0, 1, 0, 0;
               0, 0, 1, 0;
               0, 0, 0, 1;
               2*f(1), 2*f(2), 2*f(3), 2*f(4)]';   
case 3 % df/dp
sys = [];
case 4 % df/dt
sys = [];
case 5 % x0
load x0 x0
sys =[x0;0];% par.u_hover(1);par.u_hover(2);par.u_hover(3);par.u_hover(4)];
case 6 % dx0/dp
sys = zeros(length(p),length(x));
otherwise % unused flag
sys = [];
end