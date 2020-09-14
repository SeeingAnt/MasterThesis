function sys = process(t,x,flag,f,p)
par = InitParameter(2);
switch flag
case 0 % f(x,u,p,t)
           cy = cos(x(9));
           sy = sin(x(9));
           cp = cos(x(8));
           sp = sin(x(8));
           cr = cos(x(7));
           sr = sin(x(7));
         
          load xf xf 
          
          W = [ 1         tan(x(8))*sr    tan(x(8))*cr; ...
                  0        cr                       -sr; ...
                  0        sr/cp                  cr/cp];
              
          R=eul2rotm([x(9) x(8) x(7)]);
          
%          drag forces and torques           
%
%          u=[0;0;0;0];
%                 
%           u(1)=par.Finv(1,:)*([x(13);x(15:17)]);
%           u(2)=par.Finv(2,:)*([x(13);x(15:17)]);
%           u(3)=par.Finv(3,:)*([x(13);x(15:17)]);
%           u(4)=par.Finv(4,:)*([x(13);x(15:17)]);
%           
%           v_perp = (x(4:6)-(x(4:6)*R(:,3))*R(:,3)); 
%           ud = -v_perp*par.kd*([-1 1 -1 1]*real(sqrt(u/par.kf)));
%           utau = par.l*[ud(2);ud(1);ud(3)];
%           
          omega=[x(10);x(11);x(12)];    
          sys = [ x(4);x(5);x(6);
                  [0;0;-par.g]+R*[0;0;x(13)]/par.m; %+ ud/par.m;
                  W*omega;
                  par.J\(-cross(omega,par.J*omega)+x(15:17));% +utau);
                  x(14);
                  f(1);
                  x(18:20);
                  f(2:4);                   
                  norm([f])^2;
                 ];   
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