% ------- Rafael Balderas Hill-------------------26/06/2016
%-----------------------------------------------------------
%--------Trajectory: seventh-degree
%----------------------------------------------


function [t,x,y,z,yaw,xd,yd,zd,yawd,xdd,ydd,zdd,yawdd, P1x, P1y, P1z, P1yaw] = Trajectory_Raf_Poly7(X0, XF,tf,freq) 

x0 = X0(1);
y0 = X0(2);
z0 = X0(3);
yaw0 = X0(4);
xf = XF(1);
yf = XF(2);
zf = XF(3);
yawf = XF(4);

    xd0=0;    xdd0=0 ;   xddd0=0;
    xdf=0;    xddf=0 ;   xdddf=0;
    yd0=0;    ydd0=0 ;   yddd0=0;
    ydf=0;    yddf=0 ;   ydddf=0;
    zd0=0;    zdd0=0;   zddd0=0;
    zdf=0;    zddf=0;   zdddf=0;
    yawd0=0;  yawdd0=0;  yawddd0=0;
    yawdf=0;  yawddf=0;  yawdddf=0;
    
    % Trajectory Definition
    format long
    
      M = [ 1  0  0  0  0  0  0  0   ; % pos ini
            0  1  0  0  0  0  0  0   ; % vel ini
            0  0  2  0  0  0  0  0   ; % acc ini 
            0  0  0  6  0  0  0  0   ;  % yerk ini
            1    tf    tf^2   tf^3       tf^4        tf^5        tf^6         tf^7         ; % pos fin
            0    1     2*tf   3*tf^2     4*tf^3      5*tf^4      6*tf^5       7*tf^6       ; %vel fin
            0    0     2      6*tf       12*tf^2     20*tf^3     30*tf^4      42*tf^5      ; % acc fin  
            0    0     0      6          24*tf       60*tf^2     120*tf^3     210*tf^4 ]    ; % yerk fin
    
        
        
     P1x= M\[x0; xd0; xdd0; xddd0; xf; xdf; xddf; xdddf];
     P1y= M\[y0; yd0; ydd0; yddd0; yf; ydf; yddf; ydddf];
     P1z= M\[z0; zd0; zdd0; zddd0; zf; zdf; zddf; zdddf];
     P1yaw= M\[yaw0; yawd0; yawdd0; yawddd0; yawf; yawdf; yawddf; yawdddf];

     t = 0/freq:1/freq:tf;
% t = freq;
  
    x=0; y=0; z=0; yaw=0; % z=Zi ;
    numel(P1x);
    for i=1:numel(P1x)
        x = x + P1x(i)*t.^(i-1);
        y = y + P1y(i)*t.^(i-1);
        z = z + P1z(i)*t.^(i-1);
        yaw = yaw + P1yaw(i)*t.^(i-1);
        
    end
    % Zf=z;
     
    % velocity in x , y , z and yaw
    for i=1:numel(x)        
        xd(i)=P1x(2) + 2*t(i)*P1x(3) + 3*t(i)^2*P1x(4)+ 4*t(i)^3*P1x(5)+ 5*t(i)^4*P1x(6)+ 6*t(i)^5*P1x(7)+ 7*t(i)^6*P1x(8);
        yd(i)=P1y(2) + 2*t(i)*P1y(3) + 3*t(i)^2*P1y(4)+ 4*t(i)^3*P1y(5)+ 5*t(i)^4*P1y(6)+ 6*t(i)^5*P1y(7)+ 7*t(i)^6*P1y(8);
        zd(i)=P1z(2) + 2*t(i)*P1z(3) + 3*t(i)^2*P1z(4)+ 4*t(i)^3*P1z(5)+ 5*t(i)^4*P1z(6)+ 6*t(i)^5*P1z(7)+ 7*t(i)^6*P1z(8);
        yawd(i)=P1yaw(2) + 2*t(i)*P1yaw(3) + 3*t(i)^2*P1yaw(4)+ 4*t(i)^3*P1yaw(5)+ 5*t(i)^4*P1yaw(6)+ 6*t(i)^5*P1yaw(7)+ 7*t(i)^6*P1yaw(8);
    end

    % acceleration in x and y
    for i=1:numel(x)
        xdd(i)=2*P1x(3) + 6*t(i)*P1x(4)+ 12*t(i)^2*P1x(5)+ 20*t(i)^3*P1x(6) + 30*t(i)^4*P1x(7) + 42*t(i)^5*P1x(8);
        ydd(i)=2*P1y(3) + 6*t(i)*P1y(4)+ 12*t(i)^2*P1y(5)+ 20*t(i)^3*P1y(6) + 30*t(i)^4*P1y(7) + 42*t(i)^5*P1y(8);
        zdd(i)=2*P1z(3) + 6*t(i)*P1z(4)+ 12*t(i)^2*P1z(5)+ 20*t(i)^3*P1z(6) + 30*t(i)^4*P1z(7) + 42*t(i)^5*P1z(8);
        yawdd(i)=2*P1yaw(3) + 6*t(i)*P1yaw(4)+ 12*t(i)^2*P1yaw(5)+ 20*t(i)^3*P1yaw(6) + 30*t(i)^4*P1yaw(7) + 42*t(i)^5*P1yaw(8);
    end
 
    
end