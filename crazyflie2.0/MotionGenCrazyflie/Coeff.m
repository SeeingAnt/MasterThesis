% function [P1x, P1y, P1z, P1yaw] = Coeff(X0, XF,tf) 
function [P1x] = Coeff(X0, XF,tf) 

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
            0  0  0  2  0  0  0  0   ;  % yerk ini
            1    tf    tf^2   tf^3       tf^4        tf^5        tf^6         tf^7         ; % pos fin
            0    1     2*tf   3*tf^2     4*tf^3      5*tf^4      6*tf^5       7*tf^6       ; %vel fin
            0    0     2      6*tf       12*tf^2     20*tf^3     30*tf^4      42*tf^5      ; % acc fin  
            0    0     0      6          24*tf       60*tf^2     120*tf^3     210*tf^4 ]    ; % yerk fin
    
        
        
     P1x= M\[x0; xd0; xdd0; xddd0; xf; xdf; xddf; xdddf];
     P1y= M\[y0; yd0; ydd0; yddd0; yf; ydf; yddf; ydddf];
     P1z= M\[z0; zd0; zdd0; zddd0; zf; zdf; zddf; zdddf];
     P1yaw= M\[yaw0; yawd0; yawdd0; yawddd0; yawf; yawdf; yawddf; yawdddf];

   
 
    
end