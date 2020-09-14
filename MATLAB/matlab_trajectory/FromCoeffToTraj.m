function [x,y,z,yaw,xd,yd,zd,yawd,xdd,ydd,zdd,yawdd] = FromCoeffToTraj(Coeff,res) 
    
    tf  = Coeff(1);
    P1x = Coeff(2:9);
    P1y = Coeff(10:17);
    P1z = Coeff(18:25);
    P1yaw = Coeff(26:34);

    step = tf/(res-1);
    t = 0:step:tf;

  
    x=0; y=0; z=1; yaw=0; 
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