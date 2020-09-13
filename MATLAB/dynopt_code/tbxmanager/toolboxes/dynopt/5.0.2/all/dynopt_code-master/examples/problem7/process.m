function sys = process(t,x,flag,u,p)


switch flag,
    case 0 % f(x,u,p,t)
        q = u(1)+u(2)+u(4);
        sys = [u(4)-q*x(1)-17.6*x(1)*x(2)-23.0*x(1)*x(6)*u(3);
               u(1)-q*x(2)-17.6*x(1)*x(2)-146.0*x(2)*x(3);
               u(2)-q*x(3)-73.0*x(2)*x(3);
               -q*x(4)+35.20*x(1)*x(2)-51.30*x(4)*x(5);
               -q*x(5)+219.0*x(2)*x(3)-51.30*x(4)*x(5);
               -q*x(6)+102.60*x(4)*x(5)-23.0*x(1)*x(6)*u(3);
               -q*x(7)+46.0*x(1)*x(6)*u(3);
               5.80*((q*x(1))-u(4))-3.70*u(1)-4.10*u(2)+q*(23.0*x(4)+11.0*x(5)+28.0*x(6)+35.0*x(7))-5.0*u(3)^2-0.099];
    case 1 % df/dx
        q = u(1)+u(2)+u(4);
        sys = [-q-17.6*x(2)-23.0*x(6)*u(3) -17.6*x(2) 0 35.20*x(2) 0 -23.0*x(6)*u(3) 46.0*x(6)*u(3) 5.80*q;
               -17.6*x(1) -q-17.6*x(1)-146.0*x(3) -73.0*x(3) 35.20*x(1) 219.0*x(3) 0 0 0;
               0 -146.0*x(2) -q-73.0*x(2) 0 219.0*x(2) 0 0 0;
               0 0 0 -q-51.30*x(5) -51.30*x(5) 102.60*x(5) 0 23.0*q;
               0 0 0 -51.30*x(4) -q-51.30*x(4) 102.60*x(4) 0 11.0*q;
               -23.0*x(1)*u(3) 0 0 0 0 -q-23.0*x(1)*u(3) 46.0*x(1)*u(3) 28.0*q;
               0 0 0 0 0 0 -q 35.0*q;
               0 0 0 0 0 0 0 0];
    case 2 % df/du
        sys = [-x(1) 1-x(2) -x(3) -x(4) -x(5) -x(6) -x(7) 5.8*x(1)-3.7+23*x(4)+11*x(5)+28*x(6)+35*x(7);      
               -x(1) -x(2) 1-x(3) -x(4) -x(5) -x(6) -x(7) 5.8*x(1)-4.1+23*x(4)+11*x(5)+28*x(6)+35*x(7) ;      
               -23*x(1)*x(6) 0 0 0 0 -23*x(1)*x(6) 46*x(1)*x(6) -10*u(3);      
               1-x(1) -x(2) -x(3) -x(4) -x(5) -x(6) -x(7) 5.8*x(1)-5.8+23*x(4)+11*x(5)+28*x(6)+35*x(7)];
    case 3 % df/dp
        sys = [];
    case 4 % df/dt
        sys = zeros(1,8);
    case 5 % x0
        sys = [0.1883;0.2507;0.0467;0.0899;0.1804;0.1394;0.1046;0];
    case 6 % dx0/dp
        sys = [];
    case 7 % M
        sys = [];
    case 8 % unused flag
        sys = [];
    otherwise
        error(['unhandled flag = ',num2str(flag)]); 
end