function y = integrate(x,x0,time)

%     global step;
    
%     s=tf('s');
%     G=1/s;
%     
%     y=lsim(ss(G),x,linspace(0,time,size(x,2)),x0,'foh');
    
    y = zeros(size(x));
    for i = 1:length(x)
       if i == 1
           y(i) = x0;
       else
           y(i) = y(i-1) + x(i)*time;
       end
    end
%    y=y';

end
