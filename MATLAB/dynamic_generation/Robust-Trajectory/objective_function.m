function [f,Df]= objective_function(t,x,u,p)
  
  f =t+x(125);
  
  % gradient of the objective function

    Df.t = 0; % dJ/dt
    Df.x = [zeros(16,1);1]; % dJ/dx
    Df.u = zeros(4,1); % dJ/du
    Df.p = 0; % dJ/dp



  
  
end
