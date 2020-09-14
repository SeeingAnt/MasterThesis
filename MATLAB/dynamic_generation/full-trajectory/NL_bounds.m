function [c,ceq,Dc,Dceq] = confun(t,x,flag,u,p)

    c=[];
    ceq=[];
    par =InitParameter(2);
    
          f(1)=par.Finv(1,:)*([x(13);x(15:17)]);
          f(2)=par.Finv(2,:)*([x(13);x(15:17)]);
          f(3)=par.Finv(3,:)*([x(13);x(15:17)]);
          f(4)=par.Finv(4,:)*([x(13);x(15:17)]);     
    
    switch flag
    case 0 % constraints in t0
    % constraints
    load u0 u0
    c = [x(1)-par.x_max;
        -x(1)+par.x_safe;
         x(2)-par.y_max;
        -x(2)+par.y_safe;
         x(3)-par.z_max;
        -x(3)+par.z_safe];
    ceq = [abs(u-u0)];
    
    if isempty(par.spatial_constraints) ~= true
        if(SPCons(x,u,p)>0)
            error("unfeasible spatial constraint")
        end    
    end
    % gradient the calculus
    if nargout == 4
        Dc.t = zeros(1,6);
        Dc.x = [1 -1  0  0  0  0;
                0  0  1 -1  0  0;
                0  0  0  0  1 -1;
                zeros(14,6)]; %zeros(17,6);
        Dc.u = zeros(4,6);
        Dc.p = [];
        Dceq.t = zeros(1,4);
        Dceq.x = zeros(17,4);
        Dceq.u = eye(4,4);
        Dceq.p = [];
    end
    
    case 1 % constraints over interval [t0,tf]
  
        if isempty(par.spatial_constraints) ~= true
          [c,ceq] = par.spatial_constraints(x,u,p);
        end
        c = [x(1)-par.x_max;
            -x(1)+par.x_safe;
             x(2)-par.y_max;
            -x(2)+par.y_safe;
             x(3)-par.z_max;
            -x(3)+par.z_safe;
            -f(1);
            -f(2);
            -f(3);
            -f(4);
             f(1)-par.f1_max;
             f(2)-par.f1_max;
             f(3)-par.f1_max;
             f(4)-par.f1_max;
             c
            ];
        
            ceq=[ceq];
          
         if nargout == 4
            Dc.t = zeros(1,14);
            Dc.x = [1 -1  0  0  0  0  0  0  0  0  0  0  0  0;
                    0  0  1 -1  0  0  0  0  0  0  0  0  0  0;
                    0  0  0  0  1 -1  0  0  0  0  0  0  0  0;
                    zeros(9,14);
                    zeros(1,6) -par.Finv(1,1:4) par.Finv(1,1:4);
                    zeros(1,6) -par.Finv(2,1:4) par.Finv(2,1:4);
                    zeros(1,6) -par.Finv(3,1:4) par.Finv(3,1:4);
                    zeros(1,6) -par.Finv(4,1:4) par.Finv(4,1:4);
                    0  0  0  0  0  0  0  0  0  0  0  0  0  0];%zeros(17,14);
            Dc.u = zeros(4,14);
            Dc.p = [];
            Dceq.t = [];
            Dceq.x = [];
            Dceq.u = [];
            Dceq.p = [];
        end   

        
    case 2 % constraints in tf
    % constraints
    load xf xf
    
     c = [];
     ceq = [];
    
    
     
     if nargout == 4
        Dc.x = [];%zeros(17,size(c,1));
        Dc.p = [];
        Dceq.x = [];%zeros(17,size(ceq,1));
        Dceq.p = [];
    end
     
    if isnan(xf(1))
       c = [c;x(1)-par.x_max;
           -x(1)+par.x_safe];
       if nargout == 4
            Dc.x = [Dc.x,[1 -1;zeros(16,2)]];%zeros(17,size(c,1));
       end
    else
        ceq = [ceq;abs(x(1)-xf(1))];
        if nargout == 4
            Dceq.x = [Dceq.x,[1;zeros(16,1)]];%zeros(17,size(c,1));
       end
    end
    if isnan(xf(2))
        c=[c;x(2)-par.y_max;
        -x(2)+par.y_safe];
        if nargout == 4
            Dc.x = [Dc.x,[0 0 ;1 -1;zeros(15,2)]];%zeros(17,size(c,1));
       end
    else
        ceq = [ceq;abs(x(2)-xf(2))];
        if nargout == 4
            Dceq.x = [Dceq.x,[0; 1;zeros(15,1)]];%zeros(17,size(c,1));
       end
    end
    if isnan(xf(3))
        c=[c; x(3)-par.z_max;
        -x(3)+par.z_safe];
        if nargout == 4
            Dc.x = [Dc.x,[0 0; 0 0; 1 -1;zeros(14,2)]];%zeros(17,size(c,1));
       end
    else
        ceq = [ceq;abs(x(3)-xf(3))];
        if nargout == 4
            Dceq.x = [Dceq.x,[0;0;1;zeros(14,1)]];%zeros(17,size(c,1));
       end
    end
    
    if length(xf)>6  
    
    if isempty(par.spatial_constraints) ~= true    
        if(SPCons(xf,u,p)>0)
            error("unfeasible spatial constraint")
        end
    end
        
    if isnan(xf(9)) == false
        ceq = [ceq;abs(x(9)-xf(9))];
        if nargout == 4
            Dceq.x = [Dceq.x,[zeros(8,1);1;zeros(8,1)]];%zeros(17,size(c,1));
       end
    end   
        
    ceq = [ceq;abs(x(7)-xf(7));
           abs(x(13)-par.m*par.g);
           abs(x(8)-xf(8));
           abs(x(10:12));
           abs(x(14:20));
           abs(x(4));
           abs(x(5));
           abs(x(6));
           abs(u);
           ];
    
    if nargout == 4
        Dc.u = zeros(4,length(c));
        Dceq.u = zeros(4,length(ceq));
        Dceq.x = [Dceq.x,[zeros(6,1);1;zeros(10,1)], ...
                         [zeros(12,1);1;zeros(4,1)],...
                         [zeros(7,1);1;zeros(9,1)],...
                         [zeros(9,3);eye(3);zeros(5,3)],...
                         [zeros(13,3);eye(3);zeros(1,3)],...
                         [zeros(3,3);eye(3);zeros(11,3)],...
                         zeros(17,4)]; 
                     
        Dc.t = zeros(1,length(c));
       Dceq.t = zeros(1,length(ceq));               
    end       
       
    else    
    
    if(SPCons([xf(1:3);zeros(3,1);x(4:6)],u,p)>0)
        error("unfeasible spatial constraint")
    end    
        
    if isnan(xf(6)) == false
        ceq = [ceq;abs(x(9)-xf(6))];
       if nargout == 4
            Dceq.x = [Dceq.x,[zeros(8,1);1;zeros(8,1)]];%zeros(17,size(c,1));
       end
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
%           abs(x(14:16));
%           abs(u);
           ];
       
     if nargout == 4
        Dc.x = [Dc.x,[zeros(12,8);
                      -par.Finv(1,1:4) par.Finv(1,1:4);
                      -par.Finv(2,1:4) par.Finv(2,1:4);
                      -par.Finv(3,1:4) par.Finv(3,1:4);
                      -par.Finv(4,1:4) par.Finv(4,1:4);
                       zeros(1,8)]] ;
        Dc.u = zeros(4,length(c));
        Dceq.u = zeros(4,length(ceq));
        Dceq.x = [Dceq.x,[zeros(6,1);1;zeros(10,1)], ...
                         [zeros(7,1);1;zeros(9,1)],...
                         [zeros(3,3);eye(3);zeros(11,3)],...
                         [zeros(9,3);eye(3);zeros(5,3)]];
                     
       Dc.t = zeros(1,length(c));
       Dceq.t = zeros(1,length(ceq));              
    end    
       
    end
    
    
    end   
      
end

