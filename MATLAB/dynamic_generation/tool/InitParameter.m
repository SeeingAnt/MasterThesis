function par = InitParameter(flag)
 
    model = "sim";
    if nargin<1
        flag = 0;
    end    
    
    switch flag
        case 1
          if(model == "real")
          par.m = 0.032;
          par.J = [1.757171e-05  0               0            ; ...
                   0             1.757171e-05    0            ; ...
                   0             0               3.0261652e-05];
          elseif (model == "sim")
          par.m = 0.027;
          par.J = [1.657171e-05  0               0            ; ...
                   0             1.657171e-05    0            ; ...
                   0             0               2.9261652e-05];     
          end
          par.g = 9.81; 
        case 2
            if(model == "real")
            par.m = 0.032;
            par.J = [1.757171e-05  0               0            ; ...
                   0             1.757171e-05    0            ; ...
                   0             0               3.0261652e-05];
            elseif(model == "sim")
            par.m = 0.027;
            par.J = [1.657171e-05  0               0            ; ...
                     0              1.657171e-05     0            ; ...
                     0              0                2.9261652e-05];     
            end
            par.l = 0.046;
            par.kf = 1.71465181e-08;
            par.kd = 1.066428e-06;
            km = 7.6461e-11;
            par.F = [1,                  1,             1,          1;
                     0        -par.l,        0,      par.l;
                    -par.l,       0,         par.l,      0;
                    -km/par.kf,         km/par.kf,        -km/par.kf,     km/par.kf];
            par.Finv =[1/4, -2/(4*par.l*sqrt(2)), -2/(4*par.l*sqrt(2)), -par.kf/(4*km);
                       1/4, -2/(4*par.l*sqrt(2)),  2/(4*par.l*sqrt(2)),  par.kf/(4*km);
                       1/4,  2/(4*par.l*sqrt(2)),  2/(4*par.l*sqrt(2)), -par.kf/(4*km);
                       1/4,  2/(4*par.l*sqrt(2)), -2/(4*par.l*sqrt(2)),  par.kf/(4*km)];     
            par.F = par.Finv^(-1);
            par.step = 0.004;
            par.g = 9.81;
            par.z_safe = 0.8;
            par.z_max = 2.5;
            par.y_safe = -2.5;
            par.y_max = 2.5;
            par.x_safe = -2.5;
            par.x_max = 2.5;
            par.f1_max = 0.1189;
            par.f2_max = 0.1189;
            par.f3_max = 0.1189;
            par.f4_max = 0.1189;
            par.flips = 1;
            par.spatial_constraints =[]; %@SPCons;
        otherwise
            par.g = 9.81;
            par.flips = 1;
            if(model == "real")
            par.m = 0.032;
            par.J = [1.757171e-05  0               0            ; ...
                   0             1.757171e-05    0            ; ...
                   0             0               3.0261652e-05];
            elseif (model == "sim")
              par.m = 0.027;
              par.J = [1.657171e-05  0               0            ; ...
                        0             1.657171e-05    0            ; ...
                        0             0               2.9261652e-05];     
            end  
            par.l = 0.046;
            par.kf = 1.71465181e-08;
            par.kd = 1.066428e-06;
            km = 7.6461e-11;
            par.Finv =[1/4, -2/(4*par.l*sqrt(2)), -2/(4*par.l*sqrt(2)), -par.kf/(4*km);
                       1/4, -2/(4*par.l*sqrt(2)),  2/(4*par.l*sqrt(2)),  par.kf/(4*km);
                       1/4,  2/(4*par.l*sqrt(2)),  2/(4*par.l*sqrt(2)), -par.kf/(4*km);
                       1/4,  2/(4*par.l*sqrt(2)), -2/(4*par.l*sqrt(2)),  par.kf/(4*km)];     
            par.F = par.Finv^(-1);      
     end    
end