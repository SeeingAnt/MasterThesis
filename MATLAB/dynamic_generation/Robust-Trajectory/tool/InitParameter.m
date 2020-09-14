function par = InitParameter(flag)
 
    
    if nargin<1
        flag = 0;
    end    
    
    switch flag
        case 1
           par.xi_q = [-1.7321;0;1.7321];  
           par.m = 0.027;
           par.J = [1.657171e-05  0               0            ; ...
                    0             1.657171e-05    0            ; ...
                    0             0               2.9261652e-05];
           par.g = 9.81; 
           par.kf = 1.71465181e-08;
           par.kd = 1.066428e-06;
           par.l = 0.046;
           km = 7.6461e-11;
           par.Finv =[1/4, -2/(4*par.l*sqrt(2)), -2/(4*par.l*sqrt(2)), -par.kf/(4*km);
                       1/4, -2/(4*par.l*sqrt(2)),  2/(4*par.l*sqrt(2)),  par.kf/(4*km);
                       1/4,  2/(4*par.l*sqrt(2)),  2/(4*par.l*sqrt(2)), -par.kf/(4*km);
                       1/4,  2/(4*par.l*sqrt(2)), -2/(4*par.l*sqrt(2)),  par.kf/(4*km)];
        case 2
            par.PsiPol = {1,[1 0],[1 0 -1],[1 0 -3 0]};
            par.PsiSqNorm = [1;1;2;6];
            par.PsiPol2 = {1,[1 0],[1 0 -1],[1     0    -3     0]};
            par.Alpha = [0,0;1,0;0,1;2,0;1,1;0,2,;3,0;2,1;1,2;0,3];
            par.PsiSqNorm2 = [1;1;1;2;1;2;6;2;2;6];
            par.xi_q = [-1.7321;0;1.7321]; 
            par.w_q = [0.1667; 0.6667; 0.1667];
            par.m = 0.027;
            par.l = 0.046;
            par.kf = 1.71465181e-08;
            par.kd = 1.066428e-06;
            km = 7.6461e-11;
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
            par.f1_max = 0.1602*0.90;
            par.f2_max = 0.1602*0.90;
            par.f3_max = 0.1602*0.90;
            par.f4_max = 0.1602*0.90;
            par.flips = 1;
            par.spatial_constraints = [];%z@SPCons;
        otherwise
            par.g = 9.81;
            par.m = 0.027;
            par.flips = 1;
            par.J = [1.657171e-05  0               0            ; ...
                    0             1.657171e-05    0            ; ...
                    0             0               2.9261652e-05];
            par.l = 0.046;
            par.PsiPol = {1,[1 0],[1 0 -1],[1 0 -3 0]};
            par.PsiSqNorm = [1;1;2;6];
            par.PsiPol2 = {1,[1 0],[1 0 -1],[1     0    -3     0]};
            par.Alpha = [0,0;1,0;0,1;2,0;1,1;0,2,;3,0;2,1;1,2;0,3];
            par.PsiSqNorm2 = [1;1;1;2;1;2;6;2;2;6];
            par.xi_q = [-1.7321;0;1.7321]; 
            par.w_q = [0.1667; 0.6667; 0.1667];
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