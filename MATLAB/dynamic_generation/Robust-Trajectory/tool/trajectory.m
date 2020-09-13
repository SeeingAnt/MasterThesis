function q = polynomial(start,goal,time,flag,step)
 
    [A,b] = build_matrix(start,goal,time);
    coeff_p = fliplr((A\b)');
    order = length(start);
    
    if flag
        t = 0:step:time;
    else 
        t = step:step:time;
    end
    
    coeff_v = polyder(coeff_p);
    coeff_a = polyder(coeff_v);
    q = [polyval(coeff_p,t);
        polyval(coeff_v,t);
        polyval(coeff_a,t)];
    
    if order == 4
        coeff_j = polyder(coeff_a);
        q = [polyval(coeff_p,t);
            polyval(coeff_v,t);
            polyval(coeff_a,t);
            polyval(coeff_j,t)];
    end
    
    if order == 5
        coeff_j = polyder(coeff_a);
        coeff_s = polyder(coeff_j);
        q = [polyval(coeff_p,t);
            polyval(coeff_v,t);
            polyval(coeff_a,t);
            polyval(coeff_j,t);
            polyval(coeff_s,t)];
    end
    
end
