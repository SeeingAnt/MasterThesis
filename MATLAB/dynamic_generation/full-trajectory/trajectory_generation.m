function [xplot,uplot,tplot,p] = trajectory_generation(trajectory,flag)

if flag == true
   
    trajectory.xf = [trajectory.xf(1:3); 0; 0; 0; trajectory.xf(4:6); 0; 0; 0; 0; 0; 0; 0;0];
    
end

xf = trajectory.xf;

save xf xf
load u0 u0
load x0 x0


par = InitParameter();
options = optimset('LargeScale','off','Display','iter');
options = optimset(options,'GradObj','off','GradConstr','off');
options = optimset(options,'MaxFunEvals',1e6);
options = optimset(options,'MaxIter',1e5);
options = optimset (options,'TolFun',1e-8);
options = optimset (options,'TolCon',2e-7);
options = optimset (options,'TolX',1e-8);
options = optimset(options,'Algorithm','interior-point','UseParallel',true);

if isempty(trajectory.p)
    optimparam.optvar = 3;
else
    optimparam.optvar = 7;
end

optimparam.objtype = [];
optimparam.ncolx = 3;
optimparam.ncolu = 3;
optimparam.li = ones(8,1)*(trajectory.initTime/8);
optimparam.tf =[];
optimparam.ui = ones(4,8).*trajectory.initu;
optimparam.par = trajectory.p;
optimparam.bdu =[]; 
optimparam.bdx = [];
optimparam.bdp = [];
optimparam.objfun = @objective_function;
optimparam.confun = @NL_bounds;
optimparam.process = @process;
optimparam.options = options;
[optimout,optimparam]=dynopt(optimparam);
save optimresults optimout optimparam
[tplot,uplot,xplot] = profiles(optimout,optimparam,1000);

x0 = xplot(end,1:end-1)';
u0 = uplot(end,:)';
p = optimout.p;

if x0(7)>= pi*2-0.02 
    x0(7) = x0(7) - 2*pi;
end    
if x0(8)>= pi*2-0.02 
    x0(8) = x0(8) -2*pi;
end    
if x0(9)>= pi*2-0.02 
    x0(9) = x0(9) - 2*pi;    
end   

if x0(7)<= -pi*2+0.02 
    x0(7) = x0(7) + 2*pi;
end    
if x0(8)<= -pi*2+0.02 
    x0(8) = x0(8) + 2*pi;
end    
if x0(9)<= -pi*2+0.02 
    x0(9) = x0(9) + 2*pi;    
end    


save x0 x0;
save u0 u0;

end