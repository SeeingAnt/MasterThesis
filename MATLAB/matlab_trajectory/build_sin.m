function [x,y,z,xd,yd,zd,xdd,ydd,zdd,yaw,yawd] = build_sin(st,ft,n,f)
syms t 
B = bernsteinMatrix(n,t);

j=1;
for i = linspace(st,ft,8)
 t=i;   
Sol(j,:) = eval(B);
j=j+1;
end

x = linspace(st,ft,8);
y = sin(2*pi*f*x);
b = [x',y'];

P = Sol^(-1)*b;

bezier = simplify(B*P);

fplot(bezier(1), bezier(2), [st,ft]);

freq=0.001;

x=zeros(1,1500);
y=zeros(1,1500);
yaw = zeros(1,1500);
z= zeros(1,1500);

xd = zeros(1,1500);
yd = zeros(1,1500);
zd = zeros(1,1500);
yawd = zeros(1,1500);

xdd = zeros(1,1500);
ydd = zeros(1,1500);
zdd = zeros(1,1500);

syms t

bezierd(1) = diff(bezier(1),t);
bezierd(2) = diff(bezier(2),t);

bezierdd(1) = diff(bezierd(1),t);
bezierdd(2) = diff(bezierd(2),t);

t=linspace(st,ft,1500);

x =  eval(bezier(1));
y =  eval(bezier(2));
xd =  eval(bezierd(1));
yd =  eval(bezierd(2));
xdd =  eval(bezierdd(1));
ydd =  eval(bezierdd(2));
z = ones(1,1500);