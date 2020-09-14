function [alpha,Psi,PsiSqNorm,P] = Jacobi_PC(M,p_order,params)
%% multi-dimensional Jacobi PC 
%{
--------------------------------------------------------------------------
Created by:                       Date:           Comment:
Felipe Uribe                      Oct/2014        ---
furibec@unal.edu.co                   
Universidad Nacional de Colombia 
Manizales Campus
--------------------------------------------------------------------------
M         % number of dimensions
p_order   % order of PC
params    % [alpha,beta] parameters of Jacobi pols
--------------------------------------------------------------------------
Based on:
1."Numerical methods for stochastic computations A spectral method approach"
   D. Xiu (2010) - Princeton university press
2."Stochastic finite element methods and reliability"
   B. Sudret and A. Der Kiureghian. State of the art report.
--------------------------------------------------------------------------
%}

%% Calculate the size of Psi
P = 1;   % total number of PC 
for s = 1:p_order
   P = P + (1/factorial(s))*prod(M+(0:s-1));   % Eq. (3.54) Ref (1)
end

%% Calculate 1D Jacobi polynomials
% here order 4 is equivalent to order 3 since MATLAB indexes from 1
alp = params(1);
bet = params(2);
syms xi;
Ja    = cell(p_order,1);
%Ja{1} =  1;   
%Ja{2} = (1/2)*((alp-bet)+(alp+bet+2)*xi);
%Ja{2} = (1/8)*(4*(alp+1)*(alp+2) + 4*(alp+bet+3)*(alp+2)*(xi-1) + (alp+bet+3)*(alp+bet+4)*(xi-1)^2);
for n = 0:p_order
   % using rodrigues formula
   num     = (-1^n)/((2^n)*factorial(n));
   den     = ((1-xi)^alp)*((1+xi)^bet);
   Ja{n+1} = factor((num/den)*diff(((1-xi)^(n+alp))*((1+xi)^(n+bet)),xi,n));
   if mod(n,2)==0
      Ja{n+1} = -Ja{n+1};
   end
end
% or diff(((1-xi)^alp)*((1+xi)^bet)*((1-xi^2)^n),xi,n)

%% Define the number of RVs in sym
x = cell(1,M);
J = cell(p_order,M);
for j = 1:M
   x{j} = sym(sprintf('xi_%d',j));
   for i = 1:p_order+1
      J{i,j} = subs(Ja{i},xi,x{j});
   end
end

%% M-dimensional PC computation
Psi    = cell(P,1);
alpha  = multi_index(M,p_order);  % create the multi-index
for i = 2:P+1
   mult = 1;
   for j = 1:M
      mult = mult*J{alpha(i-1,j)+1,j};
   end
   Psi{i-1} = mult;
end

%% Compute the square norm of the PC
PsiSqNorm  = prod(factorial(alpha), 2);

%% show results
fprintf('Number of random variables (K-L terms): M = %d \n',M);
fprintf('Order of the polynomial chaos: p = %d \n',p_order);
fprintf('Total number of polynomial chaos: p = %d \n\n',P);
for k = 1:P
   fprintf('j = %d \t',k-1);
   fprintf('E[Psi^2] = %d \t\t',PsiSqNorm(k));
   fprintf('Psi   = %s \n',char(Psi{k}));
end

%%END