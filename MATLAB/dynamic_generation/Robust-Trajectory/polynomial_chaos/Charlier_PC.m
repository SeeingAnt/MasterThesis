function [alpha,Psi_s,PsiSqNorm,P] = Charlier_PC(M,p_order,param)
%% multi-dimensional Charlier PC 
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
params    % a, parameter of Charlier pols
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

%% Calculate 1D Charlier polynomials
% symbolic
a = param;
syms xi;
Ch_s    = cell(p_order,1);
Ch_s{1} = sym(1);   
Ch_s{2} = -(a-xi)/a;
for j = 1:p_order
   % using recursive formula
   Ch_s{j+1} = simplify((a^-1)*xi*subs(Ch_s{j},xi-1)-Ch_s{j});
end

%% Define the number of RVs
x    = cell(1,M);
CH_s = cell(p_order,M);   % Hermite polinomial for each dimension syms
for j = 1:M
   x{j} = sym(sprintf('xi_%d',j));
   for i = 1:p_order+1
      CH_s{i,j} = subs(Ch_s{i},xi,x{j});
   end
end

%% M-dimensional PC computation
Psi_s  = cell(P,1);   % symbolic version
alpha  = multi_index(M,p_order);  % create the multi-index
for i = 2:P+1
   mult_s = 1;
   for j = 1:M
      mult_s = mult_s*CH_s{alpha(i-1,j)+1,j};
   end
   Psi_s{i-1} = mult_s;
end

%% Calculate the square norm
PsiSqNorm  = prod(factorial(alpha),2);

%% show results
fprintf('Number of random variables (K-L terms): M = %d \n',M);
fprintf('Order of the polynomial chaos: p = %d \n',p_order);
fprintf('Total number of polynomial chaos: p = %d \n\n',P);
for k = 1:P
   fprintf('j = %d \t',k-1);
   fprintf('E[Psi^2] = %d \t\t',PsiSqNorm(k));
   fprintf('Psi   = %s \n',char(Psi_s{k}));
end

%%END