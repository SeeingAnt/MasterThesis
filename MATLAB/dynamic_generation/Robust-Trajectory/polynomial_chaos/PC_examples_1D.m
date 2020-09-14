%% PC applications 1D
%{
--------------------------------------------------------------------------
Created by:                       Date:           Comment:
Felipe Uribe                      Oct/2014        ---
furibec@unal.edu.co                   
Universidad Nacional de Colombia 
Manizales Campus
--------------------------------------------------------------------------
Based on:
1."Stochastic finite elements A spectral approach"
   R. Ghanem and P.D. Spanos. Rev edition 2012. Dover publications Inc.
2."Stochastic finite element methods and reliability"
   B. Sudret and A. Der Kiureghian. State of the art report.
3."Numerical methods for stochastic computations"
   D. Xiu. (2010). Princeton university press.
--------------------------------------------------------------------------
%}
clear; clc; format long g; close all;

%% initial parameters
p_order = 6;                    % order of the polynomial chaos: 0,1,2...
example = 4;                    % choose example: 1, 2, 3, 4, or 5
xi_cdf  = @(x) normcdf(x,0,1);  % gaussian measure: homogeneous chaos

%% define target distributions
switch example
   % Example "Exponential" - Laguerre pols are exactly
    case 1
        mu     = 1;                      % parameters
        u_pdf  = @(x) exppdf(x,mu);      % target pdf 
        u_cdf  = @(x) expcdf(x,mu);      % cdf
        u_icdf = @(p) expinv(p,mu);      % inv cdf
        aa = 0;   bb = 6;                % xlim
    % Example "Gumbel"            
    case 2 
        mu = 1;   sigma = 2;             % parameters
        u_pdf  = @(x) evpdf(x,mu,sigma); % target pdf
        u_cdf  = @(x) evcdf(x,mu,sigma); % cdf
        u_icdf = @(p) evinv(p,mu,sigma); % inv cdf
        aa = -10;   bb = 6;              % xlim
    % Example "beta"     
    case 3
        alpha = 5;   beta = 2;               % bimodal for a=b<1
        u_pdf  = @(x) betapdf(x,alpha,beta);   % target pdf
        u_cdf  = @(x) betacdf(x,alpha,beta);   % cdf
        u_icdf = @(p) betainv(p,alpha,beta);   % inv cdf
        aa = 0;   bb = 1.5;                      % xlim
    % Example "Gaussian"     
    case 4
        mu = 2;   sigma = 1;
        u_pdf  = @(x) normpdf(x,mu,sigma);   % target pdf
        u_cdf  = @(x) normcdf(x,mu,sigma);   % cdf
        u_icdf = @(p) norminv(p,mu,sigma);   % inv cdf
        aa = -4;   bb = 4;                   % xlim
    % Example: sum of Gaussians
    case 5 
        aa  = -2;  bb     = 11;   % xlim
        mu1 = 1;   sigma1 = 1;    % params 1st Gaussian
        mu2 = 5;   sigma2 = 2;    % params 2nd Gaussian
        u_pdf  = @(x) normpdf(x,mu1,sigma1) + normpdf(x,mu2,sigma2);   % target pdf
        u_cdf  = @(x) normcdf(x,mu1,sigma1) + normcdf(x,mu2,sigma2);   % cdf
        % create inverse cdf function of the mixture
        xx     = aa:0.01:bb;
        y      = u_pdf(xx);
        u_pdf  = @(x) interp1(xx,y/trapz(xx,y),x);   % normalized
        y2     = u_cdf(xx);
        u_cdf  = @(x) interp1(xx,(y2-min(y2))/(max(y2)-min(y2)),x);   % cdf
        u_icdf = @(p) interp1((y2-min(y2))/(max(y2)-min(y2)),xx,p);   % inv cdf
   otherwise
      error('You must choose between 1, 2, 3, 4 or 5');
end

%% obtain the gPC Hermite expressions
[~,PsiPol,~,PsiSqNorm,P] = Hermite_PC(1,p_order);  % 1D Hermite polynomials

%% find the PC deterministic coefficients using proyection approach
N          = 6;
[xi_q,w_q] = gauss_quad(N,'he_prob');   % Gauss-Hermite quadrature
u_k        = zeros(P,1);

for k = 1:P
   for j = 1:N
      u_k(k) = u_k(k) + (u_icdf(xi_cdf(xi_q(j)))*...
               polyval(sym2poly(PsiPol{k}),xi_q(j))*w_q(j));   % Ref 3. Eq.(5.16) 
   end
end
u_k = u_k./PsiSqNorm;

%% approximate the random variable u by PCE
N       = 1e4;          % number of samples
xi      = randn(N,1);   % homogeneous chaos
u_tilde = zeros(N,1);
for k = 1:P
   u_tilde = u_tilde + u_k(k)*polyval(sym2poly(PsiPol{k}),xi);
end

[n1,x1]     = hist(u_tilde,ceil(sqrt(N)));           % histogram
[PC_pdf,x2] = ksdensity(u_tilde);                    % approx pdf
[PC_cdf,x3] = ksdensity(u_tilde,'function','cdf');   % approx cdf

%% plots
xp = aa:0.01:bb;
set(0,'defaultTextInterpreter','latex'); 

% PDF
figure(1); hold on;
bar(x1,n1/trapz(x1,n1),'c','EdgeColor',[0,0,0]);
plot(xp,u_pdf(xp),'b-','LineWidth',1);
plot(x2,PC_pdf,'r--','LineWidth',1);  % 'k-+' 'g-.' 'r--'
grid minor; axis tight; set(gca,'FontSize',12);
xlabel('$$\xi$$','FontSize',15);
ylabel('$$f_{\alpha}(\xi)$$','FontSize',15);
legend('Hist','Exact',sprintf('%d-order PC approx',p_order),'Location','Best');

% CDF
figure(2); 
% cdf tail beg
subplot(3,4,1:2); hold on;
plot(xp,u_cdf(xp),'b-','LineWidth',2);
plot(x3,PC_cdf,'r--','LineWidth',2); 
grid minor; ylim([0 0.05]); set(gca,'FontSize',22);
xlabel('$$\xi$$','FontSize',23);
ylabel('$$F_{\alpha}(\xi)$$','FontSize',20);
title('Tail at beginning','FontSize',25);
% cdf tail end
subplot(3,4,3:4); hold on;
plot(xp,u_cdf(xp),'b-','LineWidth',2); 
plot(x3,PC_cdf,'r--','LineWidth',2);
grid minor; ylim([0.95 1]); set(gca,'FontSize',22);
xlabel('$$\xi$$','FontSize',23);
ylabel('$$F_{\alpha}(\xi)$$','FontSize',24);
title('Tail at end','FontSize',25);
% cdf complete
subplot(3,4,5:12);
plot(xp,u_cdf(xp),'b-','LineWidth',2); hold on;
plot(x3,PC_cdf,'r--','LineWidth',2);
grid minor; axis tight; set(gca,'FontSize',22);
xlabel('$$\xi$$','FontSize',27);
ylabel('$$F_{\alpha}(\xi)$$','FontSize',27);
legend('Exact',sprintf('%d-order PC approx',p_order),'Location','Best');

%%END