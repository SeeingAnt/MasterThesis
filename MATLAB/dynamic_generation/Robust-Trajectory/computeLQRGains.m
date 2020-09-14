function K = computeLQRGains(EulerAngles,omega,f)
%  computeLQRGains generates the time-varying LQR gains for the infinite
%  horixon problem.
%          
%   K = computeLQRGains(EulerAngles,omega) generates the gain K (3N x 6)
%   through the matrices A and B computed as
%                                      
%           A = [-skew(omega(:,i))                                         eye(3);
%                 zeros(3)         skew(par.J*omega(:,i))-skew(omega(:,i))*par.J];
%           B = [zeros(3); eye(3)];
%

K =zeros(4*size(EulerAngles,2),12);
par = InitParameter();
% 
% Q=eye(12).*[1e3 1e3 1e3 1e3 1e3 1e3 1e3 1e3 1e3 1e3 1e3 1e3];
% Rb=eye(4).*[3e2 4e1 4e1 4e1];
% 
% P=eye(12).*[1e4 1e4 1e4 1e4 1e4 1e4 1e4 1e4 1e4 1e4 1e4 1e4];

Q=eye(12).*[1e3 1e3 1e3 1e3 1e3 1e3 1e3 1e3 1e3 1e3 1e3 1e3];
Rb=eye(4).*[1e1 4e0 4e0 4e0];

P=eye(12).*[1e4 1e4 1e4 1e4 1e4 1e4 1e4 1e4 1e4 1e4 1e4 1e4];


% phase margin to guarantie convergence also for time delay inputs
index = 1:size(EulerAngles,2);
        for i=flip(index)

            R=eul2rotm([EulerAngles(3,i) EulerAngles(2,i) EulerAngles(1,i)]);    
          
            A = [ zeros(3) eye(3) zeros(3) zeros(3);
                  zeros(3) zeros(3) -f(i)*R*skew([0;0;1])/par.m zeros(3);
                  zeros(3) zeros(3) -skew(omega(:,i)) eye(3);
                  zeros(3) zeros(3)  zeros(3)  par.J\(skew(par.J*omega(:,i))-skew(omega(:,i))*par.J)];
              
            B = [zeros(3,1) zeros(3);R*[0;0;1]/par.m zeros(3);zeros(3,1) zeros(3);zeros(3,1) par.J\eye(3)];

%             sysd = ss(A,B,eye(12),zeros(12,4));
%             sysd = c2d(sysd,0.002);
            sysd.A = eye(12)+A*0.001;
            sysd.B = B*0.001;
            
            
            [K(4*(i-1)+1:4*i,:),P]  =  FiniteHorizonLQR(sysd.A,sysd.B,Q,Rb,P,1);
%            [K(4*(i-1)+1:4*i,:)]  = lqr(A,B,Q,Rb);
        end        

end 

function [k,P] = FiniteHorizonLQR(A,B,Q,R,P,N)

  P = Q + A'*P*A -A'*P*B*(R+B'*P*B)^(-1)*B'*P*A; 
  k=(R + B'*P*B)^(-1)*B'*P*A;
  
end

function R = skew(w)
%SKEW  generates a skew-symmetric matrix given a vector w
%
%	R = SKEW(w)
%
% See also: ROTAXIS, SKEWEXP, SKEWCOORDS.

% $Id: skew.m,v 1.1 2009-03-17 16:40:18 bradleyk Exp $
% Copyright (C) 2005, by Brad Kratochvil

  if 3 ~= size(w,1),
    error('SCREWS:skew','vector must be 3x1')
  end
  
  if isnumeric(w),
    R = zeros(3,3);
  end
  
  R(1,2) = -w(3);
  R(1,3) =  w(2);
  R(2,3) = -w(1);

  R(2,1) =  w(3);
  R(3,1) = -w(2);
  R(3,2) =  w(1);

%   R(1,1) = 0;
%   R(2,2) = 0;
%   R(3,3) = 0;
  
end

%             A = [ -skew(omega(:,i)) eye(3);
%                    zeros(3)  par.J\(skew(par.J*omega(:,i))-skew(omega(:,i))*par.J)];
% 
%             B = [zeros(3); par.J\eye(3)];