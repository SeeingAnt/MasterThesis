function [c,ceq] = SPCons(x,u,p)

%        c =[-x(3)+0.9;
%             x(3)-1.1];
        c=[x(1)-0.3;
          -x(1)-0.3;
           x(2)-0.3;
          -x(2)-0.3;];
        ceq=[];
%       ceq=[abs(x(3)-(-p(1)*(x(2)+x(1)-p(2))^2+p(3)))];
    
        
%        R=eul2rotm([x(9) x(8) x(7)]);
%        q1 = R * [0.046;0;0];
%        q2 = R * [0;0.046;0];
%        q3 = R * [0;0;0.02];

%        c = [(x(1)-0.3+q1(1))^2/(0.035)^2-(x(2)-0.5+q1(2))^2/(0.02^2)+(x(3)-2+q1(3))^2/(0.12)^2-1;
%             (x(1)-0.3-q1(1))^2/(0.025)^2-(x(2)-0.5-q1(2))^2/(0.02^2)+(x(3)-2-q1(3))^2/(0.12)^2-1;
%             (x(1)-0.3+q2(1))^2/(0.025)^2-(x(2)-0.5+q2(2))^2/(0.02^2)+(x(3)-2+q2(3))^2/(0.12)^2-1;
%             (x(1)-0.3-q2(1))^2/(0.025)^2-(x(2)-0.5-q2(2))^2/(0.02^2)+(x(3)-2-q2(3))^2/(0.12)^2-1;
%             (x(1)-0.3+q3(1))^2/(0.025)^2-(x(2)-0.5+q3(2))^2/(0.02^2)+(x(3)-2+q3(3))^2/(0.12)^2-1];

%        A = [1 0 1; 0 1 0;-1 0 1]*[1/(0.06)^2 0 0; 0 -1/(0.001)^2 0; 0 0 1/(0.12)^2]*[1 0 1; 0 1 0;-1 0 1]^(-1);
%        c = [[(x(1)-0.3+q1(1));(x(2)-0.5+q1(2));(x(3)-2+q1(3))]'*A*[(x(1)-0.3+q1(1));(x(2)-0.5+q1(2));(x(3)-2+q1(3))]-1;
%             [(x(1)-0.3-q1(1));(x(2)-0.5-q1(2));(x(3)-2-q1(3))]'*A*[(x(1)-0.3-q1(1));(x(2)-0.5-q1(2));(x(3)-2-q1(3))]-1;
%             [(x(1)-0.3+q2(1));(x(2)-0.5+q2(2));(x(3)-2+q2(3))]'*A*[(x(1)-0.3+q2(1));(x(2)-0.5+q2(2));(x(3)-2+q2(3))]-1;
%             [(x(1)-0.3-q2(1));(x(2)-0.5-q2(2));(x(3)-2-q2(3))]'*A*[(x(1)-0.3-q2(1));(x(2)-0.5-q2(2));(x(3)-2-q2(3))]-1;
%             [(x(1)-0.3+q3(1));(x(2)-0.5+q3(2));(x(3)-2+q3(3))]'*A*[(x(1)-0.3+q3(1));(x(2)-0.5+q3(2));(x(3)-2+q3(3))]-1];
% 
%        c = c/1e5; 
end

%
% Draw constraints
%
% [X,Y,Z] = meshgrid(-10:0.05:10,-10:0.05:10,-10:0.05:10);
% A = [1 0 1; 0 1 0;-1 0 1]*[1/(0.06)^2 0 0; 0 -1/(0.001)^2 0; 0 0 1/(0.12)^2]*[1 0 1; 0 1 0;-1 0 1]^(-1);      
% V = A(1,1)*(X-0.3).^2 + A(1,2)*(X-0.3).*(Y-0.5) + A(1,3)*(X-0.3).*(Z-2.1) + ...
%     A(2,1)*(Y-0.5).*(X-0.3) + A(2,2)*(Y-0.5).^2 + A(2,3)*(Y-0.5).*(Z-2.1) + ...
%     A(3,1)*(Z-2.1).*(X-0.3) + A(3,2)*(Z-2.1).*(Y-0.5) + A(3,3)*(Z-2.1).^2 ;
% p=patch(isosurface(X,Y,Z,V,1)); % This is the key step. It involves getting the part of the volume corresponding to the surface defined by the equation
% set(p,'FaceAlpha',0.2,'EdgeColor','none');
% daspect([1 1 1])
% view(3);
% xlim([-0.2 0.6])
% ylim([-0.5 1.8])
% zlim([0 2.5])

% [X,Y,Z] = meshgrid(-10:0.05:10,-10:0.05:10,-10:0.05:10);
% V = (X-0.3).^2/(0.05^2) - (Y-0.5).^2/(0.02^2) + (Z-2).^2/(0.12^2) ;
% p=patch(isosurface(X,Y,Z,V,1)); % This is the key step. It involves getting the part of the volume corresponding to the surface defined by the equation
% set(p,'FaceAlpha',0.2,'EdgeColor','none');
% daspect([1 1 1])
% view(3);
% xlim([-0.2 0.6])
% ylim([-0.5 1.8])
% zlim([0 2.5])
%
%        