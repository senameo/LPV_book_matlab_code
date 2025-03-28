% Author: Olivier Sename
% March 2025
%
% L,P,gamma_2,VP]=obs_LPVpol_pole_h2_hinf(A,B1,C,N,Cz,gamma_inf,alpha,theta,r,solver)
% Description
% Function that computes an LPV polytopic observer with gH2 optimal cost, given Hinfinity and pole
% placement constraints (defined as LMI region)
% To use this function one have to define the polytopic system st:
%  xdot = [   A  B1  ]  [x w]^T
%   y   =  [ C]  [x]^T + Nn
%   z   =  [ Cz]  [x]^T  
% where w is the external input, y the measured output, n the measurment
% noise and z the variable to be estimated
%
% The objective is to find the observer gain L  (polytopic observer) such
% that:
% 1. the poles of the matrices Ai-LCi (i being the vertex i of the
% polytopic system)  are in a cone defined by:
%    - alpha : vertical exes in the LHP
%    - theta : cone angle
%    - r     : cone radius
% 2. the L2 iduced gain of z/w is lower than gamma_inf (fixed a priori)
% 3. the generalized H2 norm from the measurement noise n to z is
% minimized
%
%  Input
%  A: list of state matrices
%  B1: list of external input matrices
%  C: Measured output matrix y=C.x
%  Cz: Performance output matrix z=Cz.x
% 
%  Output
%  L  : gain of the polytopic observer 
%  gamma_2   : optimal gH2 (L2 induced gain of z/n)
%  P : constant Lyapunov matrice P that ensures quadratic stability of the
%  estimation error
% VP: eigencalues of the matrices Ai-LCi (i being the vertex i of the
% polytopic system) 

function [L,P,gamma_2,VP]=obs_LPVpol_pole_h2_hinf(A,B1,C,N,Cz,gamma_inf,alpha,theta,r,solver)
epsi = 1e-10;
[nx,nw] = size(B1{1});
[ny,nx] = size(C);
[nz,nx] = size(Cz);
nb_vertices = size(A,2);
disp('check')

P  = sdpvar(nx,nx,'symmetric');
for i=1:nb_vertices
    Y{i} = sdpvar(nx,ny,'full');
    AtP{i} = A{i}'*P - C'*Y{i}';
    PA{i}  = P*A{i} - Y{i}*C;
end
gamma_22=sdpvar(1,1,'full');%1; gamma_2^2

LMIs = [P >= epsi];
LMIs = [LMIs,gamma_22>=epsi];
for i=1:nb_vertices
    LMIs = [LMIs, [PA{i} + AtP{i} + 2*alpha*P <= -epsi]];
    LMIs = [LMIs, [-r*P, PA{i}; AtP{i}, -r*P] <= -epsi];
    LMIs = [LMIs, [sin(theta)*(PA{i} + AtP{i}), cos(theta)*(PA{i} - AtP{i});...
        cos(theta)*(AtP{i} - PA{i}), sin(theta)*(PA{i} + AtP{i})] <= -epsi];
    M11{i}=PA{i} + AtP{i}; 
    M21{i}=B1{i};
    M31{i}=P*Cz';
    M32{i}=zeros(nw,nz);
    Hinf{i}  = [M11{i},M21{i},M31{i};
             M21{i}',-gamma_inf*eye(nw),M32{i};
             M31{i}',M32{i}',-gamma_inf*eye(nz)]; 
    LMIs = [LMIs,Hinf{i}<=-epsi];
    H2{i}  = [M11{i},-Y{i}*N;
             -N'*Y{i}',-eye(ny)]; 
    LMIs = [LMIs,H2{i}<=-epsi];
 H2g= [P Cz';Cz gamma_22*eye(nz)]
LMIs = [LMIs,H2g>=epsi];

end;

ops = sdpsettings('solver',solver);
optimize(LMIs,gamma_22,ops);
gamma_2 = sqrt(double(gamma_22));

for i=1:nb_vertices
    L{i}=value(P)\value(Y{i});
end

for i=1:nb_vertices
    VP{i} = (eig(A{i}-L{i}*C));
end

VP_ = cell2mat(VP);
VP_ = num2cell(VP_);

fprintf('\n')
Les_VP = cell2table(VP_);
disp(Les_VP)
P=value(P);

VP_P = eig(value(P));
for i=1:size(VP_P,1)
    if (VP_P(i) <= 0)
        disp('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
        disp('Error, Lyapunov function are non positive definite')
        disp('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
  elseif (abs(VP_P(i)) <= 1e-8)
        disp('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
        disp('Warning : VP of P are very small')
        disp('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
    end
end

end