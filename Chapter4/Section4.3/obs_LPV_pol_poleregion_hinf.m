% Author: Olivier Sename
% Jan 2025
%
% [L,P,gamma_obs,VP]=obs_LPV_pol_poleregion_hinf(A,E,C,Cz,alpha,theta,r,solver)
% Description
% Function that computes an LPV polytopic observer with Hinfinity and pole
% placement constraints (defined as LMI region)
% To use this function one have to define the polytopic system st:
%  xdot = [   A  Bw  Bu  ]  [x w u]^T
%   y   =  [ C]  [x]^T
%   z   =  [ Cz]  [x]^T  
% where w is the external input, y the measured output and z the variable
% to be estimated
%
% The objective is to find the observer gain L  (polytopic observer) such
% that:
% 1. the poles of the matrices Ai-LCi (i being the vertex i of the
% polytopic system)  are in a cone defined by:
%    - alpha : vertical exes in the LHP
%    - theta : cone angle
%    - r     : cone radius
% 2. the L2 iduced gain of z/w is minimized 
%
%  Input
%  A: list of state matrices
%  E: list of external input matrices
%  C: Measured output matrix y=C.x
%  Cz: Performance output matrix z=Cz.x
% 
%  Output
%  L  : gain of the polytopic observer 
%  gamma_obs   : optimal gamma (L2 induced gain of z/w)
%  P : constant Lyapunov matrice P that ensures quadratic stability of the
%  estimation error
% VP: eigencalues of the matrices Ai-LCi (i being the vertex i of the
% polytopic system) 


function [L,P,gamma_obs,VP]=obs_LPV_pol_poleregion_hinf(A,E,C,Cz,alpha,theta,r,solver)

epsi = 1e-10;
[nx,nw] = size(E{1});
[ny,nx] = size(C);
[nz,nx] = size(Cz);
nb_sommet = size(A,2);

disp('Definition of the decision variables')
P  = sdpvar(nx,nx);
for i=1:nb_sommet
    Y{i} = sdpvar(nx,ny,'full');
    AtP{i} = A{i}'*P - C'*Y{i}';
    PA{i}  = P*A{i} - Y{i}*C;
end
gamma = sdpvar(1,1,'full');%1;%

LMIs = [P >= epsi];
for i=1:nb_sommet
    LMIs = [LMIs, [A{i}'*P + P*A{i} - Y{i}*C - C'*Y{i}' + 2*alpha*P <= -epsi]];
    LMIs = [LMIs, [-r*P, PA{i}; AtP{i}, -r*P] <= -epsi];
    LMIs = [LMIs, [sin(theta)*(PA{i} + AtP{i}), cos(theta)*(PA{i} - AtP{i});...
        cos(theta)*(AtP{i} - PA{i}), sin(theta)*(PA{i} + AtP{i})] <= -epsi];
    M11{i}=A{i}'*P+P*A{i}' - Y{i}*C - C'*Y{i}'; 
    M21{i}=E{i};
    M31{i}=P*Cz';
    M32{i}=zeros(nw,nz);
    H1{i}  = [M11{i},M21{i},M31{i};
             M21{i}',-gamma*eye(nw),M32{i};
             M31{i}',M32{i}',-gamma*eye(nz)]; 
    LMIs = [LMIs,H1{i}<=-epsi];
end;

ops = sdpsettings('solver',solver);
optimize(LMIs,gamma,ops);
gamma_obs = double(gamma);

for i=1:nb_sommet
    L{i}=value(P)\value(Y{i});
end

for i=1:nb_sommet
    VP{i} = (eig(A{i}-L{i}*C));
end

VP_ = cell2mat(VP);
VP_ = num2cell(VP_);

fprintf('\n')
Les_VP = cell2table(VP_);
disp(Les_VP)

VP_P = eig(value(P));
for i=1:size(VP_P,1)
    if (VP_P(i) <= 0)
        disp('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
        disp('Error, Lyapunov function are non positive definite')
        disp('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
    elseif (VP_P(i) <= 1e-8)
        disp('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
        disp('Warning : VP of P are very small')
        disp('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
    end
end

end