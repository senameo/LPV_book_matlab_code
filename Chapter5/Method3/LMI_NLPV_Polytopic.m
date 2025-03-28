% Author: Olivier Sename
% March 2025
%
% [Lp,P,gamma_p,epsilon_lp]=LMI_NLPV_Polytopic(A,B1,B2,Lambda,C,D,Cz)
% Description
% Function that computes an LPV polytopic observer with Hinf optimal cost,
% with a Lipschitz non linearity
% To use this function one have to define the polytopic system st:
%  xdot = [   A  B1  B2 ]  [x w  PHI]^T
%   y   =  [ C]  [x]^T + Dw
%   z   =  [ Cz]  [x]^T  
% where w=[wr,wn] is the external input , y the measured output and z the variable to be estimated
%
% The objective is to find the observer gain L  (polytopic observer) such
% that the L2 iduced gain of z/w is lower than gamma_inf (fixed a priori)
%
%  Input
%  A: list of state matrices
%  B1: external input matrix
%  B2: list of input matrices related to the non linearity PHI
%  C: Measured output matrix y=C.x
%  Cz: Performance output matrix z=Cz.x
% 
%  Output
%  Lp  : gain of the polytopic observer 
%  P : constant Lyapunov matrice P that ensures quadratic stability of the
%  estimation error
%  gamma_p  : optimal Hinf cost (L2 induced gain of z/n)
%  epsilon_lp: positive scalar
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [Lp,P,Y,gamma_p,epsilon_lp]=LMI_NLPV_Polytopic(A,B1,B2,Lambda,C,D,Cz,solver)
epsi = eps;
[nx,nw] = size(B1);
[ny,nx] = size(C);
[nz,nx] = size(Cz);
nphi=size(B2{1},2);
nb_vertices = size(B2,2);
disp('Definition of the decision variables')
P  = sdpvar(nx,nx,'symmetric');
for i=1:nb_vertices
    Y{i} = sdpvar(nx,ny,'full');
end
epsilon_lp=sdpvar(1,1,'full');
gamma2=sdpvar(1,1,'full');

disp('Definition of the LMIs')
LMIs = [epsilon_lp>=epsi];
LMIs = [LMIs,gamma2>=epsi];
LMIs = [LMIs,P>=epsi];
for i=1:nb_vertices
    M11{i}=A'*P+P*A+Y{i}*C+C'*Y{i}'+Cz'*Cz+epsilon_lp*Lambda'*Lambda; 
    M12{i}=P*B2{i};
    M13{i}=P*B1+Y{i}*D;
    M23{i}=zeros(nphi,nw);
    Hinf{i}  = [M11{i},M12{i},M13{i};
             M12{i}',-epsilon_lp*eye(nphi),M23{i};
             M13{i}',M23{i}',-gamma2*eye(nw)]; 
  LMIs = [LMIs,Hinf{i}<=-epsi];
end;

disp('Solution of the LMI problem')

ops = sdpsettings('solver',solver);
optimize(LMIs,gamma2,ops);
gamma_p = sqrt(double(gamma2));
P=value(P);
epsilon_lp=value(epsilon_lp);
for i=1:nb_vertices
    Y{i}=double(Y{i});
    Lp{i}=-P\Y{i};
end

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
disp('check constraints')
check(LMIs)
end



