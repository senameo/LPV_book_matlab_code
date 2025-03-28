% Author: Thanh-Phong PHAM & Olivier Sename
% March 2025
%
% Lg,P,Y,gamma_g,epsilon_lg]=LMI_NLPV_Griding(A,B1,B,Lambda,C,D,Cz,rho,maxdrho,solver)
% Description
% Function that computes an LPV grid-based observer with Hinf optimal cost,
% with a Lipschitz non linearity
% To use this function one have to define the polytopic system st:
%  xdot = [   A  B1  B2 ]  [x w  PHI]^T
%   y   =  [ C]  [x]^T + Dw
%   z   =  [ Cz]  [x]^T  
% where w=[wr,wn] is the external input , y the measured output and z the variable to be estimated
%
% The objective is to find the observer gain L(rho)  (grid-based observer) such
% that the L2 iduced gain of z/w is lower than gamma_inf (fixed a priori)
%
%  Input
%  A: list of state matrices
%  B1: external input matrix
%  B2: list of input matrices related to the non linearity PHI
%  C: Measured output matrix y=C.x
%  Cz: Performance output matrix z=Cz.x
%  rho: grid of frozen values of the varying parameyer rho
%  maxdroh: bound on the derivative of the varying parameter, i.e |d
%  rho/dt|< maxrdrho
% 
%  Output
%  Lg  : list of gridded LPV observer gains Lg(rho)
%  P : list of Parameter dependent Lyapunov matrix P(rho) that ensures stability of the
%  estimation error
%  gamma_g  : optimal Hinf cost (L2 induced gain of z/n)
%  epsilon_lg: positive scalar
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [Lg,P,Y,gamma_g,epsilon_lg]=LMI_NLPV_Griding(A,B1,B,Lambda,C,D,Cz,rho,maxdrho,solver)
% matrices size
[nx,nx]=size(A);
[nx,nw] = size(B1);
[ny,nx] = size(C);
[nz,nx] = size(Cz);
nphi=size(B{1},2);
disp('declaration of decision variables')
%% Lyapunov matrix: P(rho)=P0 + rho*P1
P0=sdpvar(nx,nx,'symmetric');
P1=sdpvar(nx,nx,'symmetric');
 for i = 1:size(rho,2)
    P{i} =P0+rho{i}*P1; 
end;
%% Derivative dP/drho
for i = 1:size(rho,2)
    dP{i} = P1; 
end;
%% Y(rho) parametrized decision variable
Y0=sdpvar(nx,ny);
Y1=sdpvar(nx,ny);
Y2=sdpvar(nx,ny);
for i = 1:size(rho,2)
    Y{i} = Y0+rho{i}*Y1+(rho{i}^2)*Y2; 
end;
%% constant scalars
epsilon_lg = sdpvar(1,1,'full'); 
gamma_g = sdpvar(1,1,'full');
%%
disp('Definiton of the LMIs')
%% LMI epsilon_lg>0 
F = [epsilon_lg>=eps]; 
%% LMI gamma_g>0 
F = [F,gamma_g>=eps]; 
%% LMIs for each grid point
for i=1:size(rho,2)
    F = [F,P{i}>=eps];
 % LMI from the parameter dependent Bounded Real Lemma with  drho/dt< maxrdrho
    M11{i}=A'*P{i}+P{i}*A+Y{i}*C+C'*Y{i}'+Cz'*Cz+epsilon_lg*Lambda'*Lambda+dP{i}*maxdrho; 
    M12{i}=P{i}*B{i};
    M13{i}=P{i}*B1+Y{i}*D;
    H1{i}  = [ M11{i}        M12{i}                  M13{i};
               M12{i}'      -epsilon_lg*eye(nphi)           zeros(nphi,nw);
               M13{i}'        zeros(nw,nphi)         -gamma_g*eye(nw)]; 
    F = [F,H1{i}<=-eps];
% LMI from the parameter dependent Bounded Real Lemma with  drho/dt> -maxrdrho
    N11{i}=A'*P{i}+P{i}*A+Y{i}*C+C'*Y{i}'+Cz'*Cz+epsilon_lg*Lambda'*Lambda-dP{i}*maxdrho; 
    N12{i}=P{i}*B{i};
    N13{i}=P{i}*B1+Y{i}*D;
    H2{i}  = [ N11{i}        N12{i}                   N13{i};
               N12{i}'     -epsilon_lg*eye(nphi)          zeros(nphi,nw);
               N13{i}'        zeros(nw,nphi)          -gamma_g*eye(nw)]; 
    F = [F,H2{i}<=-eps];
   
end
disp('Solution of the LMI problem')
ops      = sdpsettings('solver',solver);
solution = optimize(F,gamma_g,ops);
%
gamma_g = double(gamma_g);
gamma_g=sqrt(gamma_g);
epsilon_lg= double(epsilon_lg);

%%% Extract numerical solution
for i = 1:size(rho,2)
P{i}=double(P{i});
Y{i}=double(Y{i});
Lg{i}=-P{i}\Y{i};
end

% Note that in case you would like to get the decision variables you can
% modify the function as
%[Lg,P,P0,P1,Y,Y0,Y1,Y2,gamma_g,epsilon_lg]=LMI_NLPV_Griding(A,B1,B,Lambda,C,D,Cz,rho,maxdrho,solver)
% and use  when extracting the numerical solution
% P0=double(P0);
% P1=double(P1);
% Y0=double(Y0);
% Y1=double(Y1);
% Y2=double(Y2);


%%% P is positive definite
for i = 1:size(rho,2)
    if (all(eig(P{i}) > 0) ~= 1)
        disp(strcat(num2str(i),'Error, Lyapunov function are non positive definite'))
    end
end;
disp('check constraints')
check(F)
end
