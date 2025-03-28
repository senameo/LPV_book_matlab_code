% Author: Olivier Sename
% Jan 2025
% Description
% [F,X] = lmiHinfStateFeedbackRobust_expstab(listP,nstate,ncon,beta,solver)
% Function that computes a robust state feedback controller for a polytopic system solving an LMI problem. 
% To use this function one have to create the polytopic plant models at
% the vertices of the polytope
% System matrices A, B2, C, D, given at the vertices of the polytope
% remark: since we consider a contstant (robust) state feedback gain, B2
% can be paramter dependent
% Function use:
% Input
%  listP  : list of plants (at the vertices of the polytope)
%  ncon   : number of control signals
%  nstate : number of state variables
%  beta   : decay rate constraint 
% Output
%  F  : stafe feedback gain 
%  X: Lyapunov matrix 


function [F,X] = lmiHinfStateFeedbackRobust_expstab(listP,nstate,ncon,beta,solver)
% Size of the generalized plant
sizeX = size(listP{1}.a,1);
sizeU = ncon;

% To tackle with some strict inequalities problems not well implemented 
% in Yalmip
epsi = 1e-6;

%% Cut the system 
for i = 1:size(listP,2)
    A{i}   = listP{i}.a(1:sizeX,1:sizeX);
    B2{i}  = listP{i}.b(1:sizeX,1:sizeU);
end;

%% Create variables matrix
Y    = sdpvar(sizeU,sizeX,'full')
X     = sdpvar(sizeX,sizeX,'symmetric'); 
% LMIs definition of the Hinf problem
LMI=[X>=epsi];
for i = 1:size(listP,2)
    H1{i}=A{i}*X+X*A{i}'+B2{i}*Y+(B2{i}*Y)'+2*beta*X;
    LMI = [LMI,H1{i}<=-epsi];
end;
% Find feasible solution minimizing gamma
ops      = sdpsettings('solver',solver);
solution = optimize(LMI,[],ops);

%% Extract numerical solution
Y=double(Y);
X = double(X);
F= Y*inv(X);


% Some verification
VPx = eig(X)
for i=1:size(X,1)
    if (VPx(i) <= 0)
        disp('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
        disp('Error, Lyapunov function are non positive definite')
        disp('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
    end;
end;
