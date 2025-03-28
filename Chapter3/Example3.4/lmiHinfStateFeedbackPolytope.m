% Author: Olivier Sename
% Jan 2025
%
% [K,gopt,P] = lmiHinfStateFeedbackPolytope_2023(listP,nstate,ncon,percentage,solver)
% 
% Description
% Function that computes the state feedback Hinf controller solving the LMI problem. 
% To use this function one have to create the 
% generalized plant P such that:
%  xdot = [   A  Bw  Bu  ]  [x w u]^T
%   z   =  [ Cz Dzw Dzu ]  [x w u]^T
% 
% Input
%  listP  : list of plants
%  ncon   : number of control signals
%  nstate : number of state variables
%  sp     : the controller is striclty proper (1=>yes, 0=>no)
%  percentage : percentage added to the gamma optimal to conditionate the
%  controller

% Output
%  K  : list of state feedback gain  (at the vertices of the polytope)
%  P
%  gopt   : optimal gamma
%

function [K,gopt,P] = lmiHinfStateFeedbackPolytope(listP,nstate,ncon,percentage,solver)
%%% Size of the generalized plant
sizeP = size(listP{1}.a,1);
sizeZ = size(listP{1},1); % number of controlled output 
sizeW = size(listP{1},2)-ncon;  % number of input - number of control
sizeU = ncon;

%%% To tackle with some strict inequalities problems not well implemented 
%%% in Yalmip
epsi = 1e-6;

%%% Shows vector lenght
input  = [sizeP sizeW sizeU];
output = [sizeP sizeZ];

%%% Cut the system 
for i = 1:size(listP,2)
    A{i}   = listP{i}.a(1:sizeP,1:sizeP);
    Bw{i}  = listP{i}.b(1:sizeP,1:sizeW);
    Bu{i}  = listP{i}.b(1:sizeP,sizeW+1:sizeW+sizeU);
    Cz{i}  = listP{i}.c(1:sizeZ,1:sizeP);
    Dzw{i} = listP{i}.d(1:sizeZ,1:sizeW);
    Dzu{i} = listP{i}.d(1:sizeZ,sizeW+1:sizeW+sizeU);

end;

% %%% Create variables matriP
for i = 1:size(listP,2)
    Y{i}     = sdpvar(sizeU,sizeP); 
end;
P     = sdpvar(sizeP,sizeP,'symmetric'); 
gamma = sdpvar(1,1,'full');%1;%

%%% LMIs definition of the Hinf problem
F=[P>=epsi];
%F  = set(H0>eps,'P and Y constraint');
for i = 1:size(listP,2)
    M11{i}=A{i}*P+P*A{i}'+Bu{i}*Y{i}+(Bu{i}*Y{i})'; 
    M21{i}=Bw{i}';
    M31{i}=Cz{i}*P+Dzu{i}*Y{i};
    M32{i}=Dzw{i};
    H1{i}  = [ M11{i}        M21{i}'               M31{i}';
               M21{i}      -eye(sizeW)           M32{i}';
               M31{i}        M32{i}          -gamma*eye(sizeZ)]; 
    name = ['Hinf constraint ' int2str(i)];
    F = [F,H1{i}<=-epsi];
end;
%F

%%% Find feasible solution minimizing gamma
%solver='sedumi';
ops      = sdpsettings('solver',solver);
solution = optimize(F,gamma,ops);

gopt = sqrt(double(gamma))*(1+percentage/100);

% % 
% % %%% EPtract numerical solution
for i = 1:size(listP,2)
Y{i}=double(Y{i});
end;
P = double(P);

for i = 1:size(listP,2)
K{i}= Y{i}*inv(P);
end;

%%% Some verifications
VPP = eig(P)
for i=1:size(P,1)
    if (VPP(i) <= 0)
        disp('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
        disp('Error, Lyapunov function are non positive definite')
        disp('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
    end;
end;
