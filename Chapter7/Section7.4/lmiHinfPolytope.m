% Author: Charles Poussot-Vassal
% 
% Description
% Function that compute the LPV / Hinf controller solving the LMI problem 
% (see C. Scherer and S. Wieland) for a polytope.
% To use this function one have to create the  generalized plant P st:
%  Xdot    A  B1  B2    X
%   Z   =  C1 D11 D12   W
%   Y      C2 D21 D22   U
% and to input them as a list of systems
% 
% Input
%  listP  : list of plants
%  nmeas  : number of measures
%  ncon   : number of control signals
%  sp     : the controller is striclty proper (1=>yes, 0=>no)
%  percentage : percentage added to the gamma optimal to conditionate the
%  controller
%  solver : name of the solver used ('dsdp','sedumi'...)
% 
% Output
%  listK  : list of controller (state space)
%  listCL : list of closed loop (state space)
%  gopt   : optimal gamma
%
% [listK,listCL,gopt] = lmiHinfPolytope(listP,nmeas,ncon,sp,percentage,solver)

function [listK,listCL,gopt] = lmiHinfPolytope(listP,nmeas,ncon,sp,percentage,solver)
%%% Size of the generalized plant
sizeX = size(listP{1}.a,1);
sizeZ = size(listP{1},1)-nmeas; % number of controlled output - number of measure
sizeY = nmeas;
sizeW = size(listP{1},2)-ncon;  % number of input - number of control
sizeU = ncon;

%%% To tackle with some strict inequalities problems not well implemented 
%%% in Yalmip
eps = 1e-6;

%%% Shows vector lenght
input  = [sizeX sizeW sizeU];
output = [sizeX sizeZ sizeY];

%%% Cut the system 
for i = 1:size(listP,2)
    A{i}   = listP{i}.a(1:sizeX,1:sizeX);
    B1{i}  = listP{i}.b(1:sizeX,1:sizeW);
    B2{i}  = listP{i}.b(1:sizeX,sizeW+1:sizeW+sizeU);
    C1{i}  = listP{i}.c(1:sizeZ,1:sizeX);
    D11{i} = listP{i}.d(1:sizeZ,1:sizeW);
    D12{i} = listP{i}.d(1:sizeZ,sizeW+1:sizeW+sizeU);
    C2{i}  = listP{i}.c(sizeZ+1:sizeZ+sizeY,1:sizeX);
    D21{i} = listP{i}.d(sizeZ+1:sizeZ+sizeY,1:sizeW);
    D22{i} = listP{i}.d(sizeZ+1:sizeZ+sizeY,sizeW+1:sizeW+sizeU);
end;

ERROR = 0;
for i = 1:size(listP,2)
    if (B2{1}==B2{i})
    else
        disp('Error: System must be parameter independent on the input')
        ERROR = 1;
    end
    if (D12{1}==D12{i})
    else
        disp('Error: System must be parameter independent on the input')
        ERROR = 1;
    end
    
    
    if (C2{1}==C2{i})
    else
        disp('Error: System must be parameter independent on the output')
        ERROR = 1;
    end
    if (D21{1}==D21{i})
    else
        disp('Error: System must be parameter independent on the output')
        ERROR = 1;
    end
    
    
    if (D22{i}==zeros(sizeY,sizeU))
    else
        disp('Error: D22 should be null')
        ERROR = 1;
    end
end;

%%% Create variables matrix
for i = 1:size(listP,2)
    At{i} = sdpvar(sizeX,sizeX,'full');
    Bt{i} = sdpvar(sizeX,sizeY,'full');
    Ct{i} = sdpvar(sizeU,sizeX,'full');
    if sp == 1
        Dt{i} = zeros(sizeU,sizeY);
    end
    if sp == 0
        Dt{i} = sdpvar(sizeU,sizeY,'full');
    end
end;
X     = sdpvar(sizeX,sizeX,'symmetric'); 
Y     = sdpvar(sizeX,sizeX,'symmetric'); 
gamma = sdpvar(1,1,'full');%1;%

%%% LMIs definition of the Hinf problem
H0 = [X eye(sizeX); eye(sizeX) Y];
F  = [H0>=eps];
for i = 1:size(listP,2)
    M11{i} = A{i}*X+X*A{i}'+B2{i}*Ct{i}+(B2{i}*Ct{i})';
    M21{i} = At{i}+(A{i}+B2{i}*Dt{i}*C2{i})';
    M22{i} = Y*A{i}+A{i}'*Y+Bt{i}*C2{i}+(Bt{i}*C2{i})';
    M31{i} = (B1{i}+B2{i}*Dt{i}*D21{i})';
    M32{i} = (Y*B1{i}+Bt{i}*D21{i})';
    M41{i} = C1{i}*X+D12{i}*Ct{i};
    M42{i} = C1{i}+D12{i}*Dt{i}*C2{i};
    M43{i} = D11{i}+D12{i}*Dt{i}*D21{i};
    H{i}   = [M11{i} M21{i}'       M31{i}'              M41{i}';
              M21{i} M22{i}        M32{i}'              M42{i}';
              M31{i} M32{i}  -gamma*eye(sizeW)          M43{i}';
              M41{i} M42{i}        M43{i}          -gamma*eye(sizeZ)];
    
    %%% Add sets
    name = ['Hinf constraint ' int2str(i)];
    F = [F,H{i}<=-eps];
end;
%F

%%% Find feasible solution minimizing gamma
%solver='sedumi';
ops      = sdpsettings('solver',solver);
solution = solvesdp(F,gamma,ops);

gopt = double(gamma)*(1+percentage/100);

%%% Numerical aspects
alpha = sdpvar(1,1,'full');

H0 = [X -alpha*eye(sizeX); -alpha*eye(sizeX) Y];
F  = [H0>=eps];
for i = 1:size(listP,2)
    H{i} = [M11{i} M21{i}'       M31{i}'              M41{i}';
            M21{i} M22{i}        M32{i}'              M42{i}';
            M31{i} M32{i}   -gopt*eye(sizeW)          M43{i}';
            M41{i} M42{i}        M43{i}          -gopt*eye(sizeZ)];
    
    %%% Add sets
    name = ['Hinf constraint ' int2str(i)];
    F =[F,H{i}<=-eps];
end;
F

%%% Find feasible solution minimizing alpha
solvesdp(F,alpha,ops);

%%% Extract numerical solution
for i = 1:size(listP,2)
    At{i} = double(At{i});
    Bt{i} = double(Bt{i});
    Ct{i} = double(Ct{i});
    Dt{i} = double(Dt{i});
end;
X = double(X);
Y = double(Y);

%%% Back to the solution
% M     = eye(sizeX);
% N     = eye(sizeX) - Y*X;
[U,S,V] = svd(eye(sizeX) - X*Y);
R       = chol(S); % R'*R=S
M       = U*R';
N       = (R*V')';

for i = 1:size(listP,2)
    %%% Compute solution
    Dc{i} = Dt{i};
    Cc{i} = (Ct{i} - Dc{i}*C2{i}*X)*inv(M');
    Bc{i} = inv(N)*(Bt{i} - Y*B2{i}*Dc{i});
    Ac{i} = inv(N)*(At{i} - Y*A{i}*X - Y*B2{i}*Dc{i}*C2{i}*X - N*Bc{i}*C2{i}*X - Y*B2{i}*Cc{i}*M')*inv(M');
    %%% Give the controller and the closed loop transert
    P{i}  = ss(listP{i}.a,listP{i}.b,listP{i}.c,listP{i}.d);
    K{i}  = ss(Ac{i},Bc{i},Cc{i},Dc{i});
    CL{i} = lft(P{i},K{i});
end;

%%% List of solutions
listK  = K;
listCL = CL;

%%% Check solution
checkset(F)

%%% Some verifications
VPx = eig(X)
VPy = eig(Y)
for i=1:size(X,1)
    if (VPx(i) <= 0) | (VPy(i) <= 0)
        disp('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
        disp('Error, Lyapunov function are non positive definite')
        disp('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
    end;
end;
if ERROR == 1 
    disp('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
    disp('Error: System definition doesn''t respects the polytopic solutions')
    disp('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
end