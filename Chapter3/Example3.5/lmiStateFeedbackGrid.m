% Author: Olivier Sename
% Jan 2025
%
%  [K,gopt,P,P0,P1,P2,Y0,Y1,Y2,CL]= lmiStateLMIeedbackGrid(listGp,rho,maxdrho,nmeas,ncon,sp,solver)
% Description
% Function that computes the LPV / Hinf state feedback controller solving the LMI problem 
% for a gridded LPV system.
% To use this function one have to create the  generalized plant P st:
%  xdot = [   A  Bw  Bu  ]  [x w u]^T
%   z   =  [ Cz Dzw Dzu ]  [x w u]^T
% and to input them as a list of systems
% LMIs are with +/- MAX drho/dt
%
%  Input
%  listGp : list of plants
%  rho : gridded point
%  maxdrho : bounded derivated of rho => max(drho/dt)
%  nmeas  : number of measures
%  ncon   : number of control signals
%  solver : name of the solver used ('dsdp','sedumi'...)
% 
%  Output
%  K  : list of controller (vector gain K)
%  gopt   : optimal gamma
%  P : list of Lyapunov matrices P  for all grid points
%  Y:  list of parameter Y for all grdi points
%  [P0, P1, P2] such that P(rho) = P0 + rho P1 + rho^2 P2
%  [Y0, Y1, Y2] such that Y(rho) = Y0 + rho Y1 + rho^2 Y2


function [K,gopt,P,P0,P1,P2,Y0,Y1,Y2,CL] = lmiStateLMIeedbackGrid(listGp,rho,maxdrho,nmeas,ncon,solver)
%%% Size of the generalized plant
sizeP = size(listGp{1}.a,1);
sizeZ = size(listGp{1},1)-nmeas; % number of controlled output - number of measure
sizeY = nmeas;
sizeW = size(listGp{1},2)-ncon;  % number of input - number of control
sizeU = ncon;
sizeR = length(rho);

%%% To tackle with some strict inequalities problems not well implemented 
%%% in Yalmip
epsi = 1e-10;

%%% Shows vector lenght
input  = [sizeP sizeW sizeU];
output = [sizeP sizeZ sizeY];

%%% Cut the system 
for i = 1:sizeR
    A{i}   = listGp{i}.a(1:sizeP,1:sizeP);
    Bw{i}  = listGp{i}.b(1:sizeP,1:sizeW);
    Bu{i}  = listGp{i}.b(1:sizeP,sizeW+1:sizeW+sizeU);
    Cz{i}  = listGp{i}.c(1:sizeZ,1:sizeP);
    Dzw{i} = listGp{i}.d(1:sizeZ,1:sizeW);
    Dzu{i} = listGp{i}.d(1:sizeZ,sizeW+1:sizeW+sizeU);
end;

    ERROR = 0;
    if (length(rho) == size(listGp,2))
    else 
        disp('Error: gridded point and LPV system mismatch')
        ERROR = 1;
    end 

%%% Create variables matrix

% define matrix form Y such that Y(rho) = Y0 + rho Y1 + rho^2 Y2
% K = -YQ^-1
Y0 = sdpvar(sizeU,sizeP); 
Y1 = sdpvar(sizeU,sizeP); 
Y2 = sdpvar(sizeU,sizeP); 
for i = 1:sizeR
    Y{i} = Y0+rho(i)*Y1+rho(i)^2*Y2; 
end;

% define matrix form Y such that P(rho) = P0 + rho P1 + rho P2
P0 = sdpvar(sizeP,sizeP,'symmetric'); 
P1 = sdpvar(sizeP,sizeP,'symmetric'); 
P2 = sdpvar(sizeP,sizeP,'symmetric'); 
for i = 1:sizeR
    P{i} = P0+rho(i)*P1+rho(i)^2*P2; 
end;

% define derivative dP/drho since in LMIs there is max(drho)*dP/drho
for i = 1:sizeR
    dP{i} = P1+2*rho(i)*P2; 
end;

% define gamma constant real in L-2 induced norm
gamma = sdpvar(1,1,'full');

% conditions on gamma > 0
LMI=[gamma>=epsi];
%%% LMIs definition of the Hinf problem
% conditions on P(rho) > 0
for i = 1:sizeR
    LMI= [LMI,P{i}>=epsi];
end

for i = 1:sizeR
    %%% LMIs for L-2 induced norm
    M11{i}=A{i}*P{i}+P{i}*A{i}'+Bu{i}*Y{i}+(Bu{i}*Y{i})'+dP{i}*maxdrho; 
    M21{i}=Bw{i}';
    M31{i}=Cz{i}*P{i}+Dzu{i}*Y{i};
    M32{i}=Dzw{i};
    H1{i}  = [ M11{i}        M21{i}'                   M31{i}';
               M21{i}      -gamma*eye(sizeW)           M32{i}';
               M31{i}        M32{i}          -gamma*eye(sizeZ)]; 
    %%% conditions on LMI     
    name1 = ['Hinf constraint ' int2str(i)];
    % Add sets 
     LMI= [LMI,H1{i}<=-epsi];
    
    N11{i}=A{i}*P{i}+P{i}*A{i}'+Bu{i}*Y{i}+(Bu{i}*Y{i})'-dP{i}*maxdrho; 
    N21{i}=Bw{i}';
    N31{i}=Cz{i}*P{i}+Dzu{i}*Y{i};
    N32{i}=Dzw{i};
    H2{i}  = [ N11{i}        N21{i}'                   N31{i}';
               N21{i}      -gamma*eye(sizeW)           N32{i}';
               N31{i}        N32{i}          -gamma*eye(sizeZ)]; 
    %%%  conditions on LMI       
    name2 = ['Hinf constraint ' int2str(i)];
    % Add sets 
        LMI= [LMI,H2{i}<=-epsi];
end;

% Solution of the LMI problemminimizing gamma
ops      = sdpsettings('solver',solver);
solution = optimize(LMI,gamma,ops);

gopt = double(gamma);

%%% Extract numerical solution
for i = 1:sizeR
P{i}=double(P{i});
Y{i}=double(Y{i});
end;

P0=double(P0);
P1=double(P1);
P2=double(P2);
%
Y0=double(Y0);
Y1=double(Y1);
Y2=double(Y2);

% Check P is positive definite
for i = 1:sizeR
    if (all(eig(P{i}) > 0) ~= 1)
        disp(strcat(num2str(i),'Error, Lyapunov function are non positive definite'))
    end
end;

%%% Calculation of the controller gains for all grid points
for i = 1:sizeR
K{i}= Y{i}*inv(P{i});
end;

% closed loop u = -Kx with transfer z/d
for i = 1:sizeR
CL{i}= ss(A{i}+Bu{i}*K{i},Bw{i},Cz{i}-Dzu{i}*K{i},Dzw{i});
end


