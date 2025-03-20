% Author: O.Sename
% Jan 2025
% 
% Description
% [P,P0,P1,P2] = LMI_ParameterDependentStability_grid(listA,rho,maxdrho,solver)
% Function that checks the parameter dependent stability of an autonomous LPV system xdot=A(rho)x using a Parameter Depdendent Lyapunov Function 
%  Input
%  listA  : list of plant state matrices
%  rho : gridded point
%  maxdrho : bounded derivated of rho => max(drho/dt)
%  solver : name of the solver used ('sdpt3','sedumi'...)
% 
%  Output
%  listP : list of Lyap P
%  [P0, P1, P2] such that P(rho) = P0 + rho P1 + rho^2 P2
%

function [P,P0,P1,P2] = LMI_ParameterDependentStability_grid(listA,rho,maxdrho,solver)
% Size of the generalized plant
sizeP = size(listA{1},1);
sizeR = length(rho);

% To tackle with some strict inequalities problems not well implemented 
% in Yalmip
epsi = 1e-15;

% Cut the system 
for i = 1:sizeR
    A{i}   = listA{i}(1:sizeP,1:sizeP);
end;

    ERROR = 0;
    if (length(rho) == size(listA,2))
    else 
        disp('Error: gridded point and LPV system mismatch')
        ERROR = 1;
    end 

% Definition of decision variables 
% define matrix form P such that P(rho) = P0 + rho P1 + rho P2
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

%%% LMIs definition of the Hinf problem
% conditions on P(rho) > 0
for i = 1:sizeR
    LMI= [P{i}>=epsi];
end

for i = 1:sizeR
    H1{i}=A{i}*P{i}+P{i}*A{i}'+dP{i}*maxdrho; 
    LMI= [LMI,H1{i}<=-epsi];
    H2{i}=A{i}*P{i}+P{i}*A{i}'-dP{i}*maxdrho; 
    LMI= [LMI,H2{i}<=-epsi];
end

% Solution of the LMI problem
ops      = sdpsettings('solver',solver,'verbose',2);
solution = optimize(LMI,[],ops);

% Extract numerical solution
for i = 1:sizeR
P{i}=double(P{i});
end;
P0=double(P0);
P1=double(P1);
P2=double(P2);


% Check that P is positive definite
for i = 1:sizeR
    if (all(eig(P{i}) > 0) ~= 1)
        disp(strcat(num2str(i),'Error, Lyapunov function are non positive definite'))
    end
end;
