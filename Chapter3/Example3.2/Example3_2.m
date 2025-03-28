%% example 3.2 (Chapter 3) 
% LPV state feedback control with fixed performances
% Olivier Sename 
% Jan 2025
clear all
clc
close all
%% 
% LPV system definition
%  A = [0 1; -rho^2, 2*rho];
B = [0; 1];
C = [1 0];
%%
% Scheduling parameter
rho_min=1;
rho_max=10;
ngridpoints=100;
rho= linspace(rho_min,rho_max,ngridpoints);
%%
% Compute linear system at frozen grid points 
for i = 1:length(rho)
   A = [0 1; -rho(i)^2, 2*rho(i)];
   listP{i}=ss(A,B,C,0); 
   OLpoles{i}=pole(listP{i})
end

%%
% Definition of the LPV state feedback gain 
% It ensures that all closed-loop poles= -1, -1
for i = 1:length(rho)
   listF{i}=[ rho(i)^2-1,-2-2*rho(i)];
   eig(listP{i}.a+listP{i}.b*listF{i});
end

% plot poles in the complex plane
figure
for i = 1:length(rho)
 EIGCL{i}=eig(listP{i}.a+listP{i}.b*listF{i});
 plot(real(EIGCL{i}(1)),imag(EIGCL{i}(1)),'x',real(EIGCL{i}(2)),imag(EIGCL{i}(2)),'+')
 hold on
end

%%
% Parameter Dependent Stability check of the closed-loop matrix
maxdrho=100;
solver='sedumi';
for i = 1:length(rho)
   listAcl{i}=listP{i}.a+listP{i}.b*listF{i}; 
   end
[X,X0,X1,X2] = LMI_ParameterDependentStability_grid(listAcl,rho,maxdrho,solver)

