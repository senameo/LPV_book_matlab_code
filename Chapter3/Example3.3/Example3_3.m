%% example 3.3 (Chapter 3) 
% LPV state feedback control with varying (adaptive) performances
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
   P(:,:,i) = ss(A,B,C,0); 
   listA{i}=A; 
   listP{i}=ss(A,B,C,0); 
end

% Definition of the LPV state feedback gain 
% It ensures that all poles= -rho, -rho 
for i = 1:length(rho)
listF{i}=[0,-4*rho(i)];
 EIGCL{i}=eig(listP{i}.a+listP{i}.b*listF{i});
 plot(real(EIGCL{i}(1)),imag(EIGCL{i}(1)),'x',real(EIGCL{i}(2)),imag(EIGCL{i}(2)),'+')
 hold on
end


maxdrho=10;
solver='sdpt3';
for i = 1:length(rho)
   listAcl{i}=listP{i}.a+listP{i}.b*listF{i}; 
   end
[X,X0,X1,X2] = LMI_ParameterDependentStability_grid(listAcl,rho,maxdrho,solver);
