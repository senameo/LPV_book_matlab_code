%% example 3.5 (Chapter 3) 
% Static state feedback control: a grid-based approach
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
   listPgrid{i}=ss(A,[0.1*B B],C,[0 0]);
end

%% 
% SF control with external input: grid-based method
% Hinf control approach
maxdrho=10;
ny=1;
nu=1;
solver='sdpt3';
[Fgrid,gopt,P,P0,P1,P2,Y0,Y1,Y2,CL]  = lmiStateFeedbackGrid(listPgrid,rho,maxdrho,ny,nu,solver)

figure
for i = 1:length(rho)
 EIGCL{i}=eig(CL{i}.a);
 plot(real(EIGCL{i}(1)),imag(EIGCL{i}(1)),'x',real(EIGCL{i}(2)),imag(EIGCL{i}(2)),'+')
 hold on
end

