%% example 3.4 (Chapter 3) 
% Static state feedback control: a polytopic approach
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
% %
% Polytopic model
% Scheduling parameter - LPV system
rho_min=1;
rho_max=10;
% Scheduling parameter for the polytopic system :  rho1=rho, rho2=rho^2
rho1_min=rho_min;
rho1_max=rho_max;
rho2_min=rho_min^2;
rho2_max=rho_max^2;
% Systems at the vertices of the polytope
A1 = [0 1; -rho2_min, 2*rho1_min];
A2 = [0 1; -rho2_max, 2*rho1_min];
A3 = [0 1; -rho2_min, 2*rho1_max];
A4 = [0 1; -rho2_max, 2*rho1_max];

P1=ss(A1,[0.1*B B],C,[0]);
P2=ss(A2,[0.1*B B],C,[0]);
P3=ss(A3,[0.1*B B],C,[0]);
P4=ss(A4,[0.1*B B],C,[0]);

%% 
% Polytopic SF control with external input
% Hinf control approach
[Fpol,gpol,X] = lmiHinfStateFeedbackPolytope({P1,P2,P3,P4},2,1,5,'sedumi')

eigA1= eig(A1+B*Fpol{1});
eigA2= eig(A2+B*Fpol{2})
eigA3= eig(A3+B*Fpol{3})
eigA4= eig(A4+B*Fpol{4})
figure
  plot(real(eigA1(1)),imag(eigA1(1)),'x',real(eigA1(2)),imag(eigA1(2)),'+')
  hold on
  plot(real(eigA2(1)),imag(eigA2(1)),'x',real(eigA2(2)),imag(eigA2(2)),'+')
  hold on
  plot(real(eigA3(1)),imag(eigA3(1)),'x',real(eigA3(2)),imag(eigA3(2)),'+')
  hold on
    plot(real(eigA4(1)),imag(eigA4(1)),'x',real(eigA4(2)),imag(eigA4(2)),'+')
  
  
