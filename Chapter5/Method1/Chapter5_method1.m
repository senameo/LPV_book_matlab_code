%% LPV  Observer for suspension systems
% Method 1: an Hinf /gH2 observer for suspension state estimation
% section 5.5 (chapter 5)
% Olivier Sename
% March 2025
close all
clear all
clc

% Parameters 
ms=2.27; % Sprung mass 2.27 kg
mus=0.25;%  unsprung mass 0.25 kg
ks=1396; %Spring stiffness 1396 N/m
kt=12270; %Tire stiffness 12270 N/m
k0=170.4; % Passive damper stiffness coefficient 170.4 N/m
c0=68.83; % Viscous damping coefficient 68.83 N.s/m
k1=218.16;%  Hysteresis coefficient due to displacement 218.16 N.s/m
c1=21;  % Hysteresis coefficient due to velocity 21 N.s/m
fc = 28.07; %Dynamic yield force of ER fluid 28.07 N
tau=43e-3; % Time constant 43 ms
k=ks+k0;

%% qLPV model - section 5.4.1.
% x = [ Zs-Zus Zs' Zus-Zr Zus' Fer ]
%rho=u*tanh(k1zdef+c1zdefdt)/(k1zdef+c1zdefdt)
rhomin=0.01;
rhomax=0.35;
A{1} = [0 1 0 -1 0; -k/ms -c0/ms 0 c0/ms -1/ms ; 0 0 0 1 0 ; k/mus c0/mus -kt/mus -c0/mus 1/mus ;fc*k1*rhomin/tau fc*c1*rhomin/tau 0 -fc*c1*rhomin/tau -1/tau];
A{2} = [0 1 0 -1 0; -k/ms -c0/ms 0 c0/ms -1/ms ; 0 0 0 1 0 ; k/mus c0/mus -kt/mus -c0/mus 1/mus ;fc*k1*rhomax/tau fc*c1*rhomax/tau 0 -fc*c1*rhomax/tau -1/tau];
B1{1} = [0 ; 0 ; -1 ; 0 ; 0]; %% Unknown input dot_zr
B1{2}=B1{1};
C=[1 0 0 0 0 ;0 1 0 -1 0];
Cz=[k0 c0 0 -c0 1];
%% Solve the LMIs to find the observer gain
% Synthesis
alpha=50;theta=pi/8;r=100;
N=eye(2)*0.001;
gamma_inf=0.01; % fixed gamma_inf
[L,P,gamma_2,VP]=obs_LPVpol_pole_h2_hinf(A,B1,C,N,Cz,gamma_inf,alpha,theta,r,'sedumi')

for i=1:size(A,2)
  Aobs(:,:,i) = A{i}-L{i}*C;
  pole_obs(:,i)=eig(Aobs(:,:,i));
  Bobs(:,:,i) = [L{i}];
 end
 pole_obs<0
tobs=100
% Analysis of the pole placement result 
max_max = r+2;

figure,
hold on
plot([-max_max,2],[0,0],'k','LineStyle','--')
plot([0,0],[-max_max,max_max],'k','LineStyle','--')
plot([-alpha,-alpha],[-max_max,max_max],'LineWidth',1)
plot([-max_max,0],[-max_max,0]*(tan(theta)),'LineWidth',1)
plot([-max_max,0],[-max_max,0]*(tan(-theta)),'LineWidth',1)
plot(r*cos(0:0.01:2*pi),r*sin(0:0.01:2*pi),'LineWidth',1);
for i=1:length(VP)
plot(real(VP{i}),imag(VP{i}),'+','LineWidth',2)
end
xlim([-max_max,2])
ylim([-100,100])
grid on

%%  Simulations
Init_simulations_chapter5_method1 %uncomment to launch