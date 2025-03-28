%% LPV  Observer for suspension systems
% Method 2: a Hinf filtering approach for damper force estimation
% section 5.6 (chapter 5)
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

rhomin=-1;
rhomax=1;
%% qLPV model - section 5.4.2.
% u= [zr F]
% x= [zs zsdt zus zusdt Fer]
% y= x
% ρ = tanh(k1(zs − zus) + c1(\dot z_s − \dot z_us))

A= [0 1 0 0 0; 
    -k/ms -c0/ms k/ms c0/ms -1/ms; 
    0 0 0 1 0; 
    k/mus c0/mus -(k+kt)/mus -c0/mus 1/mus;
    0 0 0 0 -1/tau];
C=eye(5);
B2{1}  = [0;0;0;0;fc/tau ]*rhomin;
B2{2} = [0;0;0;0;fc/tau ]*rhomax;
B1 = [0;0;0;kt/mus;0];

for i=1:size(B2,2)
G{i}=ss(A,[B1,B2{i}],C,0);
end

%% defintion of the Hinf problem statement using weighting functions see Fig 5.9
% Performance weight on z=Fd
Cz= [k0 c0 -k0 -c0 1]; % output performmance matrix
epsi = 0.01 ;
Ms = 1 ;
wb= 10; 
We=(tf([1/Ms wb], [1 wb*epsi]));

% road input
Wzr=ss(1e0);

% Filter for the compliance with the polytopic approach
wf=1e3;
Af=-2*pi*wf;
Bf=2*pi*wf;
Cf=1;
Df=0;
F=ss(Af,Bf,Cf,Df);
for i=1:size(B2,2)
QoV=G{i};
[a,b,c,d]=linmod('Pgenerationsimulink_chapter5_method2');
disp('P from Simulink')
listP{i}=ss(a,b,c,d);
end
%%
percentage = 5; % gamma=gamma_min+gamma_ga where gamma_ga=percentage*gamma_min
nmeas      = 3;     % number of measured output
ncon       = 1;     % number of input to estimator   
sp         = 0;     % striclty proper controller? (1=>yes, 0=>no)


[listK,listCL,gammaLPV] = lmiHinfPolytope(listP,nmeas,ncon,sp,percentage,'mosek');


%% Problem solution analysis
% Frequency domain plot of the estumation error Bode diagram, compared with
% the performance template
w=logspace(-2,4,1000);
figure, bodemag(1/We,'b',listCL{1}(1,1)/We,listCL{2}(1,1)/We,w)

%%  Simulations
% extraction of state space matrices of the LPV polytopic filter
Estmin = listK{1}; EstMax = listK{2};
Akmin=Estmin.a;
Bkmin=Estmin.b;
Ckmin=Estmin.c;
Dkmin=Estmin.d;
Akmax=EstMax.a;
Bkmax=EstMax.b;
Ckmax=EstMax.c;
Dkmax=EstMax.d;
Init_simulations_chapter5_method2 %uncomment to launch