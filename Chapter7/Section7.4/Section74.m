% LPV adaptive control function of varying endogeneous parameters
% section 7.4 (chapter 7)
% Olivier Sename
% March 2025
clc
clear all
close all
%% System model - section 7.4.1
disp('Models of the flexible system with no load, half load, and full load')
load('model_flexiblesystem.mat')
w=logspace(-1,2,1000);
figure
bodemag(model_0,model_50,model_100,w),legend
%%
% 7.4.3  Performance specification using weighting functions (here LTI)
Ms=2; wb=2; epsi=1e-3; We = tf([1/Ms wb], [1 wb*epsi]);   % weight on tracking error
Mu=2; wbc=20; epsi1=1e-3; Wu = tf([1 wbc/Mu], [epsi1 wbc]);   % weight on control input

%%
% % LTI Nomimal design at  load O%
G=model_0;
systemnames = 'G We Wu ';
inputvar = '[ r;n;u]';
outputvar = '[We; Wu; r;G+n]';
input_to_G = '[u]';
input_to_We = '[r-G-n]';
input_to_Wu = '[u]';
sysoutname = 'P0';
cleanupsysic = 'yes';
sysic;
% Find H-infinity optimal controller
nmeas=2; nu=1; gmn=0.5; gmx=20; tol=0.001;
[khinf0,CL0,gammaLTI,info] = hinfsyn(P0,nmeas,nu,'display','on')
disp('Sensitivity analysis')
S0=CL0(1,1)/We;
T0=1-S0;
SG0=CL0(1,2)/We;
KS0=CL0(2,1)/Wu;

figure
sgtitle('Nominal design at load 0%')
subplot(1,2,1), bodemag(1/We,S0,w),title('S')
subplot(1,2,2), bodemag(1/Wu,KS0,w),title('KS')

%% Computation LPV polytopic controller
% To apply the polytopic method the genralized plant must be such that [B2 D12] and [C2 D21] must be parameter independent
% Here the weights being LTI it is enough to filter the ssyetm model at the
% input and at the output
tauf=1e-3;
filter=ss(-1/tauf,1/tauf,1,0);
% Generalized plant P is found with function sysic
G = series(filter,series(model_0,filter));
systemnames = 'G We Wu ';
inputvar = '[ r(1);n;u]';
outputvar = '[We; Wu; r;G+n]';
input_to_G = '[u]';
input_to_We = '[r-G-n]';
input_to_Wu = '[u]';
sysoutname = 'Pm';
cleanupsysic = 'yes';
sysic;

G = series(filter,series(model_100,filter));
systemnames = 'G We Wu ';
inputvar = '[ r(1);n;u]';
outputvar = '[We; Wu; r;G+n]';
input_to_G = '[u]';
input_to_We = '[r-G-n]';
input_to_Wu = '[u]';
sysoutname = 'PM';
cleanupsysic = 'yes';
sysic;

% LPV synthesis of a polytopic controller 
[listK,listCL,gamma] = lmiHinfPolytope({Pm PM},nmeas,nu,1,1,'sedumi');
%  sensitivity functions
for i=1:2
Slpv{i}=listCL{i}(1,1)/We;
KSlpv{i}=listCL{i}(2,1)/Wu;
end
% analysis in the frequency domain

figure
subplot(1,2,1), sigma(Slpv{1:2},1/We,w), title('Sensitivity function S'), legend('Slpv1','Slpv2','1/W_e')
subplot(1,2,2), sigma(KSlpv{1:2},1/Wu,w), title('Controller*Sensitivity KS'), legend('KSlpv1','KSlpv2','1/W_u')

%%
% Simulink
K=khinf0;
Kr=K(:,1);
Ky=K(:,2);%
Corrm = listK{1}; CorrM = listK{2};


Akmin=Corrm.a;
Bkmin=Corrm.b;
Ckmin=Corrm.c;
Dkmin=Corrm.d;
Akmax=CorrM.a;
Bkmax=CorrM.b;
Ckmax=CorrM.c;
Dkmax=CorrM.d;
rhomin=0.01;
rhomax=0.99;


scenario = input('Which scenario would you like to run: 1 or 2 ? Answer = ');

switch scenario
    case 1
% simulation time
time_end=90;
sim('Simulations_section74')
figure
subplot(3,1,1), plot(y.time,y.signals.values), legend('ref','yLPV','yLTI'), title('System Output (LTI & LPV cases)')
subplot(3,1,2), plot(u.time,u.signals.values), legend('uLPV','uLTI'), title('Control input (LTI & LPV case)')
subplot(3,1,3), plot(rho.time,rho.signals.values), legend('rho'); title('Scheduling parameter')

    case 2
        % simulation time
time_end=50;
sim('Simulations_section74')
figure
subplot(3,1,1), plot(y.time,y.signals.values), legend('ref','yLPV','yLTI'), title('System Output (LTI & LPV cases)')
subplot(3,1,2), plot(u.time,u.signals.values), legend('uLPV','uLTI'), title('Control input (LTI & LPV case)')
subplot(3,1,3), plot(rho.time,rho.signals.values), legend('rho'); title('Scheduling parameter')

%%%%%%%%%%%%%%%%%%%%%%%%%%
    otherwise
        disp('Choose 1 or 2')
end












