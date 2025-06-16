% LPV adaptive control with varying closed-loop performances (function of external parameters)
% section 7.3 (chapter 7)
% Olivier Sename
% March 2025
clc
clear all
close all


% LTI System  definition
A = [0 1;1 1];
B  = [0;1];
C  = [-1 0];
D  = 0;
G = ss(A,B,C,D);
nu=1;
ny=1;
% 7.3.2  Performance specification using weighting functions
% Parameter range for parameter dependent weights
rhom = 0.1; %rho min 
rhoM= 0.99; % rho max

% Determination of the LPV generalized plant 
% Vertex 1 rho=rhom
% Generalized plant P is found with function sysic
Ms = 2; epsi = .001;
wb = rhom;
Ae=-wb*epsi;Be=wb*(Ms-epsi);Ce=1/Ms;De=1/Ms;
W1m = ss(Ae,Be,Ce,De);
W2m=ss((11-10*rhom)/300); % this allows to consider the specification wrt to the maxima control gain defined as Mu=300*rhom;

systemnames  = 'G W1m W2m';
inputvar     = '[r;u]';
outputvar    = '[W1m; W2m; r-G]';
input_to_G   = '[u]';
input_to_W1m  = '[r-G]';
input_to_W2m  = '[u]';
sysoutname   = 'Pm';
cleanupsysic = 'yes';
sysic;

% Vertex 2 rho=rhoM
% Generalized plant P is found with function sysic
wb = rhoM;
Ae=-wb*epsi;Be=wb*(Ms-epsi);Ce=1/Ms;De=1/Ms;
W1M = ss(Ae,Be,Ce,De);
W2M=ss((11-10*rhoM)/300); % this allows to consider the specification wrt to the maxima control gain defined as Mu=300*rhoM;
systemnames  = 'G W1M W2M';
inputvar     = '[r; u]';
outputvar    = '[W1M; W2M; r-G]';
input_to_G   = '[u]';
input_to_W1M  = '[r-G]';
input_to_W2M  = '[u]';
sysoutname   = 'PM';
cleanupsysic = 'yes';
sysic;
% Pm and Pm have input matrices parameter dependent.
% To apply the polytopic approach, [B2 D12] and [C2 D21] must be parameter independent
% (see section 3.3.2.1)
% So we add an input filter, see Example 3.6:
tau_f = .001;
filter=tf(1,[tau_f 1]);
% Af=-1/tau_f;
% Bf=1/tau_f;
% Cf=1;
% Df=0;
% nf=size(Af,1);
% filter=ss(Af,Bf,Cf,Df);%
% nz=size(Pm.c,1)-ny;
% nw=size(Pm.b,2)-nu;
% nx=size(Pm.a,2);

[listPf] = InputFilter4Polytopic({Pm,PM},ny,nu,ss(filter))
%%

% Section 7.3.3 Problem solution and analysis
% LPV polytopic design
[listK,listCL,gamma] = lmiHinfPolytope(listPf,1,1,1,1,'sdpt3');
%  sensitivity functions
for i=1:2
Slpv{i}=inv(1+G*listK{i});
KSlpv{i}=listK{i}*Slpv{i};
end
% analysis in the frequency domain

w=logspace(-5,5,1000);
figure
subplot(2,1,1), sigma(Slpv{1:2},1/W1m,1/W1M,w), title('Sensitivity function S'), legend('Slpv1','Slpv2','1/W_emin','1/W_eMax')
subplot(2,1,2), sigma(KSlpv{1:2},1/W2m,1/W2M,w), title('Controller*Sensitivity KS'), legend('KSlpv1','KSlpv2','1/W_umin','1/W_uMax')



%% LTI DESIGN 
% weighting functions
Ms = 2; wb = 0.2; epsi = .001;
Ae=-wb*epsi;Be=wb*(Ms-epsi);Ce=1/Ms;De=1/Ms;
We = ss(Ae,Be,Ce,De);
Mu = 80; 
Wu=ss(1/Mu);
systemnames  = 'G We Wu';
inputvar     = '[r; u]';
outputvar    = '[We; Wu; r-G]';
input_to_G   = '[u]';
input_to_We  = '[r-G]';
input_to_Wu  = '[u]';
sysoutname   = 'P';
cleanupsysic = 'yes';
sysic;
% Hinf control design
nmeas=1; ncon=1;
[Klti,CL,gammaLti,info] = hinfsyn(P,nmeas,ncon,'DISPLAY','ON')
gammaLti

Slti=inv(1+G*Klti);
KSlti=Klti*Slti;

%%%%
w=logspace(-3,3,500);
figure
subplot(2,1,1), bodemag(Slti,'r',1/We,w), title('Sensitivity function S -  LTI case')
subplot(2,1,2), bodemag(KSlti,'r',1/Wu,w), title('Controller*Sensitivity KS - LTI case')

%% 
% Comparison of the sensitivity functions LTI vs LPV
w=logspace(-5,5,1000);
figure
subplot(2,1,1), sigma(Slpv{1:2},Slti,1/W1m,1/W1M,w), title('Sensitivity function S'), legend('S1','S2','Slti','1/W_emin,1/W_eMax')
subplot(2,1,2), sigma(KSlpv{1:2},KSlti,1/W2m,1/W2M,w), title('Controller*Sensitivity KS'), legend('KS1','KS2','Slti','1/W_umin','1/W_uMax')

%% Simulations
% for simulink
Corrm = listK{1}; CorrM = listK{2};
Akmin=Corrm.a;
Bkmin=Corrm.b;
Ckmin=Corrm.c;
Dkmin=Corrm.d;
Akmax=CorrM.a;
Bkmax=CorrM.b;
Ckmax=CorrM.c;
Dkmax=CorrM.d;
rhomin=rhom;
rhomax=rhoM;


scenario = input('Which scenario would you like to run: 1 or 2 ? Answer = ');

switch scenario
    case 1
% simulation time
time_end=250;
sim('Simulations_section73')
figure
subplot(3,1,1), plot(y.time,y.signals.values), legend('ref','yLPV','yLTI'), title('System Output (LTI & LPV cases)')
subplot(3,1,2), plot(u.time,u.signals.values), legend('uLPV','uLTI'), title('Control input (LTI & LPV case)')
subplot(3,1,3), plot(rho.time,rho.signals.values), legend('rho'); title('Scheduling parameter')
%
    case 2
% simulation time
time_end=500;
sim('Simulations_section73')

figure
subplot(3,1,1), plot(y.time,y.signals.values), legend('ref','yLPV','yLTI'), title('System Output (LTI & LPV cases)')
subplot(3,1,2), plot(u.time,u.signals.values), legend('uLPV','uLTI'), title('Control input (LTI & LPV case)')
subplot(3,1,3), plot(rho.time,rho.signals.values), legend('rho'); title('Scheduling parameter')

%%%%%%%%%%%%%%%%%%%%%%%%%%
    otherwise
        disp('Choose 1 or 2')
end
