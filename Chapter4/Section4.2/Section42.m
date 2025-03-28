%% Hinf/LPV control of a non linear system
% Chapter 4; section 4.2
% O. Sename Feb 2025
% LPV System description
% x1dot=rho1.x1+x2, rho1=sin(x1)/x1  add , if not x1 goes to 0
% x2dot=rho2.x2+u, rho2=x1
% y=x1
clear all
clc
close all

%% LTI models
% equilibrium =(pi/2, -1)
Anom1=[1 1;0 0];
B=[0;1];
C= [1 0];
Glti1=ss(Anom1,B,C,0)
pole(Glti1)
% equilibrium =(pi/2, -1)
Anom2=[0 1;-1 pi/2];
B=[0;1];
C= [1 0];
Glti2=ss(Anom2,B,C,0)
pole(Glti2)

%% Section 4.2.1
% Polytopic model
% parameters min/max values
rho1max=1;
rho1min=-0.3;
rho2max=5;
rho2min=-5;
% Application of Algorithm 1 in section 3.3.2.4 : Vertices computation of a matrix M
vertices = [rho1min rho1max;rho2min rho2max];
% Binary elements
N = size(vertices,1);
bi_j_str = [];
bi_j = [];
for i=0:2^N-1
    bi_j_str{i+1} = dec2bin(i,N);
end
for i=1:2^N
    for j=1:N
        bi_j(i,j) = str2num(bi_j_str{i}(j));
    end
end

% Matrix vertices
rho_tilde = [];
A = [];
for i=1:2^N
    for j=1:N
        if bi_j(i,N+1-j) == 0
            rho_tilde(j) = vertices(j,1);
        else
            rho_tilde(j) = vertices(j,2);
        end
    end
    A{i} = [rho_tilde(1) 1;0 rho_tilde(2)];
     G{i}=ss(A{i},B,C,0);
end

%% Section 4.2.2  LTI and LPV Hinf control
% 4.2.3  Performance specification using weighting functions
Ms = 2; wb =1; epsi = .001;
We = tf([1/Ms wb],[1 wb*epsi]);
Mu = 100; wbc = 1000; epsi1 = .01;
Wu = tf([1 wbc/Mu],[epsi1 wbc]);
% Determination of the LPV generalized plant 
for i=1:2^N
Sys=G{i};
systemnames  = 'Sys We Wu';
inputvar     = '[r; d;u]';
outputvar    = '[We; Wu; r-Sys]';
input_to_Sys   = '[u+d]';
input_to_We  = '[r-Sys]';
input_to_Wu  = '[u]';
sysoutname   = 'Pgen';
cleanupsysic = 'yes';
sysic;
P{i}=Pgen;
end
% Section 4.2.4  Problem solution and analysis
nmeas=1;ncon=1;sp=1;percentage=5;
[listK,listCL,gamma] = lmiHinfPolytope(P,nmeas,ncon,sp,percentage,'sdpt3');
%  sensitivity functions
for i=1:2^N
S{i}=inv(1+G{i}*listK{i});
T{i}=1-S{i};
SG{i}=S{i}*G{i};
KS{i}=listK{i}*S{i};
end
% analysis in the frequency domain

w=logspace(-2,3,500);
figure
subplot(2,2,1), sigma(S{1:4},1/We,w), title('Sensitivity function S'), legend('S1','S2','S3','S4','1/W_e')
subplot(2,2,2), sigma(T{1:4},w),  title('Complementary sensitivity function T'), legend('T1','T2','T3','T4')
subplot(2,2,3), sigma(SG{1:4},w), title('Sensitivity*Plant SG'), legend('SG1','SG2','SG3','SG4')
subplot(2,2,4), sigma(KS{1:4},1/Wu,w), title('Controller*Sensitivity KS'), legend('KS1','KS2','KS3','KS4','1/W_u')


%% LTI design at 2 equilibirum points
G1=Glti1;
systemnames  = 'G1 We Wu';
inputvar     = '[r; d;u]';
outputvar    = '[We; Wu; r-G1]';
input_to_G1   = '[u+d]';
input_to_We  = '[r-G1]';
input_to_Wu  = '[u]';
sysoutname   = 'P1';
cleanupsysic = 'yes';
sysic;
%%% Find H-infinity optimal controller
nmeas=1; ncon=1;
[KLTI1,CL1,gammaLti1,info1] = hinfsyn(P1,nmeas,ncon,'DISPLAY','ON')
% sensitivity functions
Slti1=inv(1+G1*KLTI1);
Tlti1=1-Slti1;
SGlti1=Slti1*G1;
KSlti1=KLTI1*Slti1;
% Case 2
G2=Glti2;
systemnames  = 'G2 We Wu';
inputvar     = '[r; d;u]';
outputvar    = '[We; Wu; r-G2]';
input_to_G2   = '[u+d]';
input_to_We  = '[r-G2]';
input_to_Wu  = '[u]';
sysoutname   = 'P2';
cleanupsysic = 'yes';
sysic;
%%% Find H-infinity optimal controller
nmeas=1; ncon=1;
[KLTI2,CL2,gammaLti2,info2] = hinfsyn(P2,nmeas,ncon,'DISPLAY','ON')
% sensitivity functions
Slti2=inv(1+G2*KLTI2);
Tlti2=1-Slti2;
SGlti2=Slti2*G2;
KSlti2=KLTI2*Slti2;


figure
subplot(2,2,1), bodemag(Slti1,'r',Slti2,'b',1/We,w), legend('Slti1','Slti2','1/W_e'), title('Sensitivity function S')
subplot(2,2,2), bodemag(Tlti1,'r',Tlti2,'b',w), legend('Tlti1','Tlti2'),  title('Complementary sensitivity function T')
subplot(2,2,3), bodemag(SGlti1,'r',SGlti2,'b',w), legend('SGlti1','SGlti2'), title('Sensitivity*Plant SG')
subplot(2,2,4), bodemag(KSlti1,'r',KSlti2,'b',1/Wu,w), legend('KSlti1','KSlti2','1/W_u'), title('Controller*Sensitivity KS')

%% Cross validation 
%  each LTI controller is applied to the 4 vertices
for i=1:2^N
S1{i}=inv(1+G{i}*KLTI1);
end
for i=1:2^N
S2{i}=inv(1+G{i}*KLTI2);
end

figure,
subplot(2,2,1),bodemag(S1{1:4},1/We,w)
subplot(2,2,2),pzmap(S1{1:4})
subplot(2,2,3),bodemag(S2{1:4},1/We,w)
subplot(2,2,4),pzmap(S2{1:4})


