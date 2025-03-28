%% LPV  Observer for suspension systems
% Method 3:  a nonlinear parameter varying approach for state estimation
% section 5.7 (chapter 5)
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

%% qLPV model - section 5.4.3.
% x = [ zs-zus zs' zus-Zr zus' Fer ]
% u= [zr]
% y= x
% ρ = u
rhomin=0;
rhomax=1;
A = [0 1 0 -1 0; -k/ms -c0/ms 0 c0/ms -1/ms ; 0 0 0 1 0 ; k/mus c0/mus -kt/mus -c0/mus 1/mus ; 0 0 0 0 -1/tau];
nx=size(A,1);
B2{1} = [0 ; 0 ; 0 ; 0 ; rhomin/tau];
B2{2} = [0 ; 0 ; 0 ; 0 ; rhomax/tau];
B1 = [0  0; 0 0; -1 0 ; 0 0 ; 0 0]; %% w=[wr wn]
C = [-k/ms -c0/ms 0 c0/ms -1/ms ; k/mus c0/mus -kt/mus -c0/mus 1/mus]; %accelerations
ny=size(C,1);
D = [0 0.01 ; 0 0.001]; %% Measurement noise
Lambda=[k1,c1,0 -c1 0];
Cz = [1 0 0 0 0; 0 1 0 0 0; 0 0 0 1 0 ; 0 0 0 0 1]; %
%%%
%% section 5.7.3. NLPV observer polytopic design
[Lp,P,Y,gamma_p,epsilon_lp]=LMI_NLPV_Polytopic(A,B1,B2,Lambda,C,D,Cz,'sdpt3')
% extract observer gain at the vertices of the polytope
L1_p=Lp{1};
L2_p=Lp{2};
%% section 5.7.4. NLPV observer grid-based design
ngrid=10;
interv=(rhomax-rhomin)/ngrid;
for i=1:(ngrid+1)
rho{i}=rhomin+(i-1)*interv;
end
% List of input matrices B(rho)
for i=1:size(rho,2)
    B{i}= [0 ; 0 ; 0 ; 0 ; rho{i}/tau];
end
%
% Solve the LMI problem to find the observer gain 
maxdrho=200; % max possible change of input value in a sampling intervam
[Lg,P,Y,gamma_g,epsilon_lg]=LMI_NLPV_Griding(A,B1,B,Lambda,C,D,Cz,rho,maxdrho,'sdpt3')
% Compute the observer gain parameters L0,L1, s.t L(rho)=L0+rho.L1
P0=P{1};
P1=(P{ngrid}-P0)/rho{ngrid}
Y0=Y{1};
Y2=(rho{3}*(Y{2}-Y0)-rho{2}*(Y{3}-Y0))/(rho{3}*rho{2}^2-rho{2}*rho{3}^2);
Y1= (Y{2}-Y0)/rho{2}-(rho{2}*Y2)
M1=[P0,zeros(nx);P1,P0;zeros(nx),P1];
M2=[Y0;Y1;Y2];
L=-M1\M2;
L0_g=L(1:nx,1:ny);
L1_g=L(nx+1:2*nx,1:ny);
%%  Simulations
%  Observer matrices definition
[nx,nw] = size(B1);
[ny,nx] = size(C);
[nz,nx] = size(Cz);
nphi=size(B{1},2);
Cobs_p = Cz;
Dobs_p = zeros(nz,nphi+ny);
Cobs_g = Cz;
Dobs_g = zeros(nz,nphi+ny);

Init_simulations_chapter5_method3 %uncomment to launch

