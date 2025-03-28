% Three tank system
% section 4.3 (chapter 4)
% Design of a polytopic observer with Hinfinity and pole placement
% constraint
% Olivier Sename
% March 2025
clc
clear all
close all

% geometrical parameters of the tanks 
a=25*10^(-2);
b=34.5*10^(-2);
c=10*10^(-2);
w=3.5*10^(-2);
R=36.4*10^(-2);
L=35*10^(-2);
H1max=35*10^(-2);
H2max=35*10^(-2);
H3max=35*10^(-2);

% Model parameters - Table 1
S1=a*w;
C1=2.3314e-04; 
C2=2.7312e-04;
C3=2.4837e-04;
Alfa1=0.3068; 
Alfa2=0.4120; 
Alfa3=0.3412;


% Input constraint
%Qmax=1.4e-4; %m^3/s 

Alfa=[Alfa1 Alfa2 Alfa3];
CC=[C1 C2 C3];
 
% Equilibium point 
H30=0.15 
Q0=C3*H30^(Alfa3) 
H10=(Q0/C1)^(1/Alfa1);
H20=(Q0/C2)^(1/Alfa2);
%
%%
% Application of algorithm 1 to compute the matrices at the vertices of the poytope
% (section 3.3.2.4). Note that algorithm 2 is part of the simulink file
% Definiton of the vertices of the plytope
% row i= rho_i 
% 1st  column = minimal value
% 2nd column = maximal value
vertices = [0.05,0.7;0.03,1.5;0.03,1.1;0.03,1.4;0.03,1.8];

% binary parameters
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

% Matrix values at the vertices
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
    A{i} = [-rho_tilde(1) 0 0;rho_tilde(2) -rho_tilde(3) 0;0 rho_tilde(4) -rho_tilde(5)];
    B=[1/S1;0; 0];
    E{i}=0.1*B;
    C=[0 0 1]; 
    Cz=eye(3);
    D=0;
end

%% Synthesis of the LPV observer
% Conde definition for the LMI region pole placement constaint
alpha=2;theta=pi/20;r=100;
% synthesis
[L,P,gamma_obs,VP]=obs_LPV_pol_poleregion_hinf(A,E,C,Cz,alpha,theta,r,'sedumi')

for i=1:size(A,2)
  Aobs(:,:,i) = A{i}-L{i}*C;
  vp(:,i)=eig(Aobs(:,:,i));
end
disp('stability check: observer eignevalues at the vertices')
vp<0

%% Simulink simulation
% paramters of the simulink file
for i=1:size(A,2)
  Aobs(:,:,i) = A{i}-L{i}*C;
  Bobs(:,:,i) = [B,L{i}];
end
x0=[H10;H20;H30];
xest0=0.5*x0;
sim('Simulations_section43')

figure
plot(xxest.time,xxest.signals.values), title('Tanks levels ane estimated levels')
figure
plot(errorest.time,errorest.signals.values), title('estimation errors')
Estimation_error=errorest.signals.values;
disp('RMSE')
RMSE=rms(Estimation_error)

