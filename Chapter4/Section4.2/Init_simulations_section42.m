% Example section 4.2 
% Time-domain simulation
% section 4.2.6
% Olivier Sename March 2025

% intialization
for i=1:2^N
  Ak_lpv(:,:,i) = listK{i}.a;
  Bk_lpv(:,:,i) = listK{i}.b;
  Ck_lpv(:,:,i) = listK{i}.c;
  Dk_lpv(:,:,i) = listK{i}.d;
end
E=1*B;

%%
scenario = input('Which scenario would you like to run: 1 or 2 ? Answer = ');

switch scenario
    case 1
x0=[0;0];
xk0=[0;0;0;0];
ueq=0;
LTIcontoller=KLTI1;
% diturbance time action
time_dist=15;
sim('Simulations_section42')

rho.signals.values=squeeze(rho.signals.values);
figure
plot(rho.time, rho.signals.values(1:2,:)),legend('rho1','rho2')
figure
plot(output.time,output.signals.values), legend('ref','yLPV','yLTI')

% 
figure
plot(u.time,u.signals.values),legend('uLPV','uLTI')


figure
plot(poly_coord.time,poly_coord.signals.values),title('polytopic coordoinates')

    case 2
x0=[pi/2;-1];
xk0=[0;0;0;0];
LTIcontoller=KLTI2;
ueq=-x0(1)*x0(2);
% diturbance time action
time_dist=20;
sim('Simulations_section42')


rho.signals.values=squeeze(rho.signals.values);
figure
plot(rho.time, rho.signals.values(1:2,:)),legend('rho1','rho2')

figure
plot(output.time,output.signals.values), legend('ref','yLPV','yLTI')

% 
figure
plot(u.time,u.signals.values),legend('uLPV','uLTI')


figure
plot(poly_coord.time,poly_coord.signals.values),title('polytopic coordoinates')

%%%%%%%%%%%%%%%%%%%%%%%%%%
    otherwise
        disp('Choose 1 or 2')
end
