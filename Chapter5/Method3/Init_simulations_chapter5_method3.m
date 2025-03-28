% Example Method 3 chapter 5
% Time-domain simulation
% Olivier Sename March 2025

% intialization
% Suspension system initial conditions
zs0=0.001;
zus0=0.001;
zsdt0=0.001;
zusdt0=0.001;
Fer0=1;
umax=0.35;
umin=0.1;
xveh0=[zs0;zsdt0;zus0;zusdt0];
% observer initial conditions
xobs_0 = [0.015 -0.15 0.0015 -0.15 3];

% data for simulation tests
Ts=0.005;
load('time_simulation.mat')
load('road_simulation.mat')
%%
scenario = input('Which scenario would you like to run: 1, 2 or 3? Answer = ');

switch scenario
    case 1
sim('Simulations_chapter5_method3')
rho=u;
Estimation_error_p=Fd_est_p.signals.values-Fd_real.signals.values;
figure,
subplot(2,2,1), plot(zr.time,zr.signals.values),title('road profile zr')
subplot(2,2,2), plot(rho.time,rho.signals.values), title('scheduling parameter') 
subplot(2,2,3), plot(Fd_est_p.time,Fd_real.signals.values,Fd_est_p.time,Fd_est_p.signals.values,'r'), title('Force and Estimated force')
subplot(2,2,4), plot(Fd_est_p.time,Estimation_error_p), title('estimation error')
sgtitle('Scenario 1 Polytopic approach')

disp('rms = RMSD Polytopic approach')
RMSE_p= rms(Estimation_error_p)
disp('NRMSE Polytopic approach')
NRMSE_p=rms(Estimation_error_p)/(max(Fd_real.signals.values)-min(Fd_real.signals.values))


Estimation_error_g=Fd_est_g.signals.values-Fd_real.signals.values;
figure,
subplot(2,2,1), plot(zr.time,zr.signals.values),title('road profile zr')
subplot(2,2,2), plot(rho.time,rho.signals.values), title('scheduling parameter') 
subplot(2,2,3), plot(Fd_est_g.time,Fd_real.signals.values,Fd_est_g.time,Fd_est_g.signals.values,'r'), title('Force and Estimated force')
subplot(2,2,4), plot(Fd_est_g.time,Estimation_error_g), title('estimation error')
sgtitle('Scenario 1 Grid-based approach')

disp('rms = RMSD Grid-based approach')
RMSE_g= rms(Estimation_error_g)
disp('NRMSE Grid-based approach')
NRMSEg=rms(Estimation_error_g)/(max(Fd_real.signals.values)-min(Fd_real.signals.values))

    case 2
sim('Simulations_chapter5_method3')
rho=u;
Estimation_error_p=Fd_est_p.signals.values-Fd_real.signals.values;
figure,
subplot(2,2,1), plot(zr.time,zr.signals.values),title('road profile zr')
subplot(2,2,2), plot(rho.time,rho.signals.values), title('scheduling parameter') 
subplot(2,2,3), plot(Fd_est_p.time,Fd_real.signals.values,Fd_est_p.time,Fd_est_p.signals.values,'r'), title('Force and Estimated force')
subplot(2,2,4), plot(Fd_est_p.time,Estimation_error_p), title('estimation error')
sgtitle('Scenario 2 Polytopic approach')

disp('rms = RMSD Polytopic approach')
RMSE_p= rms(Estimation_error_p)
disp('NRMSE Polytopic approach')
NRMSE_p=rms(Estimation_error_p)/(max(Fd_real.signals.values)-min(Fd_real.signals.values))


Estimation_error_g=Fd_est_g.signals.values-Fd_real.signals.values;
figure,
subplot(2,2,1), plot(zr.time,zr.signals.values),title('road profile zr')
subplot(2,2,2), plot(rho.time,rho.signals.values), title('scheduling parameter') 
subplot(2,2,3), plot(Fd_est_g.time,Fd_real.signals.values,Fd_est_g.time,Fd_est_g.signals.values,'r'), title('Force and Estimated force')
subplot(2,2,4), plot(Fd_est_g.time,Estimation_error_g), title('estimation error')
sgtitle('Scenario 2 Grid-based approach')

disp('rms = RMSD Grid-based approach')
RMSE_g= rms(Estimation_error_g)
disp('NRMSE Grid-based approach')
NRMSEg=rms(Estimation_error_g)/(max(Fd_real.signals.values)-min(Fd_real.signals.values))

    case 3
sim('Simulations_chapter5_method3')
rho=u;
Estimation_error_p=Fd_est_p.signals.values-Fd_real.signals.values;
figure,
subplot(2,2,1), plot(zr.time,zr.signals.values),title('road profile zr')
subplot(2,2,2), plot(rho.time,rho.signals.values), title('scheduling parameter') 
subplot(2,2,3), plot(Fd_est_p.time,Fd_real.signals.values,Fd_est_p.time,Fd_est_p.signals.values,'r'), title('Force and Estimated force')
subplot(2,2,4), plot(Fd_est_p.time,Estimation_error_p), title('estimation error')
sgtitle('Scenario 3 Polytopic approach')

disp('rms = RMSD Polytopic approach')
RMSE_p= rms(Estimation_error_p)
disp('NRMSE Polytopic approach')
NRMSE_p=rms(Estimation_error_p)/(max(Fd_real.signals.values)-min(Fd_real.signals.values))


Estimation_error_g=Fd_est_g.signals.values-Fd_real.signals.values;
figure,
subplot(2,2,1), plot(zr.time,zr.signals.values),title('road profile zr')
subplot(2,2,2), plot(rho.time,rho.signals.values), title('scheduling parameter') 
subplot(2,2,3), plot(Fd_est_g.time,Fd_real.signals.values,Fd_est_g.time,Fd_est_g.signals.values,'r'), title('Force and Estimated force')
subplot(2,2,4), plot(Fd_est_g.time,Estimation_error_g), title('estimation error')
sgtitle('Scenario 3 Grid-based approach')

disp('rms = RMSD Grid-based approach')
RMSE_g= rms(Estimation_error_g)
disp('NRMSE Grid-based approach')
NRMSEg=rms(Estimation_error_g)/(max(Fd_real.signals.values)-min(Fd_real.signals.values))

    otherwise
        disp('Choose 1, 2 or 3')
end

