% Example Method 1 chapter 5
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
xest0=0*[zs0,zus0,zsdt0,zusdt0,Fer0]';

% data for simulation tests
Ts=0.005;
load('time_simulation.mat')
load('road_simulation.mat')

%%
scenario = input('Which scenario would you like to run: 1, 2 or 3? Answer = ');

switch scenario
    case 1
sim('Simulations_chapter5_method1')
Estimation_error=Fd_est.signals.values-Fd_real.signals.values;


figure,
subplot(2,2,1), plot(zr.time,zr.signals.values),title('road profile zr')
subplot(2,2,2), plot(rho.time,rho.signals.values), title('scheduling parameter') 
subplot(2,2,3), plot(Fd_est.time,Fd_real.signals.values,Fd_est.time,Fd_est.signals.values,'r'), title('Force and Estimated force')
subplot(2,2,4), plot(Fd_est.time,Estimation_error), title('estimation error')
sgtitle('Scenario 1')

disp('rms = RMSD')
RMSE= rms(Estimation_error)
disp('NRMSE')
NRMSE=rms(Estimation_error)/(max(Fd_real.signals.values)-min(Fd_real.signals.values))

    case 2
sim('Simulations_chapter5_method1')
Estimation_error=Fd_est.signals.values-Fd_real.signals.values;
figure, 
subplot(2,2,1), plot(u.time,u.signals.values-mean(u.signals.values),zr.time,zr.signals.values*10),title('control signal u & road profile zr')
subplot(2,2,2), plot(rho.time,rho.signals.values), title('scheduling parameter') 
subplot(2,2,3), plot(Fd_est.time,Fd_real.signals.values,Fd_est.time,Fd_est.signals.values,'r'), title('Force and Estimated force')
subplot(2,2,4), plot(Fd_est.time,Estimation_error), title('Estimation error')
sgtitle('Scenario 2')

disp('rms = RMSD')
RMSE= rms(Estimation_error)
disp('NRMSE')
NRMSE=rms(Estimation_error)/(max(Fd_real.signals.values)-min(Fd_real.signals.values))

    case 3
sim('Simulations_chapter5_method1')
Estimation_error=Fd_est.signals.values-Fd_real.signals.values;
figure, 
subplot(2,2,1), plot(u.time,u.signals.values-mean(u.signals.values),zr.time,zr.signals.values*10),title('control signal u & road profile zr')
subplot(2,2,2), plot(rho.time,rho.signals.values), title('scheduling parameter') 
subplot(2,2,3), plot(Fd_est.time,Fd_real.signals.values,Fd_est.time,Fd_est.signals.values,'r'), title('Force and Estimated force')
subplot(2,2,4), plot(Fd_est.time,Estimation_error), title('Estimation error')
sgtitle('Scenario 3')

disp('rms = RMSD')
RMSE= rms(Estimation_error)
disp('NRMSE')
NRMSE=rms(Estimation_error)/(max(Fd_real.signals.values)-min(Fd_real.signals.values))

    otherwise
        disp('Choose 1, 2 or 3')
end

