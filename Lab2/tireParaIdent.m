% % % % % % % % % % % % % % % % % 
%
% TSFS02
% Exercise 2 - Lateral Dynamics
%
% Task 6: Identify Tire Parameters
% 
% % % % % % % % % % % % % % % % % 

% Cell 1
clear all

%% Cell 2: Load data
% Load vehicle parameters
load('vehicleParameters.mat') % L, l1, m, Iz, g

% Load measurement data from double lane change tests
load('tireParaMeas.mat') % delta, ay, Omegaz, dOmegaz, vx, vy

%% Cell 3: Calculate slip angles and lateral forces
l2 = L-l1;
g = 9.81;

alphaf = delta-(l1.*Omegaz+vy)./vx;
alphar = (l2.*Omegaz-vy)./vx;
Fyf = (m.*ay+(Iz.*dOmegaz)./l2)./(cos(delta).*(1+l1/l2));
Fyr = (m.*ay-(Iz.*dOmegaz)./l1)./(1+l2/l1);

%% Cell 4: Plot Fy-alpha diagram of measurement data
fHandle = figure('position',[560 527 1116 420]);
subplot(121)
plot(alphaf*180/pi,Fyf,'.c')
title('Front suspension')
ylabel('F_{y,f}, [N]')
xlabel('\alpha_f , [deg]')
subplot(122)
plot(alphar*180/pi,Fyr,'.c')
title('Rear suspension')
ylabel('F_{y,r}, [N]')
xlabel('\alpha_r , [deg]')

%% Cell 5: Determine cornering stiffness
% Cornering stiffnesses

Caf = alphaf\(Fyf/2); 
Car = alphar\(Fyr/2);

Caf = 4.9e4;
Car = 4.6e4;

% slip angle range (only for plotting)
af = [-8:.01:8]*pi/180;
ar = [-6:.01:6]*pi/180;

% Force equations, linear model
FyfL = 2*Caf*af;
FyrL = 2*Car*ar;

%% Cell 6: Plot cornering stiffness
figure();
subplot(121)
plot(alphaf*180/pi,Fyf,'.c'); hold on
plot(af*180/pi,FyfL,'--r'); hold off
title('Front suspension')
ylabel('F_{y,f}, [N]')
xlabel('\alpha_f , [deg]')
subplot(122)
plot(alphar*180/pi,Fyr,'.c'); hold on
plot(ar*180/pi,FyrL,'--r'); hold off
title('Rear suspension')
ylabel('F_{y,r}, [N]')
xlabel('\alpha_r , [deg]')

%% Cell 7: Determine Magic Formula parameters

% Front Magic Formula parameters

Cf_init = 1.9584;
mu_y_init = 1;
Fz_init = m*g*l2/(2*L);
Df_init = mu_y_init*Fz_init;
Df_init = 4.194e3;
Bf_init = Caf/(Cf_init*Df_init);
Bf_init = 13.38;

start_values = [Cf_init Df_init Bf_init];
MF_f = zeros(1,3);

% fit_equation = fittype(@(Bf,Cf,Df,alphaf) Df.*sin(Cf.*atan(Bf.*alphaf)));
% fit_data = fit(alphaf,Fyf,fit_equation,start_values);

fit_equation = @(MF_f,alphaf)(MF_f(2).*sin(MF_f(1).*atan(MF_f(3).*alphaf)));
MF_f = lsqcurvefit(fit_equation,start_values,alphaf,Fyf);

Cf = MF_f(1);
Df = MF_f(2);
Bf = MF_f(3);

% Rear Magic Formula parameters

% Cr = 
% Dr = 
% Br = 

% Force equations, Magic Formula
FyfMF = Df*sin(Cf*atan(Bf*af));
% FyrMF = Dr*sin(Cr*atan(Br*ar));

%% Cell 8: Plot tire models on top of measurement data
figure(fHandle);
subplot(121)
plot(alphaf*180/pi,Fyf,'.c'); hold on
plot(af*180/pi,FyfL,'--r')
plot(af*180/pi,FyfMF,'b'); hold off
title('Front suspension')
ylabel('F_{y,f}, [N]')
xlabel('\alpha_f , [deg]')
subplot(122)
plot(alphar*180/pi,Fyr,'.c'); hold on
plot(ar*180/pi,FyrL,'--r')
plot(ar*180/pi,FyrMF,'b'); hold off
title('Rear suspension')
ylabel('F_{y,r}, [N]')
xlabel('\alpha_r , [deg]')

%% Cell 9: Save tire parameters
MFcoef.Bf = Bf;
MFcoef.Cf = Cf;
MFcoef.Df = Df;
MFcoef.Br = Br;
MFcoef.Cr = Cr;
MFcoef.Dr = Dr;
save('tireParameters.mat','Caf','Car','MFcoef')
