% % % % % % % % % % % % % % % % % 
%
% TSFS02
% Exercise 2 - Lateral Dynamics
%
% Task 9: Ramp Steer
% 
% % % % % % % % % % % % % % % % % 

clear all

%% Load data
load vehicleParameters.mat
load tireParameters.mat

load rsMeasData.mat  % Ramp steer measurement data

% Modify vehicle properties
% For example:
%       k = k*0.90;


%% Simulate
meas = rs;
% Single-Track with Linear tire model
x0 = zeros(2,1);
odeOpts = odeset('RelTol',1e-4);
[tout,xout] = ode45(@(t,x) STL(t,x,meas.t,meas.delta,meas.vx,Caf,Car,L,l1,m,Iz),meas.t([1,end]),x0,odeOpts);
stl.t = tout;
stl.vy = xout(:,1);
stl.Omegaz = xout(:,2);
vydot = [0, diff(stl.vy)' ./ diff(stl.t)'];
stl.ay = vydot + [interp1(meas.t, meas.vx, stl.t).*stl.Omegaz]';
stl.alphaf = interp1(meas.t,meas.delta,stl.t) - (stl.vy + stl.Omegaz*l1)./interp1(meas.t,meas.vx,stl.t);
stl.alphar = -(stl.vy - stl.Omegaz*(L-l1))./interp1(meas.t,meas.vx,stl.t);

% Single-Track with Magic Formula
x0 = zeros(2,1);
odeOpts = odeset('RelTol',1e-4);
[tout,xout] = ode45(@(t,x) STMF(t,x,meas.t,meas.delta,meas.vx,MFcoef,L,l1,m,Iz),meas.t([1,end]),x0,odeOpts);
stmf.t = tout;
stmf.vy = xout(:,1);
stmf.Omegaz = xout(:,2);
vydot = [0, diff(stmf.vy)' ./ diff(stmf.t)'];
stmf.ay = vydot + [interp1(meas.t, meas.vx, stmf.t).*stmf.Omegaz]';
stmf.alphaf = interp1(meas.t,meas.delta,stmf.t) - (stmf.vy + stmf.Omegaz*l1)./interp1(meas.t,meas.vx,stmf.t);
stmf.alphar = -(stmf.vy - stmf.Omegaz*(L-l1))./interp1(meas.t,meas.vx,stmf.t);


%% Plot
figure('position',[339 250 1098 697]);
subplot(321)
plot(meas.t,meas.delta*180/pi,'c','linewidth',2);
xlim([0 meas.t(end)])
ylabel('Steer angle, [deg]')
title('Ramp Steer')

subplot(323)
plot(meas.t,meas.Omegaz,'c','linewidth',2); hold on
plot(stl.t,stl.Omegaz,'--r')
plot(stmf.t,stmf.Omegaz,'b')
xlim([0 meas.t(end)])
ylabel('Yaw rate, [rad/s]')

subplot(325)
plot(meas.t,meas.ay/g,'c','linewidth',2); hold on
plot(stl.t,stl.ay/g,'--r')
plot(stmf.t,stmf.ay/g,'b')
ylabel('a_y, [g]')
xlim([0 meas.t(end)])

subplot(222)
plot(meas.t,meas.alphaf*180/pi,'c','linewidth',2); hold on
plot(stl.t,stl.alphaf*180/pi,'--r')
plot(stmf.t,stmf.alphaf*180/pi,'b')
ylabel('\alpha_f, [deg]')
xlim([0 meas.t(end)])

subplot(224)
plot(meas.t,meas.alphar*180/pi,'c','linewidth',2); hold on
plot(stl.t,stl.alphar*180/pi,'--r')
plot(stmf.t,stmf.alphar*180/pi,'b')
ylabel('\alpha_r, [deg]')
xlim([0 meas.t(end)])


legend('Measurement','Sim: Linear','Sim: Magic Formula','location','best')
