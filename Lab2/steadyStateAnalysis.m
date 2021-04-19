% % % % % % % % % % % % % % % % % 
%
% TSFS02
% Exercise 2 - Lateral Dynamics
% 
% Task 7: Steady State Cornering
% 
% % % % % % % % % % % % % % % % % 

% Cell 1
clear all

%% Cell 1: Load data
load vehicleParameters.mat
load tireParameters.mat
Bf = MFcoef.Bf;
Cf = MFcoef.Cf;
Df = MFcoef.Df;
Br = MFcoef.Br;
Cr = MFcoef.Cr;
Dr = MFcoef.Dr;

load ssMeasData.mat % Steady state measurement data
% The variables from ssMeasData.mat lies in a struct called "ss".
% For example, the variable "delta" is reached with ss.delta.

R  = 27.75;   % radius of roundabout
ay = 0:.05:8; % range of lateral accelerations to evaluate

%% Cell 2: Calculate tire forces
# Fyf = 
# Fyr = 

%% Cell 3: Single track with linear tire model

# alphaf_lin = 
# alphar_lin = 
# delta_lin = 

%% Cell 4: Single track with Magic Formula model

# alphaf_mf = 
# alphar_mf = 
# delta_mf =

%% Cell 5: Plot
figure
% Plot measurement data in the handling diagram
plot( # , # ,'.c','markersize',5); hold on

% Plot the single track with linear tire model
plot( # , # , '--r')

% Plot the single track with Magic Formula tire model
plot( # , # ,'b')

xlabel('...')
ylabel('...')

title('Handling diagram')
legend('Measurments','Single Track: Linear','Single Track: Magic Formula','location','southwest')
