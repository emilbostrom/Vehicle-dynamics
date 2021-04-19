% --------------------------------------------
% Simulate quarter-car models, excited by a 
% sinusoidal road profile input.
% --------------------------------------------

clear all

%% Simulate 'qc_pl_cnom.m'
% Note that you have to change model name
% on row 16 if you want to use this
% script for other models.

clear sol t z1 z2 dz1 dz2 ddz1 ddz2 zr

% Define which model to run
qc = @(t,z,ZR) qc_pnl(t,z,ZR);

v  = 30/3.6; % vehicle velocity
xf = 50;     % total distance
tf = xf/v;   % end time

% Create road profile function
Aroad = 0.1; % amplitude
Lroad = 2;   % wavelength
omega = 2*pi/Lroad;
ZR = @(t) Aroad*sin(omega*t);

% Simulate
z0 = zeros(4,1); % initial conditions
sol = ode15s(@(t,z) qc(t,z,ZR),[0 tf],z0);

% Extract results
t   = sol.x;
z1  = sol.y(1,:);
z2  = sol.y(2,:);
dz1 = sol.y(3,:);
dz2 = sol.y(4,:);

for i = 1:length(t) % Retrive accelerations of ms and mus
    z = [z1(i); z2(i); dz1(i); dz2(i)];
    dz = qc(t(i),z,ZR);
    ddz1(i) = dz(3);
    ddz2(i) = dz(4);
end

for i = 1:length(t) % Create a vector with the road profile
    zr(i) = ZR(t(i));
end

% Plot result
figure
subplot(411)
plot(t,zr)
ylabel('zr')
subplot(412)
plot(t,z1)
ylabel('z1')
subplot(413)
plot(t,ddz1)
ylabel('ddz1')
subplot(414)
plot(t,z2-zr)
ylabel('z2-zr')
xlabel('t')