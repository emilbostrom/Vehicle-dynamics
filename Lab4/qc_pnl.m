function [dz] = qc_pnl(t,z,ZR)

% ----------------------------------------------
% Quarter-car model - Passive nonlinear damping
% ----------------------------------------------

z1  = z(1);  % Sprung mass displacement
z2  = z(2);  % Unsprung mass displacement
dz1 = z(3);  % Sprung mass velocity
dz2 = z(4);  % Unsprung mass velocity
zr  = ZR(t); % Road elevation

% Fill in parameters and equations to 
% calculate the accelerations of the 
% sprung and unsprung mass.

ms = 400;
mus = 40;
ks = 30e3;
kt = 200e3;

F_c1 = damping_force(dz1,dz2);
F_c2 = damping_force(dz2,dz1);

ddz1 = (-F_c1-ks*(z1-z2))/ms; % Sprung mass acceleration
ddz2 = (kt*zr-F_c2-ks*(z2-z1)-kt*z2)/mus; % Unsprung mass acceleration

dz = [dz1 dz2 ddz1 ddz2]';