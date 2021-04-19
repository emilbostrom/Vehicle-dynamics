function [dz] = qc_sh1(t,z,ZR)

% --------------------------------------------
% Quarter-car model - Semi-active damping
% Skyhook 1
% --------------------------------------------

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
c_max = 3000;
c_min = 500;

if dz1*(dz1-dz2) <= 0
    c = c_min;
else
    c = c_max;
end

ddz1 = (-c*(dz1-dz2)-ks*(z1-z2))/ms; % Sprung mass acceleration
ddz2 = (kt*zr-c*(dz2-dz1)-ks*(z2-z1)-kt*z2)/mus; % Unsprung mass acceleration

dz = [dz1 dz2 ddz1 ddz2]';