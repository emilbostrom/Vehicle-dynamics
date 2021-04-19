%%% TestLinearSystem
clear;

%% Get symbolic expressions
[Msym, Dsym, Cdeltafsym, Cdeltatsym, CDeltaMsym] = getLinearSystem();

%% Calculate symbolic expressions for A, Bdeltaf, Bdeltat, BDeltaM
% dx = A*x + Bdeltaf*x + Bdeltat*x + BDeltaM*x

Asym = Msym\Dsym;
Bdeltafsym = Msym\Cdeltafsym;
Bdeltatsym = Msym\Cdeltatsym;
BDeltaMsym = Msym\CDeltaMsym;

%% Load some parameters to replace symbolic expressions 

p = load('params');
p.vx = 1;

A = double(subs(Asym,p));
Bdeltaf = double(subs(Bdeltafsym,p));
Bdeltat = double(subs(Bdeltatsym,p));
BDeltaM = double(subs(BDeltaMsym,p));


%% Compare output with correct values

x = [1 2 1 4].';
u1 = 0.1;
u2 = 0.2;
u3 = 1000;
dx = A*x+Bdeltaf*u1+Bdeltat*u2+BDeltaM*u3

dx_correct = [-72.5829, -396.7133, 396.6950, 1.0000].'
