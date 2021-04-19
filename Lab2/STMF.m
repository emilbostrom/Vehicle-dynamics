function dx = STMF(t,x,T,Delta,Vx,MFcoef,L,l1,m,Iz)

% 
% SINGLE-TRACK MODEL with Magic Formula tire model
% 
% Loaded parameters:
% Bf, Cf, Df, Br, Cr, Dr, L, l1, m, Iz
%

% Load tire parameters
Bf = MFcoef.Bf;
Cf = MFcoef.Cf;
Df = MFcoef.Df;
Br = MFcoef.Br;
Cr = MFcoef.Cr;
Dr = MFcoef.Dr;

% Inputs
delta  = interp1(T,Delta,t);
vx     = interp1(T,Vx,t);

% States
vy     = x(1);
Omegaz = x(2);

% Equations
# dvy = 
# dOmegaz = 

% Outputs
dx = [dvy; dOmegaz];