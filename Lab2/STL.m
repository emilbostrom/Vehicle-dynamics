function dx = STL(t,x,T,Delta,Vx,Caf,Car,L,l1,m,Iz)

%
% SINGLE-TRACK MODEL with Linear tire model
% 
% Loaded parameters:
% Caf, Car, L, l1, m, Iz
%

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