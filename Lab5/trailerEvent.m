function [value, isterminal, direction] = trailerEvent(t, y)
% Stops integration if hitch angle is larger than 90 degrees or
% if longitudinal velocity is small.
maxHitchAngle = deg2rad(90);
minVelocity = 1e-3;
value      = (abs(y(5)) > maxHitchAngle | y(1) < minVelocity)-0.5;
isterminal = 1;   % Stop the integration
direction  = 0;