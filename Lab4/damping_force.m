function [F_c] = damping_force(z_dot1,z_dot2)

c_max = 3000;
c_min = 500;
v_min = 0.2;

m1 = (-v_min)*c_min-(-v_min)*c_max;
m2 = v_min*c_max-v_min*c_min;

v = z_dot1-z_dot2;

if  v < -v_min
    F_c = v*c_min-m1;
elseif v > -v_min && v < v_min
    F_c = v*c_max;
else
    F_c = v*c_min+m2;
end

