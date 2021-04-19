ms = 400;
mus = 40;
ks = 30e3;
kt = 200e3;

A1 = ms*mus;
B1 = ms*ks+ms*kt+mus*ks;
C1 = ks*kt;

omega_2_1 = (B1+sqrt(B1^2-4*A1*C1))/(2*A1);
omega_2_2 = (B1-sqrt(B1^2-4*A1*C1))/(2*A1);
omega1 = sqrt(omega_2_1);
omega2 = sqrt(omega_2_2);

%% Dämpningskraft
c_max = 3000;
c_min = 500;
v_min = 0.2;

v = -0.7:0.01:0.7;
F_c = zeros(length(v),1);

m1 = (-v_min)*c_min-(-v_min)*c_max;
m2 = v_min*c_max-v_min*c_min;

for i = 1:length(v)
    if v(i) < -v_min
        F_c(i) = v(i)*c_min-m1;
    elseif v(i) > -v_min && v(i) < v_min
        F_c(i) = v(i)*c_max;
    else
        F_c(i) = v(i)*c_min+m2;
    end
end

figure()
plot(v,F_c)
xlabel('Damper velocity [m/s]')
ylabel('Damper Force [N]')
title('Force characteristics for damper')

%% Uppgift 9

