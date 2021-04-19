function delta = sin_steering(t)

    amplitude = deg2rad(0.1);
    offset = 1; % When to start steering
    delta = amplitude*sin((t-offset)*2).*heaviside(pi-t+offset).*heaviside(t-offset);

end

