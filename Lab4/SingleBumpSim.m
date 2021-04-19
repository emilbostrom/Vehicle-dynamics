% -------------------------------------------------
% 
% Simulate the QC-model driving over a single bump
% 
% -------------------------------------------------

clear;
fh = figure;
disp(' ')
disp(' ')

%% Specify velocity and road profile
v  = 70/3.6; % Velocity
xf = 10;     % Total distance
tf = xf/v;   % Final time

z0 = zeros(4,1); % Initial conditions

% Define road profile
a1 = 0.1033; b1 = 0.05004; c1 = 0.02796;
ZR = @(t)  a1*exp(-((v*t-b1)/c1).^2);

%% Simulate: Passive damping, nominal damping coefficient (cnom)

for ii = 1:6
    switch ii
        case 1; qc = @(t,z,zr) qc_pl_cmin(t,z,zr);
        case 2; qc = @(t,z,zr) qc_pl_cnom(t,z,zr);
        case 3; qc = @(t,z,zr) qc_pl_cmax(t,z,zr);
        case 4; qc = @(t,z,zr) qc_pnl(t,z,zr);
        case 5; qc = @(t,z,zr) qc_sh1(t,z,zr);
        case 6; qc = @(t,z,zr) qc_sh2(t,z,zr);
        otherwise; continue;
    end

    clear sol t z1 z2 dz1 dz2 ddz1 ddz2 zr

    % Run simulation
    sol = ode15s(@(t,z) qc(t,z,ZR),[0 tf],z0);

    % Retrieve results
    t   = sol.x;
    z1  = sol.y(1,:);
    z2  = sol.y(2,:);
    dz1 = sol.y(3,:);
    dz2 = sol.y(4,:);
    % Retrive accelerations of ms and mus
    for i = 1:length(t)
        z = [z1(i); z2(i); dz1(i); dz2(i)];
        dz = qc(t(i),z,ZR);
        ddz1(i) = dz(3);
        ddz2(i) = dz(4);
    end
    % Create road profile vector
    zr = ZR(t);

    % Plot results (modify if you miss any variables)
    C2 = {'b','r','k','g','m','c'};
    figure(1)
    subplot(411)
    plot(t,zr,'color',C2{ii}); hold on
    ylabel('zr')
    subplot(412)
    plot(t,z1-z2,'color',C2{ii}); hold on
    ylabel('z1-z2')
    subplot(413)
    plot(t,ddz1,'color',C2{ii}); hold on
    ylabel('ddz1')
    subplot(414)
    plot(t,z2-zr,'color',C2{ii}); hold on
    ylabel('z2-zr')
    legend({'cmin','cnom','cmax','nonlin','sh1','sh2'})
    

    % Calculate performance indices
    J_comfort  = sqrt(trapz(t,ddz1.^2)/tf);
    J_handling = sqrt(trapz(t,(z2-zr).^2)/tf);
    disp([num2str(ii), ': J_comfort = ', num2str(J_comfort), '  J_handling = ', num2str(J_handling)])

end

 
