% -----------------------------------
% 
% Frequency response diagrams
% 
% -----------------------------------

clear;


%% FR diagrams: Passive linear damping

% Compute the gain/transmissbility functions, over the frequency span 0.5
% to 20 Hz, using the analytic expression from Task 7.

% Define frequency range of interest
f_min = 0.5; f_max = 20; 
f = 10.^linspace(log10(f_min), log10(f_max), 100);

cmin = 500;
cnom = 1500;
cmax = 3000;
ms = 400;
mus = 40;
ks = 30e3;
kt = 200e3;

M = [ms 0;
     0 mus];
 
A = [ks -ks;
     -ks ks+kt];
 
D = [0;
     kt];

w = f.*2*pi;

gain = zeros(2,length(w));
count = 1;
for c = [cmin cnom cmax] % Define which damping constants to use
    
    % Calculate amplitude gain for sprung mass and tire deflection
    C = [c -c;
         -c c];
    
    for j = 1:length(w)
        gain(:,j) = abs((M*(1i*w(j))^2+C*1i*w(j)+A)\D-[0;1]);
    end
     
    % Plot the amplitude gain for the sprung mass (in fig1).
    figure(1);
    loglog(f,gain(1,:));
    title('Amplitude gain sprung mass')
    ylabel('Amplitude')
    xlabel('Frequency [Hz]')
    %legend([mass_plot,'cmin', 'cnom', 'cmax'])
    %loglog(f,#); hold on;
    hold on

    % Plot the amplitude gain for the tire deflection (in fig2).
    figure(2);
    loglog(f,gain(2,:));
    title('Amplitude gain tire deflection')
    ylabel('Amplitude')
    xlabel('Frequency [Hz]')
    %legend(['cmin' 'cnom' 'cmax'])
    hold on
    %loglog(f,#); hold on;
    count = count+1;
end

%% FR diagrams: Simulation based
% Simulations for passive linear damping, with nominal damping coefficient.
% Copy-paste and/or modify the script below and use it for the other models.
clear Tsm Ttd

% Define which model to run
for ii = 1:6
    switch ii
        % Uncomment to run model
%         case 1; qc = @(t,z,zr) qc_pl_cmin(t,z,zr);
%         case 2; qc = @(t,z,zr) qc_pl_cnom(t,z,zr);
%         case 3; qc = @(t,z,zr) qc_pl_cmax(t,z,zr);
         case 4; qc = @(t,z,zr) qc_pnl(t,z,zr);
%         case 5; qc = @(t,z,zr) qc_sh1(t,z,zr);
%         case 6; qc = @(t,z,zr) qc_sh2(t,z,zr);
        otherwise; continue;
    end

    % Define frequency range of interest
    f_min = 0.5; f_max = 20; 
    n_f = 25; % number of 'frequencies' to simulate
    f = 10.^linspace(log10(f_min), log10(f_max), n_f);

    z0 = zeros(4,1); % Inital conditions
    tf = 10;         % Simulation end time

    Aroad = 0.01; % Road amplitude

    for i = 1:length(f)
        clear sol t z1 z2 zr idx
        omega = 2*pi*f(i);
        ZR = @(t) Aroad*sin(omega*t); % Define the road profile (need to be updated for each new freq.)

        % Simulate
        sol = ode15s( @(t,z) qc(t,z,ZR), [0 tf], z0);

        % Extract results
        t  = sol.x;
        z1 = sol.y(1,:);
        z2 = sol.y(2,:);
        zr = ZR(t);

        % Calculate gain
        idx = [ceil(.75*length(sol.x)):length(sol.x)]; % Pick out the last 25% of the results (where init. cond. have little effect on the behavior)
        zr_rms   = sqrt( trapz(t(idx), zr(idx).^2) );
        z1_rms   = sqrt( trapz(t(idx), z1(idx).^2) );
        z2zr_rms = sqrt( trapz(t(idx), (z2(idx)-zr(idx)).^2) );

        Tsm(i) = z1_rms/zr_rms;
        Ttd(i) = z2zr_rms/zr_rms;
    end

% ------------------------------------------------------------------------
% Plot the frequency response diagrams with the above precalculated
% variables:
% 
%   - f : frequency vector for 0.5 to 20 Hz
% 
%   - Tsm : transmissibility for the frequency span in f
%     (amplitude gain for the sprung mass)
% 
%   - Ttd :  amplitude gain for the tire deflection for the frequency
%     span in f

% 
% ------------------------------------------------------------------------


    % Plot transmissibility (amplitude gain for sprung mass)
    figure(3);
    %C1 = {'b','r','k','g','m','c'};
    loglog(f,Tsm,'.-'); hold on
    title('Amplitude gain sprung mass simulation')
    ylabel('Amplitude')
    xlabel('Frequency [Hz]')
    %legend({'cmin','cnom','cmax','nonlin','sh1','sh2'})
    

    % Plot amplitude gain for the tire deflection ratio
    figure(4);
    %C2 = {'b','r','k','g','m','c'};
    loglog(f,Ttd,'.-'); hold on
    title('Amplitude gain tire deflection simulation')
    ylabel('Amplitude')
    xlabel('Frequency [Hz]')
    %legend({'cmin','cnom','cmax','nonlin','sh1','sh2'})
    
end


