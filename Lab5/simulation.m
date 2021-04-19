clear

% Set to true to simulate and plot closed loop systems
plot_controlled = true;

[Msym, Dsym, Cdeltafsym, Cdeltatsym, CDeltaMsym] = getLinearSystem();

Asym = Msym\Dsym;
Bdeltafsym = Msym\Cdeltafsym;
Bdeltatsym = Msym\Cdeltatsym;
BDeltaMsym = Msym\CDeltaMsym;

% Initial speed
vx0 = 30;

% Load parameters
p = load('params');
p.vx = vx0;

%% Task 6: Modify parameters
% e.g. p.Mc = 2*p.Mc



%% Task 7: Implement LQR
Q = 10000*[1 0 0 0;
     0 1 0 0;
     0 0 1 0;
     0 0 0 1];
 
Q2 = [100000 0 0 0;
     0 100000 0 0;
     0 0 1*1e9 0;
     0 0 0 1*1e8];
 
A = double(subs(Asym,p));
Bdeltaf = double(subs(Bdeltafsym,p));
Bdeltat = double(subs(Bdeltatsym,p));
BDeltaM = double(subs(BDeltaMsym,p));

L = lqr(A,Bdeltat,Q,1e-5);

L1 = lqr(A,BDeltaM,Q2,1e-5);

deltat_func = @(t,vy,r,dpsi,psi) -L*[vy,r,dpsi,psi]'; % Change to your controller


DM_func = @(t,vy,r,dpsi,psi) -L1*[vy,r,dpsi,psi]'; % Change to your controller

%% Simulate
tspan = [0 20];
y0 = [vx0 0 0 0 0 0 0 0];

% Default inputs
deltaf_func = @(t,vy,r,dpsi,psi) sin_steering(t);
deltar_func0 = @(t,vy,r,dpsi,psi) zeros(size(t));
deltat_func0 = @(t,vy,r,dpsi,psi) zeros(size(t));
DF_func0 = @(t,vy,r,dpsi,psi) zeros(size(t));
DM_func0 = @(t,vy,r,dpsi,psi) zeros(size(t));

% Simulate uncontrolled system
[M,rhs] = car_trailer_model(deltaf_func,deltar_func0,deltat_func0,DF_func0,DM_func0,p);
opt = odeset('Mass',M,'Events', @trailerEvent);
[t_u,y_u] = ode15s(rhs,tspan,y0,opt);

if plot_controlled
    % Simulate active trailer steering systen
    [M,rhs] = car_trailer_model(deltaf_func,deltar_func0,deltat_func,DF_func0,DM_func0,p);
    opt = odeset('Mass',M,'Events', @trailerEvent);
    [t_s,y_s] = ode15s(rhs,tspan,y0,opt);

    % Simulate torque vectoring systen
    [M,rhs] = car_trailer_model(deltaf_func,deltar_func0,deltat_func0,DF_func0,DM_func,p);
    opt = odeset('Mass',M,'Events', @trailerEvent);
    [t_b,y_b] = ode15s(rhs,tspan,y0,opt);
end


%% Plot
figure('position',[339 150 1198 797]);
subplot(421)
plot(t_u,deltaf_func(t_u,y_u(:,2),y_u(:,3),y_u(:,4),y_u(:,5))*180/pi,'k','linewidth',2);
ylabel('Car steer angle \delta_f, [deg]')
title(['v_0 = ', num2str(vx0*3.6,3), ' km/h'])

subplot(423)
plot(t_u,deltat_func0(t_u,y_u(:,2),y_u(:,3),y_u(:,4),y_u(:,5))*180/pi,'k','linewidth',2);
if plot_controlled
    hold on
    plot(t_s,arrayfun(deltat_func,t_s,y_s(:,2),y_s(:,3),y_s(:,4),y_s(:,5))*180/pi,'b','linewidth',2);
    hold off
end
ylabel('Trailer steer angle \delta_t, [deg]')

subplot(425)
plot(t_u,DM_func0(t_u,y_u(:,2),y_u(:,3),y_u(:,4),y_u(:,5)),'k','linewidth',2);
if plot_controlled
    hold on
    plot(t_b,arrayfun(DM_func,t_b,y_b(:,2),y_b(:,3),y_b(:,4),y_b(:,5)),'r','linewidth',2);
    hold off
end
ylabel('Yaw Moment \Delta M, [Nm]')

subplot(427)
plot(t_u,(y_u(:,1).^2+y_u(:,2).^2).^0.5,'k','linewidth',2); 
if plot_controlled
    hold on
    plot(t_s,(y_s(:,1).^2+y_s(:,2).^2).^0.5,'b','linewidth',2);
    plot(t_b,(y_b(:,1).^2+y_b(:,2).^2).^0.5,'r','linewidth',2);
    hold off
    legend('without control', 'active trailer steering', 'torque vectoring', 'Location', 'southwest')
end
xlabel('Time, [s]')
ylabel('Car speed, [m/s]')

subplot(422)
plot(t_u,y_u(:,2),'k','linewidth',2); 
if plot_controlled
    hold on
    plot(t_s,y_s(:,2),'b','linewidth',2);
    plot(t_b,y_b(:,2),'r','linewidth',2);
    hold off
end
ylabel('Lateral speed, [m/s]')

subplot(424)
plot(t_u,y_u(:,3)*180/pi,'k','linewidth',2); 
if plot_controlled
    hold on
    plot(t_s,y_s(:,3)*180/pi,'b','linewidth',2);
    plot(t_b,y_b(:,3)*180/pi,'r','linewidth',2);
    hold off
end
ylabel('Yaw rate, [deg/s]')

subplot(426)
plot(t_u,y_u(:,4)*180/pi,'k','linewidth',2); 
if plot_controlled
    hold on
    plot(t_s,y_s(:,4)*180/pi,'b','linewidth',2);
    plot(t_b,y_b(:,4)*180/pi,'r','linewidth',2);
    hold off
end
ylabel('Hitch rate, [deg/s]')

subplot(428)
plot(t_u,y_u(:,5)*180/pi,'k','linewidth',2); 
if plot_controlled
    hold on
    plot(t_s,y_s(:,5)*180/pi,'b','linewidth',2);
    plot(t_b,y_b(:,5)*180/pi,'r','linewidth',2);
    hold off
end
xlabel('Time, [s]')
ylabel('Hitch angle, [deg]')