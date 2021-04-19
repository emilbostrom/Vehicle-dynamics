function [M, RHS] = car_trailer_model(deltaf_func,deltar_func,deltat_func,DF_func,DM_func,p)
%Nonlinear CAR TRAILER MODEL
% RHS: forces right hand side equations of car-trailer system
% M: Mass matrix of car-trailer system

    % Car
    mc = p.mc; % Car mass
    Ic = p.Ic; % Car yaw inertia
    a = p.l1c; % Distance from car CoM to front axle
    b = p.l2c; % Distance from car CoM to rear axle
    h1 = p.l3c; % Distance from car CoM to hitch

    % Trailer
    mt = p.mt; % Trailer mass
    It = p.It; % Trailer yaw inertia
    c = p.l2t; % Distance from trailer CoM to trailer axle
    h2 = p.l1t; % Distance from trailer CoM to hitch

    % Tires
    kf = 2*p.kf; % Front wheel tire stiffness
    kr = 2*p.kr; % Rear wheel tire stiffness
    kt = 2*p.kt; % Trailer wheel tire stiffness
    
    function RHS = rhs(t,vx,vy,r,dpsi,psi,theta)
        % Inputs
        deltaf = deltaf_func(t,vy,r,dpsi,psi);
        deltar = deltar_func(t,vy,r,dpsi,psi);
        deltat = deltat_func(t,vy,r,dpsi,psi);
        DF = DF_func(t,vy,r,dpsi,psi);
        DM = DM_func(t,vy,r,dpsi,psi);

        % Slip angles
        alphaf = deltaf - atan((vy+r*a)/vx);
        alphar = deltar - atan((vy-r*b)/vx);
        alphat = deltat - atan(((vy-r*h1)*cos(psi) - vx*sin(psi) - (h2+c)*(r+dpsi))/(vx*cos(psi)+(vy-r*h1)*sin(psi)));

        % Tire forces
        Fyf = kf*alphaf;
        Fyr = kr*alphar;
        Fyt = kt*alphat;

        Rx = vx*cos(theta)-vy*sin(theta);
        Ry = vx*sin(theta)+vy*cos(theta);
        Rtheta = r;
        
        Rvx = -Fyf*sin(deltaf) - Fyr*sin(deltar) - Fyt*sin(psi+deltat) ...
            +(mc+mt)*vy*r - mt*h1*r^2 - mt*h2*(r+dpsi)^2*cos(psi) + DF;

        Rvy = Fyf*cos(deltaf) + Fyr*cos(deltar) + Fyt*cos(psi+deltat) ...
            -(mc+mt)*vx*r - mt*h2*(r+dpsi)^2*sin(psi);

        Rr = a*Fyf*cos(deltaf) - b*Fyr*cos(deltar) - Fyt*(h2+c+h1*cos(psi+deltat)) ...
            +mt*h2*(vy*sin(psi) + vx*cos(psi))*r + mt*h1*vx*r + mt*h1*h2*(dpsi^2+2*r*dpsi)*sin(psi) + DM;

        Rpsi = dpsi;
        Rdpsi = -Fyt*(h2 + c) + mt*h2*(vx*cos(psi)+vy*sin(psi))*r - mt*h1*h2*r^2*sin(psi);
        
        RHS = [Rvx;Rvy;Rr;Rdpsi;Rpsi;Rx;Ry;Rtheta];
    end
    RHS = @(t,y) rhs(t,y(1),y(2),y(3),y(4),y(5),y(8));
    
    
    function M = m(psi)
    % Moment of inertia components
    I22 = Ic+It+mt*(h1^2+h2^2)+2*mt*h1*h2*cos(psi);
    I33 = It+mt*h2^2;
    I23 = I33 + mt*h1*h2*cos(psi);
    I32 = I23;

    M = [(mc+mt)        0                       mt*h2*sin(psi)          mt*h2*sin(psi)
         0              (mc+mt)                 -mt*(h1+h2*cos(psi))    -mt*h2*cos(psi)
         mt*h2*sin(psi) -mt*(h1+h2*cos(psi))    I22                     I23
         mt*h2*sin(psi) -mt*(h2*cos(psi))       I32                     I33];
    M(5:8,5:8) = eye(4);
    end
    M = @(t,y) m(y(5));

end

