function [Msym, Dsym, Cdeltafsym, Cdeltatsym, CDeltaMsym] = getLinearSystem()
%GETLINEARSYSTEM
% Calculate symbolic expression for linear system
% Mdx = Dx + Cu

    % Variables and inputs
    syms x1 dx1 x2 dx2 x4 dx4 x3 dx3 deltaf deltat DeltaM
    vars = [dx1,dx2,dx3,dx4,x1,x2,x3,x4,deltaf,deltat,DeltaM];

    % Parameters
    syms vx mc Ic l1c l2c l3c mt It l2t l1t kf kr kt
    
    
    %% Task 5: Enter system of equations below
   
    
    Omegazt = x2+x3; 
    Omegazt_dot = dx2+dx3;
    vyt_dot = dx1-Omegazt_dot*l1t-dx2*l3c-x3*vx;
    vyt = x1-Omegazt*l1t-x2*l3c;

    af = deltaf-(x1+l1c*x2)/vx;
    ar = (l2c*x2-x1)/vx;
    at = deltat-(vyt-l2t*Omegazt)/vx+x4;

    Fyr = 2*kr*ar;
    Fyf = 2*kf*af;
    Fyt = 2*kt*at;

    Fyht = mt*(vyt_dot+Omegazt*vx)-Fyt;
    Fyhc = -Fyht;

    f1 = mc*dx1+mc*vx*x2-Fyf-Fyr-Fyhc;
    f2 = Ic*dx2-l1c*Fyf+l2c*Fyr-l3c*Fyht-DeltaM;
    f3 = Omegazt_dot*It-Fyht*l1t+Fyt*l2t;
    f4 = -dx4 + x3;



    
    %% Compute matrices

    fs = [f1 f2 f3 f4];
    Msym(4,4) = sym(0);
    Dsym(4,4) = sym(0);
    Cdeltafsym(4,1)=sym(0);
    Cdeltatsym(4,1)=sym(0);
    CDeltaMsym(4,1)=sym(0);
    for jj = 1:4
        f = fs(jj);

        % M matrix
        for ii = 1:4
            var = vars(ii);
            Msym(jj,ii) = -diff(f,var);
        end
        % D(vx) matrix
        for ii = 1:4
            var = vars(4+ii);
            Dsym(jj,ii) = diff(f,var);
        end
        % Cdeltaf matrix
        Cdeltafsym(jj,1) = diff(f,deltaf);
        % Cdeltat matrix
        Cdeltatsym(jj,1) = diff(f,deltat);
        % CDeltaM matrix
        CDeltaMsym(jj,1) = diff(f,DeltaM);
    end

end

