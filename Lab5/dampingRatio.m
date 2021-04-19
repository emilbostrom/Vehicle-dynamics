%%% Damping ratios
clear;

% Set to true to plot damping ratios of closed-loop systems
plot_controlled = true;

%% Get symbolic expressions
[Msym, Dsym, Cdeltafsym, Cdeltatsym, CDeltaMsym] = getLinearSystem();

%% Task 6: Calculate symbolic expressions for A, Bdeltaf, Bdeltat, BDeltaM
% dx = A*x + Bdeltaf*x + Bdeltat*x + BDeltaM*x

Asym = Msym\Dsym;
Bdeltafsym = Msym\Cdeltafsym;
Bdeltatsym = Msym\Cdeltatsym;
BDeltaMsym = Msym\CDeltaMsym;

%% Load Default parameters
p = load('params');
p.vx = 1;



%% Task 6: Modify parameters
% e.g. p.Mc = 2*p.Mc

p.mt = p.mt*2;
p.It = p.It*2;

Q = 10*[1 0 0 0;
     0 1 0 0;
     0 0 1 0;
     0 0 0 1];
 
Q2 = [1 0 0 0;
     0 1 0 0;
     0 0 1*1e9 0;
     0 0 0 1*1e8];

%% Compute damping ratio for different vehicle speeds.

vxs = linspace(1,50);
for ii = 1:numel(vxs)
    % Update velocity parameter
    p.vx = vxs(ii);
    p.l2t = 0.5;
    p.l1t = 6-p.l2t;
    
    A = double(subs(Asym,p));
    Bdeltaf = double(subs(Bdeltafsym,p));
    Bdeltat = double(subs(Bdeltatsym,p));
    BDeltaM = double(subs(BDeltaMsym,p));

    
    % Task 6: Compute damping ratios for uncontrolled system.
    
    A_eig = eig(A);
    
    damping_ratios(ii,1:4) = -real(A_eig)./sqrt(real(A_eig).^2+imag(A_eig).^2);
    
    % Task 7: Compute damping ratios for actively controlled systems.
    
    L = lqr(A,Bdeltat,Q,1);
    
    eigenval = eig(A-Bdeltat*L);
    
    L1 = lqr(A,BDeltaM,Q2,1);
    eigenvalM = eig(A-BDeltaM*L1);
    
    
    
    damping_ratios_deltat(ii,1:4) = -real(eigenval)./sqrt(real(eigenval).^2+imag(eigenval).^2);
   
    
    damping_ratios_DeltaM(ii,1:4) = -real(eigenvalM)./sqrt(real(eigenvalM).^2+imag(eigenvalM).^2);
end

% Plot damping ratios
figure;
plot(vxs,damping_ratios,'.k')
if plot_controlled
    hold on
    plot(vxs,damping_ratios_deltat,'.b')
    plot(vxs,damping_ratios_DeltaM,'.r')
    ls = get(gca,'Children');
    legend(ls(end:-4:1),'without control', 'active trailer steering', 'torque vectoring', 'Location', 'southwest')
    hold off
end
grid on
xlabel('Speed, [m/s]')
ylabel('Damping Ratio')