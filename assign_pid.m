

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Digital Control of Automotive Powertrains - Spring 2017
%
% Model of I.C. engine dynamics for idle speed control.
% 
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all
% clear
clc

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Load engine geometric parameters and constant inputs

Vd = 2.4e-3;   % Displacement (m^3)
Z = 4;         % Number of Cylinders
Vm = 5.8e-3;   % Intake Manifold Volume (m^3)
J = 0.0789;    % Mass moment of inertia

p_amb = 1.0121*1e5;
T_amb = 302;
R=288;
gam = 1.35;

P0 = 26431;   % Initial MAP
N0 = 828;     % Initial RPM

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Model parameters (from steady-state calibration)

a = [1.69e-07,-1.136e-06,6.89e-06];  % Throttle
si = 0.812;   yi = 0.0633;           % Volumetric efficiency
P_IMEP = [0.0220,-1.1649];           % IMEP
par_sp = [-0.0017 -0.0277 1.36];     % Spark timing effect
par_fr = [7.4198e-7 -4.989e-4 11.3]; % Friction
par_IMEP0 = [1.2323e-4 2.1256];      % Base IMEP model

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Conversion to Crank Angle Domain

% Coefficients for crank-angle based model
Kthr = p_amb/sqrt(R*T_amb)*sqrt(gam)*sqrt((2/(gam+1))^((gam+1)/(gam-1)));
Kp1 = R*T_amb/Vm;
Kp2 = si*Vd/(4*pi*Vm);
Kp3 = yi*Vd/(4*pi*Vm);
KT = 1e5*Vd/(4*pi);
Kfr1 = (30/pi)^2 * par_fr(1);
Kfr2 = (30/pi) * par_fr(2);
Kfr3 = par_fr(3);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Calculate Equilibrium Condition (p_0 T_0 w_0)

setp(1) = 9.81; % Throttle Position
setp(2) = -25;  % Spark Timing
setp(3) = 10;   % Load Torque
X0 = [26424 21.3765773202354 83.9019428270409]; % Equilibrium Conditions

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Linearization

% Coefficients for linearized model as shown in lecture
K1 = Kp1*Kthr*(2*a(1)*setp(1)+a(2));
K2 = Kp1*Kthr*(a(1)*setp(1)^2 + a(2)*setp(1) + a(3));
K3 = Kp2;
Kp = KT*par_IMEP0(1)*(par_sp(1)*setp(2)^2 + par_sp(2)*setp(2) + par_sp(3));    % Pressure multiplier
Kt = KT*(par_IMEP0(1)*X0(1) - par_IMEP0(2)) * (par_sp(1)*setp(2) + par_sp(2)); % Spark Timing multiplier
Kf = 2*Kfr1*X0(3)^2 + Kfr2*X0(3);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Constants Calculation
Tau_d = pi/3;
Ndel = 3; 

CT1 = 1+ ((pi*K3)/6);
CT2 = ((pi*K3)/6)-1;
CT3 = (pi*K2)/(3*2*X0(3)^2);
CT4 = (pi*K1)/(2*X0(3)*3);
CT5 = ((pi*Kf)/(2*3*J*X0(3)^2))+1;
CT6 = ((pi*Kf)/(2*J*(X0(3)^2)*3))-1;
CT7 = pi/(3*2*J*X0(3));
CT8 = pi/(3*2*J*X0(3));

% Initialize PID Gains
%KP = 0.2590;
KP = 0.5;
%TI = 1; 
%KI = 0.0112;
KI = 0.04;
%Td = 1;  
%Kd = 4.8771;
Kd = 5;

%simOut = sim('idle_speed_model_project_pid');

% Colors
colors = [0 0 1; 0 1 0; 0 1 1; 1 0 0; 1 0 1; 1 1 0; 
    1 0.5 0.5; 0.5 0 1; 0.5 1 0; 0.5 1 1; 0 0 0];
% Set number of particles
d = 10;
% Define inertial constant
%w = 0.4;
w = 0.2;
% Define Cognitive component
%c1 = 0.6;
c1 = 0.3;
%define Social component
%c2 = 0.3;
c2 = 0.15;
% Define Tolerance
tol = 1e-2;
% Create particle vectors
Ks = rand(3,d);
for x = 1:d
    Ks(:,x) = [KP;KI;Kd];
end
Ks = Ks + (rand(3,d)-rand(3,d))*0.01;
Ks(:,1) = [KP;KI;Kd];
scatter(Ks(1,:), Ks(2,:),20,colors(1,:))
% Initialize particle best vector
P = Ks;
% Initialize cost vector
costs = rand(1,d);
% Initialize Velocities
v = rand(3,d) - rand(3,d);
% Perform Initial cost calculations
for x = 1:d
        KP = Ks(1,x);
        KI = Ks(2,x);
        Kd = Ks(3,x);
        % Run Initial Simulation
        simOut = sim('idle_speed_model_project_pid');
        % store cost in G array
        costs(x) = sum((Speed.Data-800).^2);
end
particle_bests = costs;
[global_best,i] = min(costs);
G = Ks(:,i);
COSTS = [global_best];
t=0;
cindex = 2;
while t < 10
    %randomize r ans s vectors
    r = rand(3,d);
    s = rand(3,d);
    G_ = rand(3,d);
    for x = 1:d
        G_(:,x) = G;
    end
    % update velocities
    v = (w*v) + ((c1*r).*(P - Ks)) + ((c2*s).*(G_ - Ks));
    % update gains
    Ks = Ks + v;
    hold on
    scatter(Ks(1,:), Ks(2,:),20,colors(cindex,:))
    cindex = cindex + 1;
    % initialize new cost array
    costs_new = rand(1,d);
    % find new costs
    for x = 1:d
        KP = Ks(1,x);
        KI = Ks(2,x);
        Kd = Ks(3,x);
        % Run Initial Simulation
        simOut = sim('idle_speed_model_project_pid');
        % store cost in G array
        costs_new(x) = sum((Speed.Data-800).^2);
    end
    
    for x = 1:d
        if costs_new(x) < particle_bests(x)
            particle_bests(x) = costs_new(x);
            P(:,x) = Ks(:,x);
        end
    end
    [m,i] = min(costs_new);
    if m < global_best
        global_best = m;
        G_new  = Ks(:,i);
    else
        G_new = G;
    end
    %diff = abs(m - G);
    G = G_new;
    t = t+1;
    COSTS = [COSTS global_best];
end
%plot(COSTS)

KP = G(1);
KI = G(2);
Kd = G(3);
simOut = sim('idle_speed_model_project_pid');
%}