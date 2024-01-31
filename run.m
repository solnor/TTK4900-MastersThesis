close all; clear; clc;
%% Folder magic
% Get the current script's directory
currentFolder           = fileparts(which("run.m"));

% Add the functions folder to the MATLAB path
addpath(fullfile(currentFolder));

%% Geometric Information
% Size of MP
MP_len_x                = 0.15; % [m]
MP_len_y                = 0.02; % [m]
% Size of frame     
F_len_x                 = 1.4;  % [m]
F_len_y                 = 0.8;  % [m]

[a,b,m_MP,Iz_MP] = initRobot(F_len_x,F_len_y,MP_len_x,MP_len_y);
[A_c,A_c_aug,B_c,B_c_aug,d_c,d_c_aug,w,zeroI] = initModel(m_MP,Iz_MP);


% General parameters
E = 100E9;  % Young's modulus of cable
A = 1e-6;   % Cross sectional area of cable
f_min = 10;
f_max = 100;
f_ref = 25;


%% Initial conditions
x0          = -0.3;
y0          = -0.3;
theta0      = deg2rad(-20);
xd0         = 0.0;
yd0         = 0.0;
thetad0     = 0.0;
r0          = [x0; y0];
q0          = [r0; theta0];
qd0         = [xd0;yd0; thetad0];
[~,ln0,~]   = InverseKinematics(a, b, q0); 

%% Final conditions
xf           = 0.3;
yf           = -0.3;
thetaf       = deg2rad(20);
xdf           = 0.0;
ydf          = 0.0;
thetadf      = 0.0;
rf           = [xf; yf];
qf          = [rf; thetaf];
qdf         = [xdf;ydf; thetadf];

%% Workspace check
OrientationWorkspace(r0, a, b, f_min, f_max, 2, 9.81*w)
% TranslationWorkspace(theta0,a,b,f_min,f_max,w,200,'r');


%% Cable force controller parameters
P_gain_f = 75;
I_gain_f = 10;

K_d = B_c\eye(6);
K_f = [180*diag([1,1,1]) 10*diag([1,1,1])];
K_r = pinv((B_c*K_f-A_c)\B_c);
K_a = [diag([-10,-10,-1]) zeros(3,3)];

%% Trajectory planner
t_end = 2;
t_limit = .5;
dt = 0.1;

[ts_w,ts_qd,ts_q] = TrajectoryPlanner(a,b,q0,qd0,qf,qdf,t_end,t_limit,dt,f_min,f_max,A_c,B_c,d_c);
%% Plot optimal path
TrajectoryPlotter(ts_w,ts_qd,ts_q);
%% Simulink
out = sim("Simulation\CDPR_simulation");
%% Plot trajectory
plotit