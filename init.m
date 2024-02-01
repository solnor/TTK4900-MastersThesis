clear all; clc;
% Plant parameters
mp  = 0.25;     % Mass of platform
Izz = 6.25e-4;  % Moment of inertia about z-axis
dtx = 0.0;      % Translational dampening coefficient in the x-direction
dty = 0.0;      % Translational dampening coefficient in the y-direction
dr  = 0.0;      % Rotational dampening coefficient about the z-axis
g   = 9.81;     % Gravitational acceleration

length = 1.4; % Horizontal length of frame
height = 0.8; % Vertical length of frame



% Continuous plant model
A_c = [0  0 0  1       0       0;        % 
       0  0 0  0       1       0;
       0  0 0  0       0       1;
       0  0 0 -dtx/mp  0       0;
       0  0 0  0      -dty/mp  0;
       0  0 0  0       0      -dr/Izz];
A_c_aug = [A_c    zeros(6,6);
           -eye(6) zeros(6,6)];

B_c = [0    0    0;
       0    0    0;
       0    0    0;
       1/mp 0    0;
       0    1/mp 0;
       0    0    1/Izz;];

B_c_aug = [B_c; zeros(6,3)];

d_c = [ 0;
        0;
        0;
        0;
       -g;
        0];
d_c_aug = [d_c; zeros(6,1)];

w = [ 0;
     -1;
      0];

zeroI = [zeros(6,6);eye(6)];

% General parameters
E = 100E9;  % Young's modulus of cable
A = 1e-6;   % Cross sectional area of cable
f_min = 5;
f_max = 50;
f_ref = 10;

% Anchor points
% a = [-1.0 -1.0  1.0  1.0;
%      -1.0  1.0  1.0 -1.0];

a = [-length/2 -length/2 length/2 length/2
     -height/2  height/2 height/2 -height/2];
% length = 0.15; % Horizontal length of platform
% height = 0.02; % Vertical length of platform
l_p = 0.15; % Horizontal length of platform
h_p = 0.02; % Vertical length of platform
% b = [-l_p/2 -l_p/2  l_p/2  l_p/2;  
%      -h_p/2  h_p/2  h_p/2 -h_p/2];
b = [0      -l_p/2  l_p/2  0;  
     -h_p/2  h_p/2  h_p/2 -h_p/2]; % Triangular
b = [-l_p/6 -l_p/2  l_p/2  l_p/6;  
     -h_p/2  h_p/2  h_p/2 -h_p/2]; % Trapezoidal
% a = [-1.0 1.0  1.0 -1.0;
%       1.0 1.0 -1.0 -1.0];
a1 = a(:,1);
a2 = a(:,2);
a3 = a(:,3);
a4 = a(:,4);

% b = [-length/2 length/2  length/2 -length/2;  
%       height/2 height/2 -height/2 -height/2];
b1 = b(:,1);
b2 = b(:,2);
b3 = b(:,3);
b4 = b(:,4);


% Initial platform position
x0       = -0.3;
y0       = -0.2;
theta0   = -0.0;
xd0      = 0.0;
yd0      = 0.0;
thetad0  = 0.0;
r0 = [x0; y0];
q0 = [r0; theta0];
qd0 = [xd0;yd0; thetad0];


% Body anchors in global coordinates
bg = zeros(size(b));
for i = 1:size(b,2)
    bg(:,i) = b_to_g(r0, theta0, b(:,i));
end

% Initial cable lengths
% l_0(:,1) = a(:,1) - bg(:,1);
ln0 = zeros(size(b,2),1);
for i = 1:size(ln0,1)
    % ln0(i) = norm(a(:,i) - bg(:,i));
    ln0(i) = A*E*norm(a(:,i) - bg(:,i))/(f_ref+A*E);
end
t = zeros(4,1);
for i=1:4
    t(i) = A*E*(norm(a(:,i) - bg(:,i)) - ln0(i))/ln0(i);
end

% Cable force controller parameters
P_gain_f = 60;
I_gain_f = 10;


K_d = B_c\eye(6);
K_f = [180*diag([1,1,1]) 10*diag([1,1,1])];
K_r = pinv((B_c*K_f-A_c)\B_c);
K_a = [diag([-10,-10,-1]) zeros(3,3)];

%% Motor parameters
K_v = 150;     % [rad/(s/V)] - Velocity constant
K_e = 1/K_v;   % [V/(rad/s)] - Back-EMF constant
n_p_pairs = 7; % [-]         - Number of pole pairs
l_m = 8.97e-6; % [H]         - Phase self-inductance
r_m = 39e-3;   % [Ohm]       - Phase resistance

i_m_max =  21; % [A]         - Max quadrature current 
i_m_min = -21; % [A]         - Min quadrature current 

J_m = 242e-6;  % [kgm^2]     - Motor inertia

r_w = 30e-3;   % [m]         - Radius of the winch the 
               %               cable is coiled around

% 
thetar0 = ln0/r_w; % [rad]   - Initial position of rotor

% Fourier coefficients for trapezoidal back-EMF unit curves
fourier_coeff_a = [ 0.08333    0.00000     0.00000;
                    0.00000    1.05296    -1.05296;
                    0.07599    0.00000     0.00000;
                    0.00000    0.00000     0.00000;
                    0.05699    0.00000     0.00000;
                    0.00000   -0.04211     0.04211;
                    0.03377    0.00000     0.00000;
                    0.00000   -0.02148     0.02148;
                    0.01424    0.00000     0.00000;
                    0.00000    0.00000     0.00000;
                    0.00303    0.00000     0.00000;
                   -0.00067    0.00870    -0.00870;
                    0.00000    0.00000     0.00000;
                   -0.00048    0.00623    -0.00623;
                    0.00155    0.00000     0.00000;
                   -0.00270    0.00000     0.00000;
                    0.00356    0.00000     0.00000;
                   -0.00392   -0.00364     0.00364;
                    0.00375    0.00000     0.00000];

fourier_coeff_b = [-1.23020    0.60792     0.60792;
                    0.00000    0.00000     0.00000;
                   -0.30874   -0.27018    -0.27018;
                    0.04667    0.00000     0.00000;
                   -0.10013    0.02431     0.02431;
                    0.05305    0.00000     0.00000;
                   -0.02686   -0.01240    -0.01240;
                    0.04801    0.00000     0.00000;
                   -0.01285    0.03002     0.03002;
                    0.03709    0.00000     0.00000;
                   -0.02140   -0.00502    -0.00502;
                    0.02652    0.00000     0.00000;
                   -0.02988    0.00359     0.00359;
                    0.02005    0.00000     0.00000;
                   -0.02932   -0.01080    -0.01080;
                    0.01783    0.00000     0.00000;
                   -0.02187    0.00210     0.00210;
                    0.01768    0.00000     0.00000;
                   -0.01422   -0.00168    -0.00168];


% Clarke Transformation
Clarke = 2/3*[1 -1/2       -1/2;
              0 sqrt(3)/2  -sqrt(3)/2];

%% Initial velocity in collision
mb = 25e-3;
s = -0.5; % Ball is dropped from a height of 0.5 [m]
vb2 = 3.796; % Want the ball to have a velocity of 3.796 [m/s] after collision
phi = 1.01586; % Angle of ball after collision
psi = phi - pi/2;


%% States
% z0 -> zs -> zf
z0 = [ q0;
       0.0;
       0.0;
       0.0;]; % Initial states

zs = [-0.5;
      -0.2; 
      -0.37;
       2;
       3.23;
       0.0]; % Starting states

zf = [ 0.5;
      -0.2;
       0.37;
       0.0;
       0.0;
       0.0]; % Final states

u0 = [0.0; 0.0; 0.0];

%% 
t_end = 1;
t_limit = t_end;
dt = 0.1;

[z_int, nx, nu, N_int] = calculate_optimal_trajectory(A_c, B_c, d_c, ...
                                                      z0, zs, ...
                                                      1, ...
                                                      0, ...
                                                      u0, ...
                                                      dt, ...
                                                      t_limit, t_end, ...
                                                      a, b, ...
                                                      f_min, f_max);
M_int = N_int;

% Coordinates
x_int     = [z0(1); z_int(1:nx:N_int*nx)];
y_int     = [z0(2); z_int(2:nx:N_int*nx)];
theta_int = [z0(3); z_int(3:nx:N_int*nx)];

% Velocities
xd_int     = [z0(4); z_int(4:nx:N_int*nx)];
yd_int     = [z0(5); z_int(5:nx:N_int*nx)];
thetad_int = [z0(6); z_int(6:nx:N_int*nx)];

% Input wrench
u_int    = [z_int(N_int*nx+1:N_int*nx + M_int*nu)];
fx_int   = [u_int(1:nu:N_int*nu-2); u_int(N_int*nu-2)];
fy_int   = [u_int(2:nu:N_int*nu-1); u_int(N_int*nu-1)];
tau_int  = [u_int(3:nu:N_int*nu-0); u_int(N_int*nu-0)];

u0 = u_int(1:nu);

t_int = 0:dt:(N_int)*dt;
%% Trajectory planner

t_end = 1;
t_limit = 0.5;
dt = 0.05;

[z_opt, nx, nu, N_opt] = calculate_optimal_trajectory(A_c, B_c, d_c, ...
                                                      zs, zf, ...
                                                      0, ...
                                                      1, ...
                                                      u0, ...
                                                      dt, ...
                                                      t_limit, t_end, ...
                                                      a, b, ...
                                                      f_min, f_max);
M_opt = N_opt;

% Coordinates
x_opt     = [zs(1); z_opt(1:nx:N_opt*nx)];
y_opt     = [zs(2); z_opt(2:nx:N_opt*nx)];
theta_opt = [zs(3); z_opt(3:nx:N_opt*nx)];

% Velocities
xd_opt     = [zs(4); z_opt(4:nx:N_opt*nx)];
yd_opt     = [zs(5); z_opt(5:nx:N_opt*nx)];
thetad_opt = [zs(6); z_opt(6:nx:N_opt*nx)];

% Input wrench
u_opt    = [z_opt(N_opt*nx+1:N_opt*nx + M_opt*nu)];
fx_opt   = [u_opt(1:nu:N_opt*nu-2); u_opt(N_opt*nu-2)];
fy_opt   = [u_opt(2:nu:N_opt*nu-1); u_opt(N_opt*nu-1)];
tau_opt  = [u_opt(3:nu:N_opt*nu-0); u_opt(N_opt*nu-0)];

t_opt = 0:dt:(N_opt)*dt;
%% Combining trajectories
x_tot     = [x_int; x_opt(2:end)];
y_tot     = [y_int; y_opt(2:end)];
theta_tot = [theta_int; theta_opt(2:end)];

xd_tot = [xd_int; xd_opt(2:end)];
yd_tot = [yd_int; yd_opt(2:end)];
thetad_tot = [thetad_int; thetad_opt(2:end)];

fx_tot = [fx_int; fx_opt(2:end)];
fy_tot = [fy_int; fy_opt(2:end)];
tau_tot = [tau_int; tau_opt(2:end)];

t_tot = [t_int t_opt(2:end) + max(t_int)];
%% Plotting
figure(1)
subplot(4,1,1);
plot(t_tot, x_tot);
legend('x')
subplot(4,1,2);
plot(t_tot, y_tot);
legend('y')
subplot(4,1,3);
plot(t_tot, theta_tot);
legend('theta')
subplot(4,1,4);
plot(t_tot, fx_tot); hold on;
plot(t_tot, fy_tot);
plot(t_tot, tau_tot); hold off;
legend('$f_x$', '$f_y$', '$\tau$', 'interpreter', 'latex')
figure(2)
plot(x_tot, y_tot);
%% Plot traj planner output
% t_opt = 0:dt:(N_opt)*dt;
% 
% subplot(4,1,1);
% plot(t_opt,x_opt)
% legend('x')
% 
% subplot(4,1,2);
% plot(t_opt,y_opt);
% legend('y')
% subplot(4,1,3);
% plot(t_opt,theta_opt);
% legend('theta')
% subplot(4,1,4);
% plot(t_opt,fx_opt); hold on;
% plot(t_opt,fy_opt);
% plot(t_opt,tau_opt); hold off;
% legend('$f_x$', '$f_y$', '$\tau$', 'interpreter', 'latex')
%%
num_variables = 2/dt; % Two seconds
zero_padding = zeros(num_variables,1);
unit_padding = ones(num_variables,1);

w_opt_pad = [zero_padding zero_padding zero_padding; 
             fx_opt       fy_opt       tau_opt];
opt_q_pad  = [z0(1)*unit_padding z0(2)*unit_padding z0(3)*unit_padding; 
              x_opt              y_opt              theta_opt];
opt_qd_pad = [z0(4)*unit_padding z0(5)*unit_padding z0(6)*unit_padding; 
              xd_opt             yd_opt             thetad_opt];

t_opt = 0:dt:(size(w_opt_pad,1)-1)*dt;
ts_w = timeseries(w_opt_pad,t_opt);
ts_qd = timeseries(opt_qd_pad, t_opt);
ts_q = timeseries(opt_q_pad, t_opt);

%%
% vx = 0.1;
% vy = 0;
% vr = 0;
% 
% l1n = abs((2*(a1(1)-x0+b1(1)*cos(theta0)+b1(2)*sin(theta0)) * (-vx-b1(1)*vr*sin(theta0)-b1(2)*vr*cos(theta0)) + 2*(a1(2)-y0-b1(2)*cos(theta0) + b1(1)*sin(theta0))*(-vy-b1(1)*vr*cos(theta0)+b1(2)*vr*sin(theta0) ) )/sqrt(complex((a1(1)-x0-b1(1)*cos(theta0)+b1(2)*sin(theta0))^2 + (a1(2) - y0 - b1(1)*sin(theta0) - b1(2)*cos(theta0))^2)))
% l2n = abs((2*(a2(1)-x0+b2(1)*cos(theta0)+b2(2)*sin(theta0)) * (-vx-b2(1)*vr*sin(theta0)-b2(2)*vr*cos(theta0)) + 2*(a2(2)-y0-b2(2)*cos(theta0) + b2(1)*sin(theta0))*(-vy-b2(1)*vr*cos(theta0)+b2(2)*vr*sin(theta0) ) )/sqrt(complex((a2(1)-x0-b2(1)*cos(theta0)+b2(2)*sin(theta0))^2 + (a2(2) - y0 - b2(1)*sin(theta0) - b2(2)*cos(theta0))^2)))
% l2n = abs((2*(a3(1)-x0+b3(1)*cos(theta0)+b3(2)*sin(theta0)) * (-vx-b3(1)*vr*sin(theta0)-b3(2)*vr*cos(theta0)) + 2*(a3(2)-y0-b3(2)*cos(theta0) + b3(1)*sin(theta0))*(-vy-b3(1)*vr*cos(theta0)+b3(2)*vr*sin(theta0) ) )/sqrt(complex((a3(1)-x0-b3(1)*cos(theta0)+b3(2)*sin(theta0))^2 + (a3(2) - y0 - b3(1)*sin(theta0) - b3(2)*cos(theta0))^2)))
% l3n = abs((2*(a4(1)-x0+b4(1)*cos(theta0)+b4(2)*sin(theta0)) * (-vx-b4(1)*vr*sin(theta0)-b4(2)*vr*cos(theta0)) + 2*(a4(2)-y0-b4(2)*cos(theta0) + b4(1)*sin(theta0))*(-vy-b4(1)*vr*cos(theta0)+b4(2)*vr*sin(theta0) ) )/sqrt(complex((a4(1)-x0-b4(1)*cos(theta0)+b4(2)*sin(theta0))^2 + (a4(2) - y0 - b4(1)*sin(theta0) - b4(2)*cos(theta0))^2)))
function [p_g] = b_to_g(p,psi,b)
    % p: origin of body frame in global coordinates
    % psi: rotation of body frame
    % b: point in body frame.
    % p_g: Returns position of point b in global frame.

    R = R_psi(psi); %= [cos(psi), -sin(psi);sin(psi),cos(psi)];
    p_g = p + R*b;
end
function R = R_psi(psi)
    R = [cos(psi), -sin(psi);sin(psi),cos(psi)];
end