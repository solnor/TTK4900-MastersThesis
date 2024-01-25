% clear all; clc;
% Plant parameters
mp  = 0.25;     % Mass of platform
Izz = 6.25e-4;  % Moment of inertia about z-axis
dtx = 0.0;      % Translational dampening coefficient in the x-direction
dty = 0.0;      % Translational dampening coefficient in the y-direction
dr  = 0.0;      % Rotational dampening coefficient about the z-axis
g   = 9.81;     % Gravitational acceleration

length = 0.4; % Horizontal length of platform
height = 0.2; % Vertical length of platform

% Continuous plant model
A_c = [0  0 0  1       0       0;
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
f_min = 10;
f_max = 100;
f_ref = 25;

% Anchor points
a = [-1.0 -1.0  1.0  1.0;
     -1.0  1.0  1.0 -1.0];
length = 0.15; % Horizontal length of platform
height = 0.02; % Vertical length of platform
b = [-length/2 -length/2  length/2  length/2;  
     -height/2  height/2  height/2 -height/2];
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
x0       = -0.2;
y0       = -0.0;
theta0   = 0.0;
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
    ln0(i) = norm(a(:,i) - bg(:,i));
end


% Cable force controller parameters
P_gain_f = 75;
I_gain_f = 10;


K_d = B_c\eye(6);
K_f = [180*diag([1,1,1]) 10*diag([1,1,1])];
K_r = pinv((B_c*K_f-A_c)\B_c);
K_a = [diag([-10,-10,-1]) zeros(3,3)];
%% 
z0 = [ q0;
       0.0;
       0.0;
       0.0;];

zf = [ 0.2;
       0.0;
       -0.2;
       0.0;
       0.0;
       0.0];

%% Trajectory planner

t_end = 5;
t_limit = 0.5;
dt = 0.25;

[z_opt, nx, nu, N_opt] = calculate_optimal_trajectory(A_c, B_c, d_c, ...
                                                      z0, zf, ...
                                                      dt, ...
                                                      t_limit, t_end, ...
                                                      a, b, ...
                                                      f_min, f_max);
M_opt = N_opt;

x_opt     = [z_opt(1:nx:N_opt*nx);z_opt(N_opt*nx-5)];
y_opt     = [z_opt(2:nx:N_opt*nx);z_opt(N_opt*nx-4)];
theta_opt = [z_opt(3:nx:N_opt*nx);z_opt(N_opt*nx-3)];

xd_opt = [z_opt(4:nx:N_opt*nx);z_opt(N_opt*nx-2)];
yd_opt = [z_opt(5:nx:N_opt*nx);z_opt(N_opt*nx-1)];
thetad_opt = [z_opt(6:nx:N_opt*nx);z_opt(N_opt*nx)];

u_opt = [z_opt(N_opt*nx+1:N_opt*nx + M_opt*nu)];
fx_opt = [u_opt(1:nu:N_opt*nu-2);u_opt(N_opt*nu-2)];
fy_opt = [u_opt(2:nu:N_opt*nu-1);u_opt(N_opt*nu-1)];
t_opt  = [u_opt(3:nu:N_opt*nu);u_opt(N_opt*nu)];
%% Plot traj planner output
t = 0:dt:(N_opt)*dt;

subplot(4,1,1);
plot(t,x_opt)
legend('x')

subplot(4,1,2);
plot(t,y_opt);
legend('y')
subplot(4,1,3);
plot(t,theta_opt);
legend('theta')
subplot(4,1,4);
plot(t,fx_opt); hold on;
plot(t,fy_opt);
plot(t,t_opt); hold off;
legend('$f_x$', '$f_y$', '$\tau$', 'interpreter', 'latex')
%%
num_variables = 2/dt; % Two seconds
zero_padding = zeros(num_variables,1);
unit_padding = ones(num_variables,1);

w_opt_pad = [zero_padding zero_padding zero_padding; 
             fx_opt       fy_opt       t_opt];
opt_q_pad  = [z0(1)*unit_padding z0(3)*unit_padding z0(5)*unit_padding; 
              x_opt              y_opt              theta_opt];
opt_qd_pad = [z0(2)*unit_padding z0(4)*unit_padding z0(6)*unit_padding; 
              xd_opt             yd_opt             thetad_opt];

t = 0:dt:(size(w_opt_pad,1)-1)*dt;
ts_w = timeseries(w_opt_pad,t);
ts_qd = timeseries(opt_qd_pad, t);
ts_q = timeseries(opt_q_pad, t);

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