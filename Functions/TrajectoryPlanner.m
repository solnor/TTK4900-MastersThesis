function [ts_w,ts_qd,ts_q] = TrajectoryPlanner(a,b,q0,qd0,qf,qdf,t_end,t_limit,dt,f_min,f_max,A_c,B_c,d_c)
z0 = [ q0;
       qd0];

zf = [ qf;
       qdf];

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

%% Padding
num_variables = 2/dt; % Two seconds
zero_padding = zeros(num_variables,1);
unit_padding = ones(num_variables,1);

w_opt_pad = [zero_padding zero_padding zero_padding; 
             fx_opt       fy_opt       t_opt];
opt_q_pad  = [z0(1)*unit_padding z0(2)*unit_padding z0(3)*unit_padding; 
              x_opt              y_opt              theta_opt];
opt_qd_pad = [z0(4)*unit_padding z0(5)*unit_padding z0(6)*unit_padding; 
              xd_opt             yd_opt             thetad_opt];

t = 0:dt:(size(w_opt_pad,1)-1)*dt;
ts_w = timeseries(w_opt_pad,t);
ts_qd = timeseries(opt_qd_pad, t);
ts_q = timeseries(opt_q_pad, t);

end