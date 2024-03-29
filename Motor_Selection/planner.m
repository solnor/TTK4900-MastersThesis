dtx = 0.5;
dty = 0.2;
dr = 0.2;
mp  = 0.25;
Izz = 6.25e-4;
g = 9.81;

A_c = [0  1      0  0      0 0;
       0 -dtx/mp 0  0      0 0;
       0  0      0  1      0 0;
       0  0      0 -dty/mp 0 0;
       0  0      0  0      0 1;
       0  0      0  0      0 -dr/Izz];
B_c = [0    0    0;
       1/mp 0    0;
       0    0    0;
       0    1/mp 0;
       0    0    0;
       0    0    1/Izz;];

d_c = [ 0;
        0;
        0;
       -g;
        0;
        0];

dt = 0.25;
t_max = 5;
A_d = eye(6) + dt*A_c;
B_d = dt*B_c;
d_d = dt*d_c;

nx = 6; % Number of states
nu = 3; % Number of inputs

x0 = [ 0.3;
      -0.0;
       0.3;
       0.0;
       0.1;
       0.0;];

xf = [-0.2;
       0.0;
      -0.2;
       0.0;
       0.1;
       0.0];
%%

N = t_max/dt;
M = N;

z0 = zeros(N*nx + M*nu, 1);
z0(1:nx) = x0;

ul = -inf*ones(nu,1);
uu = inf*ones(nu,1);

xl = [-1;
      -inf;
      -1;
      -inf;
      -inf;
      -inf];
xu = [1;
      inf;
      1;
      inf;
      inf;
      inf;];


[vlb, vub] = gen_constraints(N, M, xl, xu, ul, uu);

t_limit = 0.5;

[vlb,vub] = set_states_at_time_onwards(t_limit, xf(1), 1, vub, vlb, dt, nx, N);
[vlb,vub] = set_states_at_time_onwards(t_limit, xf(2), 2, vub, vlb, dt, nx, N);
[vlb,vub] = set_states_at_time_onwards(t_limit, xf(3), 3, vub, vlb, dt, nx, N);
[vlb,vub] = set_states_at_time_onwards(t_limit, xf(5), 5, vub, vlb, dt, nx, N);


q = diag(1*ones(nx,1));
r = diag([0 0 0]);
Q = 2*gen_q(q,r,N,M);

cT = [repmat(-xf.'*2*q,1,N) zeros(1,nu*N)];

%%

A_eq = gen_aeq(A_d, B_d, N, nx, nu);
% b_eq = [A_d*x0;
%         zeros(nx*(N-1), 1)];
b_eq = [A_d*x0 + d_d;
        repmat(d_d,N-1,1)];
%%
opt = optimoptions('fmincon' ,          ...
                   'Algorithm', 'sqp' , ...
                   'MaxFunEvals', 8000);
z = fmincon(@(z)0.5*z.'*Q*z + cT*z, z0, [], [], A_eq, b_eq, vlb, vub, ...
            @(z)nonlcon(z,N,M,nx,nu), opt);
%%
x_opt = [z(1:nx:N*nx);z(N*nx-5)];
xd_opt = [z(2:nx:N*nx);z(N*nx-4)];
y_opt = [z(3:nx:N*nx);z(N*nx-3)];
yd_opt = [z(4:nx:N*nx);z(N*nx-2)];
theta_opt = [z(5:nx:N*nx);z(N*nx-1)];
thetad_opt = [z(6:nx:N*nx);z(N*nx)];

u_opt = [z(N*nx+1:N*nx + M*nu)];
fx_opt = [u_opt(1:nu:N*nu-2);u_opt(N*nu-2)];
fy_opt = [u_opt(2:nu:N*nu-1);u_opt(N*nu-1)];
t_opt  = [u_opt(3:nu:N*nu);u_opt(N*nu)];




t = 0:dt:(N)*dt;


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
nonlcon(z,N,M,nx,nu);

% ts_w = timeseries([fx_opt fy_opt t_opt],t);
num_variables = 2/dt;
zero_padding = zeros(num_variables,1);
unit_padding = ones(num_variables,1);
% size(zero_padding)
% size(fx_opt)
fx_opt_pad = [zero_padding; t_opt];
% size(fx_opt_pad)
data = [zero_padding zero_padding zero_padding; fx_opt fy_opt t_opt];
t = 0:dt:(size(data,1)-1)*dt;
ts_w = timeseries(data,t);

qd = [x0(2)*unit_padding x0(4)*unit_padding x0(6)*unit_padding; xd_opt yd_opt thetad_opt];
q  = [x0(1)*unit_padding x0(3)*unit_padding x0(5)*unit_padding; x_opt y_opt theta_opt];
ts_qd = timeseries(qd, t);
ts_q = timeseries(q, t);

function [vlb, vub] = set_states_at_time(t, value, state_offset, vlb, vub, dt, nx)
    vlb(t/dt*(nx)+state_offset) = value;
    vub(t/dt*(nx)+state_offset) = value;
end
function [vlb, vub] = set_states_at_time_onwards(t, value, state_offset, vlb, vub, dt, nx, N)
    vlb(t/dt*nx + state_offset:nx:N*nx) = value;
    vub(t/dt*nx + state_offset:nx:N*nx) = value;
end


function [ci, ceq] = nonlcon(z,N,M,nx,nu)
    x_h      = [z(1:nx:N*nx)];
%     xd_h     = [z(2:nx:N*nx)];
    y_h      = [z(3:nx:N*nx)];
%     yd_h     = [z(4:nx:N*nx)];
    theta_h  = [z(5:nx:N*nx)];
%     thetad_h = [z(6:nx:N*nx)];

    u_h     = [z(N*nx+1:N*nx + M*nu)];
    fx_h    = [u_h(1:nu:N*nu-2)];
    fy_h    = [u_h(2:nu:N*nu-1)];
    tau_h   = [u_h(3:nu:N*nu)];
    
    % Constant parameters
    length=0.4; % Horizontal length of platform
    height=0.2; % Vertical length of platform

    a = [-1.0 1.0  1.0 -1.0;
          1.0 1.0 -1.0 -1.0];
    b = [-length/2 length/2  length/2 -length/2;  
          height/2 height/2 -height/2 -height/2];

    ci  = zeros(N,1);
    ceq = zeros(2*N,1);
    for i = 1:N
        % x     = x_h(i);
        % y     = y_h(i);
        % theta = theta_h(i);
        q = [x_h(i); y_h(i); theta_h(i);];
%         qd = [xd_h(i); yd_h(i); thetad_h(i);];
        
        w = [fx_h(i); fy_h(i); tau_h(i)];

        
    
        AT = calculate_sm(q,a,b);
        h = null(AT);
    
        f_0 = -pinv(AT)*w;
        f_max = 150;
        f_min = 20;
        
        ll = zeros(4,1);
        ul = zeros(4,1);
        for j = 1:4
            ll(j) = (f_min - f_0(j))/h(j);
            ul(j) = (f_max - f_0(j))/h(j);
        end
        mn = max(ll);
        mx = min(ul);
        
        ci(i) = mn - mx;
        ceq(i) = [];
    end
    
    
    
end

function AT = calculate_sm(q, a, b)
    r = q(1:2);
    theta = q(3);

    ln = inverseKinematics(a, b, q, 4);
    l = calculate_l_vec(q, a, b);
    u = [l(:,1)/ln(1) l(:,2)/ln(2) l(:,3)/ln(3) l(:,4)/ln(4)];

    R = R_z(theta);
    bi = [R*b(:,1) R*b(:,2) R*b(:,3) R*b(:,4)];
    
    bcrossu = zeros(1,4);
    for i = 1:4
        biskew = skew([bi(:,i); 0]);
        crossProd = biskew*[u(:,i);0];
        bcrossu(i) = crossProd(3);
    end
    
    AT = [u;bcrossu];
end

function uskew = skew(u)
    u1 = u(1);
    u2 = u(2);
    u3 = u(3);
    uskew = [ 0  -u3  u2;
              u3  0  -u1;
             -u2  u1  0];
end

function l_vec = calculate_l_vec(q, a, b)
    r = q(1:2);
    theta = q(3);
    l_vec = zeros(2,4);
    for i = 1:4
        l_vec(:,i) = a(:,i) - r - R_z(theta)*b(:,i);
    end
end

function l = inverseKinematics(a, b, pose, m)
    x     = pose(1);
    y     = pose(2);
    theta = pose(3);

    l = zeros(m,1);
    for i=1:m
        l(i) = norm(a(:,i)-[x;y]-R_z(theta)*b(:,i),2);
    end
end


function R = R_z(theta)
    R = [cos(theta) -sin(theta);
         sin(theta)  cos(theta)];
end















function [vlb,vub] = gen_constraints(N,M,xl,xu,ul,uu)
    % Function to generate constraints on states and inputs.
    % N     - Time horizon for states
    % M     - Time horizon for inputs
    % xl    - Lower bound for states (column vector, mx*1)
    % xu    - Upper bound for states (column vector, mx*1)
    % ul    - Lower bound for inputs (column vector, mu*1)
    % uu    - Upper bound for inputs (column vector, mu*1)
    %
    % Oppdatert 29/3-2001 av Geir Stian Landsverk
    % Updated January 2018 by Andreas L. Flåten (translated to English)
    
    vlb_x	= repmat(xl,N,1);
    vub_x	= repmat(xu,N,1);
    
    vlb_u	= repmat(ul,M,1);
    vub_u	= repmat(uu,M,1);
    
    vlb	    = [vlb_x; vlb_u];
    vub	    = [vub_x; vub_u];
end

function y = diag_repeat(varargin)

    % BLKDIAG2  Block diagonal concatenation of input arguments.
    %
    %                                   |A 0 .. 0|
    %   Y = diag_repeat(A,N)  produces  |0 A .. 0|
    %                                   |0 0 .. A|
    %   where 	size(A) = [m,n]
    %			size(Y) = [N*m,N*n]
    %
    %   See also DIAG, HORZCAT, VERTCAT
    
    % Wes Wang 9/9/94, 9/30/95.  Greg Wolodkin 1/30/98
    % Copyright (c) 1984-98 by The MathWorks, Inc.
    % $Revision: 1.2 $
    
    % Modified version of blkdiag.m
    % Geir Stian Landsverk 3/8/01
    
    
    x = varargin{1};
    [p2,m2] = size(x);
    y = [];
    for k = 1:varargin{2}
        [p1,m1] = size(y); 
        y = [y zeros(p1,m2); zeros(p2,m1) x];
    end
end

function Q = gen_q(Q1,P1,N,M)
    % Function to build a matrix Q that has the following form:
    %      -           -                                            
    %      |Q1         |                                            
    %  Q = |  .        |                                            
    %      |   Q1      |                                            
    %      |     P1    |                                            
    %      |       .   |                                            
    %      |         P1|                                            
    %      -           -                                            
    % where Q1 is repeated N times and P1 is repeated M times.
    %
    % Q1 - Weight on states (mx*mx matrix)                       
    % P1 - Weight on inputs (mu*mu matrix)                          
    % N  - Time horizon for states
    % M  - Time horizon for inputs
    %                                                               
    % 08.03.2001 Geir Stian Landsverk
    % January 2018, Andreas L. Flåten (translated to English)
    q1	= diag_repeat(Q1,N);
    p1	= diag_repeat(P1,M);
    
    Q	= blkdiag(q1,p1); 
end



function A = gen_aeq(A1,B1,N,mx,mu)
% Function to build a matrix representing equality constraints for a 
% linear discrete time dynamical system. The resulting matrix has the form                            
%      -                    -                                   
%      |I        |-B1       |                                   
%  A = |-A1 I    |  -B1     |                                   
%      |   . .   |     .    |                                   
%      |   -A1 I |      -B1 |                                   
%      -                    -                                   
%                                                               
% A1 - Discrete time system matrix                          
% B1 - Discrete time input matrix                       
% N  - Time horizon, assumes M = N
% mx - Number of states                               
% mu - Number of inputs                                     
%                                                               
% 08.03.2001 Geir Stian Landsverk
% January 2018, Andreas L. Flåten (translation to English)
                                                              

A 	= zeros(N*mx,N*mx+N*mu);
b1	= diag_repeat(-B1,N);
a1	= eye(N*mx);
for i=1:(N-1)
  a1([(i*mx+1):((i+1)*mx)],[((i-1)*mx+1):(i*mx)])=-A1;
end
A	= [a1 b1];
end