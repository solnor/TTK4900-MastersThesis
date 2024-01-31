function [z, nx, nu, N] = calculate_optimal_trajectory(A_c, B_c, d_c, ...
                                                  x0, xf, ...
                                                  ctrl_vel, ...
                                                  u0, ...
                                                  dt, ...
                                                  t_limit, t_end, ...
                                                  a, b, ...
                                                  f_min, f_max)
    A_d = eye(6) + dt*A_c;
    B_d = dt*B_c;
    d_d = dt*d_c;

    nx = size(A_d,2); % Number of states
    nu = size(B_d,2); % Number of inputs

    N = t_end/dt;  % Horizon
    M = N;

    z0 = zeros(N*nx + M*nu, 1);  % Initial trajectory
    z0(1:nx) = x0;               % Set states for the first 
                                 % time step to initial conditions
    % z0(nx+1:nx+nu) = u0;

    ul = -inf*ones(nu,1);        % Lower limit on inputs
    uu =  inf*ones(nu,1);        % Upper limit on inputs
    
    % Lower limit on states (ASSUMING a1 LOWER LEFT)
    xl = [ a(1,1); % x
           a(2,1); % y
          -pi/2;    % theta
          -inf;    % x_dot
          -inf;    % y_dot
          -inf];   % theta_dot
    % Upper limit on states (ASSUMING a3 UPPER RIGHT)
    xu = [a(1,3); % x
          a(2,3); % y
          pi/2;    % theta
          inf;    % x_dot
          inf;    % y_dot
          inf;];  % theta_dot

    [vlb, vub] = gen_constraints(N, M, xl, xu, ul, uu);
    
    % Set state vector at time zero to initial conditions
%     [vlb,vub] = set_state_vector_at_time(0, x0, vlb, vub, dt, nx);
    % [vlb,vub] = set_state_at_time_onwards(0, -0.4, 0, vlb, vub, dt, nx, N); % x
    % Set lower- and upper bounds on states to the final value specified
    [vlb,vub] = set_state_at_time_onwards(t_limit, xf(1), 0, vlb, vub, dt, nx, N); % x
    [vlb,vub] = set_state_at_time_onwards(t_limit, xf(2), 1, vlb, vub, dt, nx, N); % y
    [vlb,vub] = set_state_at_time_onwards(t_limit, xf(3), 2, vlb, vub, dt, nx, N); % theta
    if ctrl_vel
        [vlb,vub] = set_state_at_time_onwards(t_limit, xf(4), 3, vlb, vub, dt, nx, N); % x
        [vlb,vub] = set_state_at_time_onwards(t_limit, xf(5), 4, vlb, vub, dt, nx, N); % y
        [vlb,vub] = set_state_at_time_onwards(t_limit, xf(6), 5, vlb, vub, dt, nx, N); % theta
    end
    
%     [vlb,vub] = set_state_at_time_onwards(t_limit, xf(5), 5, vlb, vub, dt, nx, N);

    q = diag(10*ones(nx,1));
    r = diag([5 5 5]);
    Q = 2*gen_q(q,r,N,M); % Hessian
    
    s = diag([1 1 1]);
    S = gen_S(s, nx, nu, N);

    cT = [repmat(-xf.'*2*q,1,N) zeros(1,nu*N)]; % c^T

    % System of equalities
    A_eq = gen_aeq(A_d, B_d, N, nx, nu);

    b_eq = [A_d*x0 + d_d;
            repmat(d_d,N-1,1)];
    
    
    % Optimisation problem
    opt = optimoptions('fmincon' ,          ...
                   'Algorithm', 'sqp' , ...
                   'MaxFunEvals', 8000);
    z = fmincon(@(z)(0.5*z.'*Q*z + cT*z), ...
                z0, ...
                [], [], ...
                A_eq, b_eq, ...
                vlb, vub, ...
                @(z)nonlcon(z, N, M, nx, nu, f_min, f_max, a, b), ...
                opt);
end

function [vlb, vub] = set_state_vector_at_time(t, state_vec, vlb, vub, dt, nx)
    vlb(t/dt*(nx)+1:t/dt*(nx)+nx) = state_vec;
    vub(t/dt*(nx)+1:t/dt*(nx)+nx) = state_vec;
end

function [vlb, vub] = set_state_at_time(t, value, state_offset, vlb, vub, dt, nx)
    vlb(t/dt*(nx)+1+state_offset) = value;
    vub(t/dt*(nx)+1+state_offset) = value;
end

function [vlb, vub] = set_state_at_time_onwards(t, value, state_offset, vlb, vub, dt, nx, N)
    % t
    % t/dt*nx + 1 + state_offset
    % value
    
    if t == 0
        vlb(1 + state_offset) = value;
        vub(1 + state_offset) = value;
    else
        vlb((t/dt-1)*nx + 1 + state_offset:nx:(N)*nx) = value;
        vub((t/dt-1)*nx + 1 + state_offset:nx:(N)*nx) = value;
    end
end


function [ci, ceq] = nonlcon(z,N,M,nx,nu, f_min, f_max, a, b)
    x_h      = [z(1:nx:N*nx)];
%     xd_h     = [z(4:nx:N*nx)];
    y_h      = [z(2:nx:N*nx)];
%     yd_h     = [z(5:nx:N*nx)];
    theta_h  = [z(3:nx:N*nx)];
%     thetad_h = [z(6:nx:N*nx)];

    u_h     = [z(N*nx+1:N*nx + M*nu)];
    fx_h    = [u_h(1:nu:N*nu-2)];
    fy_h    = [u_h(2:nu:N*nu-1)];
    tau_h   = [u_h(3:nu:N*nu)];

    ci  = zeros(N,1);
    ceq = zeros(2*N,1);
    for i = 1:N
        q = [x_h(i); y_h(i); theta_h(i);]; % Generalized coordinates
        w = [fx_h(i); fy_h(i); tau_h(i)];  % Inputs
    
        AT = calculate_sm(q,a,b);
        h = null(AT);
    
        f_0 = -pinv(AT)*w;
        
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

    ln = inverse_kinematics(a, b, q, 4);
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

function l = inverse_kinematics(a, b, pose, m)
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