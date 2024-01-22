length=0.4; % Horizontal length of actuator
height=0.2; % Vertical length of actuator

a = [-1.0 1.0  1.0 -1.0;
      1.0 1.0 -1.0 -1.0];
% a_n(dimension, cable)
% b = [-0.05 0.05  0.05 -0.05;
%       0.05 0.05 -0.05 -0.05];
b = [-length/2 length/2  length/2 -length/2;  
      height/2 height/2 -height/2 -height/2];

x = 0;
y = 0;
theta = 0;
q = [x; y; theta];

r = [x;y];


ln = inverseKinematics(a, b, q, 4);


l1 = a(:,1) - r - R_z(theta)*b(:,1);
l2 = a(:,2) - r - R_z(theta)*b(:,2);
l3 = a(:,3) - r - R_z(theta)*b(:,3);
l4 = a(:,4) - r - R_z(theta)*b(:,4);

u = [l1/ln(1) l2/ln(2) l3/ln(3) l4/ln(4)];

b1i = R_z(theta)*b(:,1);
b2i = R_z(theta)*b(:,2);
b3i = R_z(theta)*b(:,3);
b4i = R_z(theta)*b(:,4);

bi = [b1i b2i b3i b4i];

bcrossu = zeros(1,4);
for i = 1:4
    biskew = skew([bi(:,i); 0]);
    crossProd = biskew*[u(:,i);0];
    bcrossu(i) = crossProd(3);
end

AT = [u;bcrossu];
clc
w = [1;1;1.4];
res = nonlcon(q,w)


% tr = quadprog(eye(4),-5*ones(4,1),[],[],AT, w,1*ones(4,1),10*ones(4,1), []);

function c = nonlcon(q,w)
    x = q(1);
    y = q(2);
    theta = q(3);
    
    % Constant parameters
    length=0.4; % Horizontal length of platform
    height=0.2; % Vertical length of platform

    a = [-1.0 1.0  1.0 -1.0;
          1.0 1.0 -1.0 -1.0];
    b = [-length/2 length/2  length/2 -length/2;  
          height/2 height/2 -height/2 -height/2];

    AT = calculate_sm(q,a,b);
    h = null(AT);

    f_0 = -pinv(AT)*w;
    % f_0 = w*ones(4,1);
    f_max = 10;
    f_min = 1;
    
    ll = zeros(4,1);
    ul = zeros(4,1);
    for i  = 1:4
        ll(i) = (f_min - f_0(i))/h(i);
        ul(i) = (f_max - f_0(i))/h(i);
    end
    mn = max(ll);
    mx = min(ul);
    
    % c = abs(max(ll)) - abs(min(ul));
    c = max(ll) - min(ul);
    % 
    
    % if mn > mx
    %     c = 1;
    %     return
    % end
    % c = -1;
    % if (f_min - f_0(i)) < 0
    %     k = -1;
    %     return
    % end
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