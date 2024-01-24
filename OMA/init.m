% Parameters
m = 0.25;      % Mass of platform
I = 6.25e-4;      % Inertia of platform
length=0.4; % Horizontal length of platform
height=0.2; % Vertical length of platform
E = 100E9;  % Young's modulus of wire
A = 1e-6;   % Cross sectional area of wire

% Anchor points
% a1 = [ 1;  1];
% a2 = [-1;  1];
% a3 = [-1; -1];
% a4 = [ 1; -1];
% a  = [a1; a2; a3; a4];
% 
% % Body anchors
% b1 = [ length/2;  height/2];
% b2 = [-length/2;  height/2];
% b3 = [-length/2; -height/2];
% b4 = [ length/2; -height/2];
a = [-1.0 1.0  1.0 -1.0;
      1.0 1.0 -1.0 -1.0];

a1 = a(:,1);
a2 = a(:,2);
a3 = a(:,3);
a4 = a(:,4);

b = [-length/2 length/2  length/2 -length/2;  
      height/2 height/2 -height/2 -height/2];
b1 = b(:,1);
b2 = b(:,2);
b3 = b(:,3);
b4 = b(:,4);
% Initial platform position
x0   = 0.3;
y0   = 0.3;
psi0 = 0.1;
% q0 = [0.3;0.3;0.0];

% Body anchors in global coordinates
b1g = b_to_g([x0;y0],psi0,b1);
b2g = b_to_g([x0;y0],psi0,b2);
b3g = b_to_g([x0;y0],psi0,b3);
b4g = b_to_g([x0;y0],psi0,b4);
bg = [b1g b2g b3g b4g];

% Initial cable lengths
l_0(:,1) = a(:,1) - bg(:,1);
l1n0 = norm(a(:,1) - bg(:,1));
l2n0 = norm(a(:,2) - bg(:,2));
l3n0 = norm(a(:,3) - bg(:,3));
l4n0 = norm(a(:,4) - bg(:,4));


P_gain_f = 75;
I_gain_f = 10;

%%

vx = 0.1;
vy = 0;
vr = 0;

l1n = abs((2*(a1(1)-x0+b1(1)*cos(psi0)+b1(2)*sin(psi0)) * (-vx-b1(1)*vr*sin(psi0)-b1(2)*vr*cos(psi0)) + 2*(a1(2)-y0-b1(2)*cos(psi0) + b1(1)*sin(psi0))*(-vy-b1(1)*vr*cos(psi0)+b1(2)*vr*sin(psi0) ) )/sqrt(complex((a1(1)-x0-b1(1)*cos(psi0)+b1(2)*sin(psi0))^2 + (a1(2) - y0 - b1(1)*sin(psi0) - b1(2)*cos(psi0))^2)))
l2n = abs((2*(a2(1)-x0+b2(1)*cos(psi0)+b2(2)*sin(psi0)) * (-vx-b2(1)*vr*sin(psi0)-b2(2)*vr*cos(psi0)) + 2*(a2(2)-y0-b2(2)*cos(psi0) + b2(1)*sin(psi0))*(-vy-b2(1)*vr*cos(psi0)+b2(2)*vr*sin(psi0) ) )/sqrt(complex((a2(1)-x0-b2(1)*cos(psi0)+b2(2)*sin(psi0))^2 + (a2(2) - y0 - b2(1)*sin(psi0) - b2(2)*cos(psi0))^2)))
l2n = abs((2*(a3(1)-x0+b3(1)*cos(psi0)+b3(2)*sin(psi0)) * (-vx-b3(1)*vr*sin(psi0)-b3(2)*vr*cos(psi0)) + 2*(a3(2)-y0-b3(2)*cos(psi0) + b3(1)*sin(psi0))*(-vy-b3(1)*vr*cos(psi0)+b3(2)*vr*sin(psi0) ) )/sqrt(complex((a3(1)-x0-b3(1)*cos(psi0)+b3(2)*sin(psi0))^2 + (a3(2) - y0 - b3(1)*sin(psi0) - b3(2)*cos(psi0))^2)))
l3n = abs((2*(a4(1)-x0+b4(1)*cos(psi0)+b4(2)*sin(psi0)) * (-vx-b4(1)*vr*sin(psi0)-b4(2)*vr*cos(psi0)) + 2*(a4(2)-y0-b4(2)*cos(psi0) + b4(1)*sin(psi0))*(-vy-b4(1)*vr*cos(psi0)+b4(2)*vr*sin(psi0) ) )/sqrt(complex((a4(1)-x0-b4(1)*cos(psi0)+b4(2)*sin(psi0))^2 + (a4(2) - y0 - b4(1)*sin(psi0) - b4(2)*cos(psi0))^2)))
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