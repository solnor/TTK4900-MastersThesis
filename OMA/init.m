% Parameters
m = 1;      % Mass of platform
I = 1;      % Inertia of platform
length=0.4; % Horizontal length of platform
height=0.2; % Vertical length of platform
E = 100E9;  % Young's modulus of wire
A = 1e-6;   % Cross sectional area of wire

% Anchor points
a1 = [ 1;  1];
a2 = [-1;  1];
a3 = [-1; -1];
a4 = [ 1; -1];
a  = [a1; a2; a3; a4];

% Body anchors
b1 = [ length/2;  height/2];
b2 = [-length/2;  height/2];
b3 = [-length/2; -height/2];
b4 = [ length/2; -height/2];

% Initial platform position
x = 0;
y = 0;
psi = 0;

% Body anchors in global coordinates
b1g = b_to_g([x;y],psi,b1);
b2g = b_to_g([x;y],psi,b2);
b3g = b_to_g([x;y],psi,b3);
b4g = b_to_g([x;y],psi,b4);
bg = [b1g; b2g; b3g; b4g];

% Initial cable lengths
l_0 = a - bg;

vx = 0.1;
vy = 0;
vr = 0;

l1n = abs((2*(a1(1)-x+b1(1)*cos(psi)+b1(2)*sin(psi)) * (-vx-b1(1)*vr*sin(psi)-b1(2)*vr*cos(psi)) + 2*(a1(2)-y-b1(2)*cos(psi) + b1(1)*sin(psi))*(-vy-b1(1)*vr*cos(psi)+b1(2)*vr*sin(psi) ) )/sqrt(complex((a1(1)-x-b1(1)*cos(psi)+b1(2)*sin(psi))^2 + (a1(2) - y - b1(1)*sin(psi) - b1(2)*cos(psi))^2)))
l2n = abs((2*(a2(1)-x+b2(1)*cos(psi)+b2(2)*sin(psi)) * (-vx-b2(1)*vr*sin(psi)-b2(2)*vr*cos(psi)) + 2*(a2(2)-y-b2(2)*cos(psi) + b2(1)*sin(psi))*(-vy-b2(1)*vr*cos(psi)+b2(2)*vr*sin(psi) ) )/sqrt(complex((a2(1)-x-b2(1)*cos(psi)+b2(2)*sin(psi))^2 + (a2(2) - y - b2(1)*sin(psi) - b2(2)*cos(psi))^2)))
l2n = abs((2*(a3(1)-x+b3(1)*cos(psi)+b3(2)*sin(psi)) * (-vx-b3(1)*vr*sin(psi)-b3(2)*vr*cos(psi)) + 2*(a3(2)-y-b3(2)*cos(psi) + b3(1)*sin(psi))*(-vy-b3(1)*vr*cos(psi)+b3(2)*vr*sin(psi) ) )/sqrt(complex((a3(1)-x-b3(1)*cos(psi)+b3(2)*sin(psi))^2 + (a3(2) - y - b3(1)*sin(psi) - b3(2)*cos(psi))^2)))
l3n = abs((2*(a4(1)-x+b4(1)*cos(psi)+b4(2)*sin(psi)) * (-vx-b4(1)*vr*sin(psi)-b4(2)*vr*cos(psi)) + 2*(a4(2)-y-b4(2)*cos(psi) + b4(1)*sin(psi))*(-vy-b4(1)*vr*cos(psi)+b4(2)*vr*sin(psi) ) )/sqrt(complex((a4(1)-x-b4(1)*cos(psi)+b4(2)*sin(psi))^2 + (a4(2) - y - b4(1)*sin(psi) - b4(2)*cos(psi))^2)))
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