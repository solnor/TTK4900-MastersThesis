function [A_c,A_c_aug,B_c,B_c_aug,d_c,d_c_aug,w,zeroI] = initModel(m_MP,Iz_MP)

dtx = 0.0;      % Translational dampening coefficient in the x-direction
dty = 0.0;      % Translational dampening coefficient in the y-direction
dr  = 0.0;      % Rotational dampening coefficient about the z-axis
g   = 9.81;     % Gravitational acceleration

% Continuous plant model
A_c = [0  0 0  1       0       0;        % 
       0  0 0  0       1       0;
       0  0 0  0       0       1;
       0  0 0 -dtx/m_MP  0       0;
       0  0 0  0      -dty/m_MP  0;
       0  0 0  0       0      -dr/Iz_MP];
A_c_aug = [A_c    zeros(6,6);
           -eye(6) zeros(6,6)];

B_c = [0    0    0;
       0    0    0;
       0    0    0;
       1/m_MP 0    0;
       0    1/m_MP 0;
       0    0    1/Iz_MP;];

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
end