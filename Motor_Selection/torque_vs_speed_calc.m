close all;
time = out.cable_tensions.Time;
ct = out.cable_tensions.Data;
ldd = out.ldd.Data;
ld = out.ld.Data;
% size(ld)

% Cable tensions
ct1 = ct(1,1,:);
ct1 = ct1(:);

ct2 = ct(2,1,:);
ct2 = ct2(:);

ct3 = ct(3,1,:);
ct3 = ct3(:);

ct4 = ct(4,1,:);
ct4 = ct4(:);

% Derivatives of cable lengths
ld1 = ld(1,1,:);
ld1 = ld1(:);

ld2 = ld(2,1,:);
ld2 = ld2(:);

ld3 = ld(3,1,:);
ld3 = ld3(:);

ld4 = ld(4,1,:);
ld4 = ld4(:);

% Double derivative of cable lengths
ldd1 = ldd(1,1,:);
ldd1 = ldd1(:);

ldd2 = ldd(2,1,:);
ldd2 = ldd2(:);

ldd3 = ldd(3,1,:);
ldd3 = ldd3(:);

ldd4 = ldd(4,1,:);
ldd4 = ldd4(:);


% Only get values after t = 0.5 [s] to remove values due to initial cable
% tensioning at t = 0 [s]
index = find(time>=1.0,1)
index_end = find(time>=5.0,1)
%%
time = time(index:index_end);

ct1 = ct1(index:index_end);
ct2 = ct2(index:index_end);
ct3 = ct3(index:index_end);
ct4 = ct4(index:index_end);

ldd1 = ldd1(index:index_end);
ldd2 = ldd2(index:index_end);
ldd3 = ldd3(index:index_end);
ldd4 = ldd4(index:index_end);

ld1 = ld1(index:index_end);
ld2 = ld2(index:index_end);
ld3 = ld3(index:index_end);
ld4 = ld4(index:index_end);

% Parameters
r_winch = 35e-3; %
% 
J_m = 15.17e-6; % kgm^2 https://ieeexplore.ieee.org/abstract/document/5984365
J_m = 120e-7; 
J_m = 242e-6;
theta1dd_r = ldd1/r_winch;
theta2dd_r = ldd2/r_winch;
theta3dd_r = ldd3/r_winch;
theta4dd_r = ldd4/r_winch;

tau1 = ct1*r_winch;
tau2 = ct2*r_winch;
tau3 = ct3*r_winch;
tau4 = ct4*r_winch;

% kgm^2/s^2+kgm^2/s^2
max(J_m*abs(theta3dd_r))
llll = max(tau3)

radps_to_rpm = 30/pi;
aaaaa = max(ld3/r_winch*radps_to_rpm)
%%
plot(time, theta1dd_r); hold on;
plot(time, theta2dd_r)
plot(time, theta3dd_r)
plot(time, theta4dd_r); hold off;
figure(2)
plot(time, ct1); hold on;
plot(time, ct2)
plot(time, ct3)
plot(time, ct4); hold off;
%%
plot(time,J_m*abs(theta3dd_r))
%%
max(J_m*abs(theta3dd_r)+abs(tau3))
max(J_m*abs(theta3dd_r)+abs(tau3))/9.81
plot(time,J_m*abs(theta3dd_r)+abs(tau3))
% plot(time, abs(J_m*ldd1)+abs(ct1))

%% ODrive Robotics D6374 - 150kv
tau_max = 3.86; % [Nm]
vel_max = 603.19; % [rad/s]




tau_e3 = J_m*(theta3dd_r)+(tau3);
v3 = abs(ld3/r_winch);
tau_en3 = tau_e3./tau_max;
vn3 = v3./vel_max;
%%
max(J_m*abs(theta3dd_r)+abs(tau3))
figure(1)
plot(time,ld3/r_winch)
figure(2)
max(theta3dd_r)
max(J_m*(theta3dd_r)+(tau3))
% plot(abs(ld3/r_winch),J_m*(theta3dd_r)+(tau3), '.'); grid
plot(vn3,tau_en3, '.'); hold on; grid
plot([0, 0.775], [0.3 0.3]); hold off;

figure(3)
plot(abs(ld1/r_winch),J_m*(theta4dd_r)+(tau1), '.'); grid
figure(4)
plot(abs(ld2/r_winch),J_m*(theta4dd_r)+(tau2), '.'); grid
figure(5)
plot(abs(ld3/r_winch),J_m*(theta4dd_r)+(tau3), '.'); grid
figure(6)
plot(abs(ld4/r_winch),J_m*(theta4dd_r)+(tau4), '.'); grid