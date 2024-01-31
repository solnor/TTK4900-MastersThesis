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
index = find(time>=0.5,1)
time = time(index:end);

ct1 = ct1(index:end);
ct2 = ct2(index:end);
ct3 = ct3(index:end);
ct4 = ct4(index:end);

ldd1 = ldd1(index:end);
ldd2 = ldd2(index:end);
ldd3 = ldd3(index:end);
ldd4 = ldd4(index:end);

ld1 = ld1(index:end);
ld2 = ld2(index:end);
ld3 = ld3(index:end);
ld4 = ld4(index:end);

% Parameters
r_winch = 30e-3; %
% 
J_m = 15.17e-6; % kgm^2 https://ieeexplore.ieee.org/abstract/document/5984365
J_m = 120e-7; 
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
plot(time,J_m*abs(theta3dd_r))
%%
max(J_m*abs(theta3dd_r)+abs(tau3))
max(J_m*abs(theta3dd_r)+abs(tau3))/9.81
plot(time,J_m*abs(theta3dd_r)+abs(tau3))
% plot(time, abs(J_m*ldd1)+abs(ct1))

%%

%%

max(J_m*abs(theta3dd_r)+abs(tau3))
figure(1)
plot(time,ld3/r_winch)
figure(2)
max(theta3dd_r)
plot((ld3/r_winch),J_m*(theta3dd_r)+(tau3), '.'); grid