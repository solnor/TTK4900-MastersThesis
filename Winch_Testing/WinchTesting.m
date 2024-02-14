%% Winch Testing

%% Sim
out = sim("Simulation\CDPR_simulation.slx")
%% General Testing

% Standard Geometric Model
MP_len_x = 0.15;
MP_len_y = 0.02;
F_len_x  = 1.4;
F_len_y  = 0.8;
[a, b, Wp, m, Iz] = initRobot(MP_len_x, MP_len_y, F_len_x, F_len_y);

% Initial platform position
xp0         = -0.3;
yp0         = -0.2;
thetap0     = -0.0;
rp0         = [xp0; yp0];
q0          = [rp0; thetap0];

% Initial cable lengths
[L0,l0, A_transposed_0] = CDPR_InverseKinematics_V2(q0, a, b);




%% Winch Param
% ldData = out.ld.Data;
% ldTime = out.ld.Time;

% % Extract Data from Simulink format
% cableLengths = zeros(4, max(size(ldTime)));
% for i=1:max(size(ldTime))
%     cableLengths(:,i) = ldData(:,:,i);
% end

% Additional Cable Lengths
d = 0.2;                                % CADDING Horizontal cable length between pulleys (=0 if only one pulley)
h0 = 0.4;                               % CADDING Verticall Cable length between pulley and winch (zero angle)

% Min/Max Cable lengths in Workspace
lmin = 0;                               % FIX
lmax = 1.5;                             % FIX

% Cable and Spool Dimensions
D_c = 2*10^(-3);                        % Cable diameter
k_s = 15;                               % Cable-Spool Factor, Pott s. 372
R_s = D_c*k_s*0.5;                      % Radius of spool, Pott s. 372
D_m = 0.02;                             % Width of "mutter-ting" on the side of the spool
P = 2.6*10^(-3);                        % Pitch of winch drum 

% Min/Max Angle of Rotation (Radians)
thetaMin = -(lmax - lmin)/(2*R_s);      % Min Angular Position of Spool/Motor
thetaMax = (lmax - lmin)/(2*R_s);       % Max Angular Position of Spool/Motor

L_s = (thetaMax*P - thetaMin*P)/(2*pi);      % Length of spool

% Initial conditions
% x0 = 


% x0 = WinchInitialPos(l0(1), L_s, L_min, L_max, d, h);   % Initial Linear Displacement of cable on the winch
% theta0 = 0;                                             % Initial Angular position of spool, prob just 0


% theta =                                                   % Angular position of spool
% x0 = theta(1,1)*D_c;

% Angular velocity of spool (SIMULIIIIINK)
% dtheta = 5;                 % !!!!! Hente fra Simulink !!!!!!
% dx = D_c*dtheta/2*pi;



%% Plotting

% Not used when using data from Simulink
T = 30;      % Simulation time
h0 = 0.2;     % Step size
t = 0:h0:T;

axes_limits = [-L_s,L_s,-L_s,h0+0.1];

% Initial plot
hold on;
% Mutter-ting
plot([-L_s/2 - D_m,-L_s/2],[1.5*R_s,R_s],'r');
plot([L_s/2, L_s/2 + D_m],[R_s,1.5*R_s],'r');
plot([-L_s/2 - D_m,-L_s/2],[-1.5*R_s,-R_s],'r');
plot([L_s/2,L_s/2 + D_m],[-R_s,-1.5*R_s],'r');

% Spool
plot([-L_s/2, -L_s/2],[R_s,-R_s],'r');
plot([L_s/2, L_s/2],[R_s,-R_s],'r');
plot([-L_s/2, L_s/2],[R_s,R_s],'r');
plot([-L_s/2 - D_m ,-L_s/2 - D_m],[1.5*R_s,-1.5*R_s],'r');
plot([L_s/2 + D_m,L_s/2 + D_m],[1.5*R_s,-1.5*R_s],'r');
plot([-L_s/2, L_s/2],[-R_s,-R_s],'r');
plot([x0,0], [R_s, h0])  % Cable Line
% circle(0,d, R_s)


% plot([x0,x0], [R_s, 10])  % Cable Line
hold off;
axis(axes_limits);
grid on;
pause


% Alarm Limits
limMax =  L_s/2;
limMin = -L_s/2;

for n = 1:max(size(ldTime))
   
    % theta = theta0 + dtheta*t(n);
    % % Linear x-position of cable 
    % x = xVec(1,n) + D_c*theta(1,n)/2*pi;
    x = xVec(1,n) + D_c*theta(1,n)/2*pi;

    plot([-L_s/2 - D_m,-L_s/2],[1.5*R_s,R_s],'r');
    hold on;
    plot([L_s/2, L_s/2 + D_m],[R_s,1.5*R_s],'r');
    plot([-L_s/2 - D_m,-L_s/2],[-1.5*R_s,-R_s],'r');
    plot([L_s/2,L_s/2 + D_m],[-R_s,-1.5*R_s],'r');
    
    plot([-L_s/2, -L_s/2],[R_s,-R_s],'r');
    plot([L_s/2, L_s/2],[R_s,-R_s],'r');
    plot([-L_s/2, L_s/2],[R_s,R_s],'r');
    plot([-L_s/2 - D_m ,-L_s/2 - D_m],[1.5*R_s,-1.5*R_s],'r');
    plot([L_s/2 + D_m,L_s/2 + D_m],[1.5*R_s,-1.5*R_s],'r');
    plot([-L_s/2, L_s/2],[-R_s,-R_s],'r');
    plot([x0,0], [R_s, h0])  % Cable Line
    % circle(0,d, R_s)
    hold off
    axis(axes_limits);
    grid on;
        
    % Fra ChatGPT for å vise simuleringstid
    % Calculate midpoint for x-coordinate of the text
    mid_x = mean(axes_limits(1:2));
    % Set y-coordinate for the text slightly above the top of the plot
    text_y = axes_limits(4) ;%+ 0.05;
    % Displaying the simulation time
    text(mid_x, text_y, sprintf('Time: %.2f seconds', ldTime(n)), ...
         'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
    
    % "Alarm" if max/min displacement is reached
    if x >= limMax
        text(mid_x, text_y - 0.05, sprintf('ERROR: Max x reached'), ...
         'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
    elseif x <= limMin
         text(mid_x, text_y - 0.05, sprintf('ERROR: Min x reached'), ...
         'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
    end

    % Forward Euler Integration
    % theta = theta + dtheta*h;
    % x = x + dx*h;

    pause(0.05)
    grid on;
end


% Plottefunksjon for evt pulley
function circle(x,y,r)
hold on
th = 0:pi/50:2*pi;
xunit = r * cos(th) + x;
yunit = r * sin(th) + y;
plot(xunit, yunit);
hold off
end