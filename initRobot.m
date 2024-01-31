function [a,b,m_MP,Iz_MP] = initRobot(F_len_x,F_len_y,MP_len_x,MP_len_y)

% Cable attachment points PULLEY (In INERTIA coordinates) CONSTANT
a1              = [-F_len_x/2;-F_len_y/2];  
a2              = [-F_len_x/2;F_len_y/2];
a3              = [F_len_x/2;F_len_y/2];    
a4              = [F_len_x/2;-F_len_y/2];   

% Cable attachment point PLATFORM
b1              = [-MP_len_x/2;-MP_len_y/2];
b1_triangle     = [0;-MP_len_y/2];
b2              = [-MP_len_x/2;MP_len_y/2];   
b3              = [MP_len_x/2;MP_len_y/2];    
b4              = [MP_len_x/2;-MP_len_y/2];   

% Matrices of the attachment points
a               = [a1 a2 a3 a4];        
b               = [b1 b2 b3 b4];    % Rectangle
b               = [b1_triangle b2 b3 b1_triangle];    % Triangle

m_MP            = 0.25; % [kg]
Iz_MP           = 1/12*m_MP*(MP_len_x^2+MP_len_y^2); % [kg m^2]

end