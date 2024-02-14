function [f, flag] = Optimal_ForceDistributions(A,w_c,m_p,f_min,f_max, f_ref)

% Kernel Translation V2.
%
% A: Structure matrix
% w: Wrench applied on the platform
% f_min: Minimum Cable Tension
% f_max: Maximum Cable Tension
% f_ref: Desired/reference Cable Tension
%
% Flag Value Representations:
% Flag = 0: Acceptable Solution
% Flag = 1: Kernel has different signs, uncontrollable 
% Flag = 2: No acceptable step length
%
% Elias Olsen Almenningen & Magnus Grøterud
%% Kernel Translation V2: return of the dumbfuckery 

% Initialize Flag
flag = 0;

% Calculate the kernel h of A^T
m = length(A);
A_t = A;
[~, ~, V] = svd(A_t);
h = V(:, end);

wp = m_p*9.81;
w = wp-w_c;

% 
fm = 0.5*(f_min*ones(4,1) + f_max*ones(4,1));                   % Medium Feasible Cable Force
A_pseudo = pinv(A_t);                                           % Pseudo-Inverse of structure matrix
f0 = -(A_pseudo*w);                                               % Arbitrarily Solution on the line

if (min(f0) > 0) && (max(f0) < f_max) && (min(f0) > f_min)      % If arbitrarily solution is wrench-feasible
    f = f0;
else                                                            % Else, need to move it 
    % Check if signs are equal 
    % (If we can "move" the cabel forces along the nullspace)
    if ~all(sign(h) == sign(h(1)))
        flag = 1;
        f = f_ref*ones(4,1);
        return;
    end

    % Calculate Step Sizes
    lambda_l = zeros(m,1);          % Lowest stepsize (Some Cables on f_min)
    lambda_h = zeros(m,1);          % Highest stepsize (Some cables on f_max)
    lambda_r = zeros(m,1);          % Desired stepsize (Preferred cable tension )

    for i = 1:m
        lambda_l(i) = (f_min - f0(i))/h(i);
        lambda_h(i) = (f_max - f0(i))/h(i);
        lambda_r(i) = (f_ref - f0(i))/h(i); % Test
    end
    
    % 
    lambda_min = max(lambda_l);
    lambda_max = min(lambda_h);
    lambda_ref = mean(lambda_r);            

    if abs(lambda_ref) > abs(lambda_min) && abs(lambda_ref) < abs(lambda_max)   % If desired step is ok
        f = f0 + h*lambda_ref;
        % Check if reference cable tension is feasible, else change it
        if min(f) < f_min
            f = f0 + h*lambda_min;
        elseif max(f) > f_max
            f = f0 + h*lambda_max;
        end
    else                                                    % If desired step is not ok
        % Mulig at dette er overflødig
        f = f0 + h*0.5*(lambda_max + lambda_min);           % Use medium cable force
    end
    % DENNE LØSNINGEN MÅ FIKSES
    if (min(f) < f_min) || (max(f) > f_max)             % If the solution is not acceptable
        f = fm;                                         % No solution, return to center of frame
        flag = 2;                                       % Set flag = 2 to alert
    end
    
end

