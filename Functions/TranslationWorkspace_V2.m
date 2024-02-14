function TranslationWorkspace_V2(phi_0,a,b, f_min,f_max, f_ref, w, resolution, color)
% phi0: RADIANS
% 
% 
% 
% 

% Extract Lengths of the Base
x_dim = norm(a(:,3) - a(:,2));
y_dim = norm(a(:,2) - a(:,1));


% Grid of the "Base Workspace"
x_grid = linspace(-x_dim/2, x_dim/2, resolution);
y_grid = linspace(-y_dim/2, y_dim/2, resolution);

X = length(x_grid);
Y = length(y_grid);

% Allocations for solutions
f_positive = zeros(X,Y);

for i=1:X
    for j=1:Y
        pose = [x_grid(i); y_grid(j);phi_0];
        [~,~,A_transpose] = CDPR_InverseKinematics_V2(pose, a, b);
        [f, flag] = General_Optimal_ForceDistributions_V5(A_transpose',w,f_min,f_max, f_ref);
        if flag == 0
            f_positive(i,j) = 1;
        end
    end
end
% Plotting

% Create a grid of coordinates from (0,0) to (1,1)
% [x, y] = meshgrid(linspace(0, 1, size(f_positive, 2)), linspace(0, 1, size(f_positive, 1)));

% [x,y] = meshgrid(x_grid, y_grid);
% Find the (x, y) coordinates where the matrix has a value of 1
[row, col] = find(f_positive == 1);

% Plot the points
scatter(x_grid(row), y_grid(col), 'filled', 'MarkerFaceColor', color); % 'filled' option fills the markers
grid on;
% Set axis labels and title
xlabel('X-axis');
ylabel('Y-axis');
title(['Translation Workspace, with $$\phi _0 = $$ ' num2str(rad2deg(phi_0)) ' degrees'], 'Interpreter','latex');

% Optionally, set axis limits based on the matrix size
 axis([-x_dim/2, x_dim/2, -y_dim/2, y_dim/2]);

% % Displaying the matrix where 1s are plotted and adjusting the axes
% imagesc(y_grid,x_grid, f_positive);
% colormap('gray'); % Setting colormap to grayscale for better visibility
% colorbar; % Displaying colorbar to indicate values
