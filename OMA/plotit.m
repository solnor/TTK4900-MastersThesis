time = out.simout.Time;
data = out.simout.Data;
data = reshape(data,size(data,1),size(data,3));

p = data(1:2,:,:);
psi = data(3,:);

a1 = a(:,1);
a2 = a(:,2);
a3 = a(:,3);
a4 = a(:,4);

b1 = b(:,1);
b2 = b(:,2);
b3 = b(:,3);
b4 = b(:,4);


% Preallocate Data
b1g = zeros(2, size(time, 1));
b2g = zeros(2, size(time, 1));
b3g = zeros(2, size(time, 1));
b4g = zeros(2, size(time, 1));

% Precalculate all Body Anchor Positions
for n = 1:max(size(time))
    b1g(:,n) = b_to_g(p(:,n),psi(n),b1);
    b2g(:,n) = b_to_g(p(:,n),psi(n),b2);
    b3g(:,n) = b_to_g(p(:,n),psi(n),b3);
    b4g(:,n) = b_to_g(p(:,n),psi(n),b4);
end

axes_limits = [-0.7,0.7,-0.4,0.4];

% Change this to make faster animation
k = 2;

% Initial plot
figure(1);
sq = [b1g(:,1), b2g(:,1), b3g(:,1), b4g(:,1), b1g(:,1)];
h = plot(sq(1,:), sq(2,:), 'b');
hold on;
plot([a1(1), b1g(1,1)], [a1(2), b1g(2,1)], 'r');
plot([a2(1), b2g(1,1)], [a2(2), b2g(2,1)], 'r');
plot([a3(1), b3g(1,1)], [a3(2), b3g(2,1)], 'r');
plot([a4(1), b4g(1,1)], [a4(2), b4g(2,1)], 'r');
axis(axes_limits);
set(h, 'LineWidth', 1);
pause
for n = 1:max(size(time))
   
    % Start of the animation
    
    if mod(n, k) == 0
        % Body anchors in global coordinates
        sq = [b1g(:,n),b2g(:,n),b3g(:,n),b4g(:,n),b1g(:,n)];
        h=plot(sq(1,:),sq(2,:),'b');
        hold on;
        plot([a1(1),b1g(1,n)],[a1(2),b1g(2,n)],'r');
        plot([a2(1),b2g(1,n)],[a2(2),b2g(2,n)],'r');
        plot([a3(1),b3g(1,n)],[a3(2),b3g(2,n)],'r');
        plot([a4(1),b4g(1,n)],[a4(2),b4g(2,n)],'r');
        % plot(ball_p(n,1),ball_p(n,2),'*');
        hold off;
        axis(axes_limits);
        set(h,"LineWidth",1);
        % axis equal;
        title(time(n))
    end
    pause(0.001)
end