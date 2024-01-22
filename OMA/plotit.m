length = 0.4;
height = 0.2;

time = out.simout.Time;
data = out.simout.Data;

p = data(:,1:2);
psi = data(:,3);

ball_t = out.ball_p.Time;
ball_p = out.ball_p.Data;

% % Anchor points
% a1 = [1;1];
% a2 = [-1;1];
% a3 = [-1;-1];
% a4 = [1;-1];
% 
% % Body anchors
% b1 = [length/2;height/2];
% b2 = [-length/2;height/2];
% b3 = [-length/2;-height/2];
% b4 = [length/2;-height/2];

axes_limits = [-1,1,-1,1];
for n = 1:max(size(time))
    % Body anchors in global coordinates
    b1g = b_to_g(p(n,:)',psi(n),b1);
    b2g = b_to_g(p(n,:)',psi(n),b2);
    b3g = b_to_g(p(n,:)',psi(n),b3);
    b4g = b_to_g(p(n,:)',psi(n),b4);
    sq = [b1g,b2g,b3g,b4g,b1g];
    h=plot(sq(1,:),sq(2,:),'b');
    hold on;
    plot([a1(1),b1g(1)],[a1(2),b1g(2)],'r');
    plot([a2(1),b2g(1)],[a2(2),b2g(2)],'r');
    plot([a3(1),b3g(1)],[a3(2),b3g(2)],'r');
    plot([a4(1),b4g(1)],[a4(2),b4g(2)],'r');
    plot(ball_p(n,1),ball_p(n,2),'*');
    hold off;
    axis(axes_limits);
    set(h,"LineWidth",3);
    % axis equal;
    if n==1
        pause;
    else
        pause(0.01);
    end
end