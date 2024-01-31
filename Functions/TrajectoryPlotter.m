function TrajectoryPlotter(ts_w,ts_qd,ts_q)

h =  findobj('type','figure');
if isempty(h)
    h = 1;
end
n = length(h);
figure(n+1)

subplot(4,1,1);
plot(ts_q.Time,ts_q.Data(:,1))
legend('x')

subplot(4,1,2);
plot(ts_q.Time,ts_q.Data(:,2))
legend('y')

subplot(4,1,3);
plot(ts_q.Time,ts_q.Data(:,3))
legend('theta')

subplot(4,1,4);
plot(ts_w.Time,ts_w.Data(:,1)); hold on;
plot(ts_w.Time,ts_w.Data(:,2));
plot(ts_w.Time,ts_w.Data(:,3)); hold off;
legend('$f_x$', '$f_y$', '$\tau$', 'interpreter', 'latex')



end