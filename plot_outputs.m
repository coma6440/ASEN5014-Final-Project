function plot_outputs(t,y,r,fname)
% Plot outputs
f = figure('Visible', 'Off');

tiledlayout(3,1)
nexttile
hold on
grid on
plot(t,y(:,1))
plot(t,r(:,1), '--k')
ylabel('x (km)')
xlabel('Time (s)')
legend('x(t)','Reference')

nexttile
hold on
grid on
plot(t,y(:,2))
plot(t,r(:,2), '--k')
ylabel('y (km)')
xlabel('Time (s)')
legend('y(t)','Reference')

nexttile
hold on
grid on
plot(t,y(:,3))
plot(t,r(:,3), '--k')
ylabel('z (km)')
xlabel('Time (s)')
legend('z(t)','Reference')

exportgraphics(f, fname + "_out.png");
close
end