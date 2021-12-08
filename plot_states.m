function plot_states(t,x,r,fname)
% plotting position
f = figure('Visible', 'Off');

tiledlayout(3,1)
nexttile
hold on
grid on
plot(t,x(:,1))
plot(t,r(:,1), '--k')
ylabel('x (km)')
xlabel('Time (s)')
legend('x(t)','Reference')

nexttile
hold on
grid on
plot(t,x(:,2))
plot(t,r(:,2), '--k')
ylabel('y (km)')
xlabel('Time (s)')
legend('y(t)','Reference')

nexttile
hold on
grid on
plot(t,x(:,3))
plot(t,r(:,3), '--k')
ylabel('z (km)')
xlabel('Time (s)')
legend('z(t)','Reference')

exportgraphics(f, fname + "_pos.png");

% Plotting velocity

f = figure('Visible', 'Off');

tiledlayout(3,1)
nexttile
hold on
grid on
plot(t,x(:,4))
ylabel('x dot (km/s)')
xlabel('Time (s)')

nexttile
hold on
grid on
plot(t,x(:,5))
ylabel('y dot (km/s)')
xlabel('Time (s)')

nexttile
hold on
grid on
plot(t,x(:,6))
ylabel('z dot (km/s)')
xlabel('Time (s)')

exportgraphics(f, fname + "_vel.png");
close
end