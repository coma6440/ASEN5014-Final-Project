function plot_error_states(t,x,fname)
% Plot position Error States
f = figure('Visible', 'Off');
tiledlayout(3,1)
nexttile
hold on
grid on
plot(t,x(:,7))
ylabel('x error (m)')
xlabel('Time (s)')

nexttile
hold on
grid on
plot(t,x(:,8))
ylabel('y error (m)')
xlabel('Time (s)')

nexttile
hold on
grid on
plot(t,x(:,9))
ylabel('z error (m)')
xlabel('Time (s)')
saveas(f, fname + "_pos_error.png");
close

% plot velocity error states
f = figure('Visible', 'Off');
tiledlayout(3,1)
nexttile
hold on
grid on
plot(t,x(:,10))
ylabel('x dot error (m)')
xlabel('Time (s)')

nexttile
hold on
grid on
plot(t,x(:,11))
ylabel('y dot error (m)')
xlabel('Time (s)')

nexttile
hold on
grid on
plot(t,x(:,12))
ylabel('z dot error (m)')
xlabel('Time (s)')
saveas(f, fname + "_vel_error.png");
close
end