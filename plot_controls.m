function plot_controls(t,u,umax,fname)
% Plot controls
f = figure('Visible', 'Off');

tiledlayout(3,1)
nexttile
hold on
grid on
plot(t,u(1,:))
yline(umax,'--k')
yline(-umax,'--k')
ylabel('ux (km/s^2)')
xlabel('Time (s)')
legend(["Input", "Bounds"])

nexttile
hold on
grid on
plot(t,u(2,:))
yline(umax,'--k')
yline(-umax,'--k')
ylabel('uy (km/s^2)')
xlabel('Time (s)')
legend(["Input", "Bounds"])

nexttile
hold on
grid on
plot(t,u(3,:))
yline(umax,'--k')
yline(-umax,'--k')
ylabel('uz (km/s^2)')
xlabel('Time (s)')
legend(["Input", "Bounds"])

exportgraphics(f, fname + "_controls.png");

% Control effort
[ex, ey, ez, et] = control_effort(t,u);
end