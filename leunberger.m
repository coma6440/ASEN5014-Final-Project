function [sys,K,F] = leunberger(A,B,C,F,K,L_poles,t,r,x0,fname)
L = place(A', C', L_poles)';
% TODO: Do we want to do integral control, will need to change K if so
A_aug = [(A-B*K), B*K; zeros(6), (A-L*C)];
B_aug = [B*F;zeros(6,3)];
C_aug = [C, zeros(3,6)];
D_aug = 0;

sys = ss(A_aug, B_aug, C_aug, D_aug);
[y,~,x] = lsim(sys,r,t,x0);

for i = 1:length(r)
    u(:,i) = -[K, K(:,1:6)]*x(i,:)'+F*r(i,:)';
end

%% Plot States
% Raj is plotting y(t) (observations), should maybe be plotting x(t)
% states?
f = figure('Visible', 'Off');

tiledlayout(3,1)
nexttile
hold on
grid on
plot(t,x(:,1))
plot(t,r(:,1), '--k')
ylabel('x (m)')
xlabel('Time (s)')
legend('x(t)','Reference')

nexttile
hold on
grid on
plot(t,x(:,2))
plot(t,r(:,2), '--k')
ylabel('y (m)')
xlabel('Time (s)')
legend('y(t)','Reference')

nexttile
hold on
grid on
plot(t,x(:,3))
plot(t,r(:,3), '--k')
ylabel('z (m)')
xlabel('Time (s)')
legend('z(t)','Reference')

saveas(f, fname + "_pos.png");

f = figure('Visible', 'Off');

tiledlayout(3,1)
nexttile
grid on
plot(t,x(:,4))
ylabel('x dot (m/s)')
xlabel('Time (s)')

nexttile
grid on
plot(t,x(:,5))
ylabel('y dot(m/s)')
xlabel('Time (s)')

nexttile
grid on
plot(t,x(:,6))
ylabel('z dot (m/s)')
xlabel('Time (s)')

saveas(f, fname + "_vel.png");
%% Plot controls
f = figure('Visible', 'Off');

tiledlayout(3,1)
nexttile
grid on
plot(t,u(1,:))
ylabel('ux (m/s)')
xlabel('Time (s)')

nexttile
grid on
plot(t,u(2,:))
ylabel('uy (m/s)')
xlabel('Time (s)')

nexttile
grid on
plot(t,u(3,:))
ylabel('uz (m/s)')
xlabel('Time (s)')

saveas(f, fname + "_controls.png");

%% Plot Error States
f = figure('Visible', 'Off');
tiledlayout(3,1)
nexttile
grid on
plot(t,x(:,7))
ylabel('x error (m)')
xlabel('Time (s)')

nexttile
grid on
plot(t,x(:,8))
ylabel('y error (m)')
xlabel('Time (s)')

nexttile
grid on
plot(t,x(:,9))
ylabel('z error (m)')
xlabel('Time (s)')
saveas(f, fname + "_pos_error.png");
close

f = figure('Visible', 'Off');
tiledlayout(3,1)
nexttile
grid on
plot(t,x(:,10))
ylabel('x dot error (m)')
xlabel('Time (s)')

nexttile
grid on
plot(t,x(:,11))
ylabel('y dot error (m)')
xlabel('Time (s)')

nexttile
grid on
plot(t,x(:,12))
ylabel('z dot error (m)')
xlabel('Time (s)')
saveas(f, fname + "_vel_error.png");
close
end