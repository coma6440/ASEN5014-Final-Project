function [sys,K,F] = leunberger(A,B,C,F,K,L_poles,t,r,umax,x0,fname)
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

plot_states(t,x,r,fname);
plot_controls(t,u,umax,fname);
plot_error_states(t,x,fname);
plot_outputs(t,y,r,fname);
end