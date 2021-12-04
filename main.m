%% Housekeeping
close all
clear
clc

%% Constants
n = sqrt(298600/(6778^3)); %Mean motion [1/s]

%% State Space Model
A = [0,0,0,1,0,0;
     0,0,0,0,1,0;
     0,0,0,0,0,1;
     3*n^2,0,0,0,2*n,0;
     0,0,0,-2*n,0,0;
     0,0,-(n^2),0,0,0];
B = [0,0,0;
     0,0,0;
     0,0,0;
     1,0,0;
     0,1,0;
     0,0,1];
C = [1,0,0,0,0,0;
     0,1,0,0,0,0;
     0,0,1,0,0,0];
D = zeros(3,3);

sys_OL = ss(A,B,C,D);
pzmap(sys_OL);
title("");
saveas(gcf, "Images/pzmap.png")
close
e_OL = eig(A);
%% Reachability and Observability
P = ctrb(A,B);
O = obsv(A,C);
fprintf("Rank of controllability matrix = %d\n",rank(P));
fprintf("Rank of observability matrix = %d\n", rank(O));

%% Generating r(t) for different mission scenarios 
dt = 0.01;
t = 0:dt:60;
rmag = 1;
x0 = [0 -10 0 0 0 0]';
[r_step,r_zero,r_piece,r_MO1,r_MO2,r_MO3,t_r_MO3,r_MO3_zeros,r_y_neg_10] = r_t_generator(rmag,t,dt,x0);
% plot(t,r_MO2)

%% Open Loop
[OL_poles,y1] = OL_Response(A,sys_OL,r_step,r_zero,r_piece,r_MO1,r_MO2,r_MO3,r_MO3_zeros,t_r_MO3,t,x0);

%% Closed Loop Reference Tracking Feedback Control
% Setting CL poles
% CL_poles = [-1 -2 -3 -4 -5 -6];
CL_poles = [-0.5 -1 -2 -5 -6 -10];

% Feedback control for r(t) = [step step step]
r = [r_step r_step+r_y_neg_10 r_step];
CL_Ref_Track_Cont(A,B,C,D,CL_poles,t,r,x0);

% Feedback control for r(t) = [step 0 0]
r = [r_step r_zero+r_y_neg_10 r_zero];
CL_Ref_Track_Cont(A,B,C,D,CL_poles,t,r,x0);

% Feedback control for r(t) = [0 step 0]
r = [r_zero r_step+r_y_neg_10 r_zero];
CL_Ref_Track_Cont(A,B,C,D,CL_poles,t,r,x0);

% Feedback control for r(t) = [0 0 step]
r = [r_zero r_zero+r_y_neg_10 r_step];
CL_Ref_Track_Cont(A,B,C,D,CL_poles,t,r,x0);

% Feedback control for MO1, r(t) = [r_MO1 0 0]
r = [r_MO1 r_zero+r_y_neg_10 r_zero];
CL_Ref_Track_Cont(A,B,C,D,CL_poles,t,r,x0);

% Feedback control for MO1, r(t) = [0 r_MO2 0]
r = [r_zero r_MO2 r_zero];
CL_Ref_Track_Cont(A,B,C,D,CL_poles,t,r,x0); %___Need to change ramp to work with IC____

% Feedback control for MO1, r(t) = [r_MO1 0 0]
r = [r_MO3_zeros r_MO3_zeros+r_y_neg_10(1:length(r_MO3_zeros),:) r_MO3];
CL_Ref_Track_Cont(A,B,C,D,CL_poles,t_r_MO3,r,x0);

% Does no coupling n states make sense? Ask Reade and Conner 
%% Closed Loop
P_CL = [-1,-2,-3,-4,-5,-6];
K = place(A,B,P_CL);

F = eye(3);

%% Luenberger observer
P_L = [-1, -2, -3, -4, -5, -6];
L = place(A', C', P_L)';

A_cl_aug = [(A-B*K), B*K; zeros(6), (A-L*C)];
B_cl_aug = [B*F;zeros(6,3)];
C_cl_aug = [C, zeros(3,6)];
D_cl_aug = 0;

sys_CL_OBS = ss(A_cl_aug, B_cl_aug, C_cl_aug, D_cl_aug);

% Simulate with zero initial error

% Simulate with non-zero initial error


%% LQR Optimal State Feedback WITHOUT Integral control and WITHOUT Observer

t = t';
r = [r_MO1 r_zero+r_y_neg_10 r_zero];

%%Actuator constraints
umax = 5; %??? what is a reasonable acceleration constraint ???
n = size(A,1); %#nominal states
m = size(B,2); %#nominal inputs
p = size(C,1); %#nominal outputs

%Define closed-loop plant poles via LQR (only vary awts for now)
awts = [ones(1,n/2)*100, ones(1,n/2)]; %initial design: relative penalties
bwts = ones(1,p);
rho = 1;

awts = awts./sum(awts);
xmax = [10 10 10 .1 .1 .1];
Q =  diag(awts./xmax); 
R =  rho*diag(bwts./umax); %rho * eye(p);

OLsys = ss(A,B,C,D);
[K,W,clEvals] = lqr(OLsys,Q,R); %get optimal K, W

F = inv(C/(-A+B*K)*B);

%CL system
Acl = A - B*K;
Bcl = B*F;
Ccl = C;
Dcl = D;
CLsys = ss(Acl,Bcl,Ccl,Dcl);

%%Check that closed-loop system specs met; change rho o'wise
%%get response to first reference input profile:
[Y_CL1,~,X_CL1] = lsim(CLsys,r',t');
%compute resulting actuator efforts in each case, where u = -Kx + Fr
U_CL1 = -K*X_CL1' + F*r';

%%Plot output response for each input/output channel; compare to desired
%%reference positions
figure()
    subplot(3,1,1), hold on; grid on
plot(t,Y_CL1(:,1),'b')
plot(t,r(:,1),'k--')
legend('y(t)','reference')
title('y_1 vs. u_1 for rhist1','FontSize',14)
xlabel('t (secs)','FontSize',14)
ylabel('displacement (m)','FontSize',14)
    subplot(3,1,2), hold on; grid on
plot(t,Y_CL1(:,2),'b')
plot(t,r(:,2),'k--')
legend('y(t)','reference')
title('y_2 vs. u_1 for rhist1','FontSize',14)
xlabel('t (secs)','FontSize',14)
ylabel('displacement (m)','FontSize',14)
    subplot(3,1,3), hold on; grid on
plot(t,Y_CL1(:,3),'b')
plot(t,r(:,3),'k--')
legend('y(t)','reference')
title('y_3 vs. u_1 for rhist1','FontSize',14)
xlabel('t (secs)','FontSize',14)
ylabel('displacement (m)','FontSize',14)

%%Plot actuator efforts and compare to constraints on u
figure()
hold on; grid on
plot(t, U_CL1(1,:),'r')
plot(t, U_CL1(2,:),'b')
plot(t, U_CL1(3,:),'g')
plot(t,umax*ones(size(t)),'k--')
plot(t,-umax*ones(size(t)),'k--')
xlabel('t (secs)','FontSize',14)
ylabel('u effort (m/s^2)','FontSize',14)
legend('u_1','u_2','u_3','u constraint')
title('Actuator response for r(t)','FontSize',14)