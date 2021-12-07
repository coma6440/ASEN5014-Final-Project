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
dt = 1;
altitude = 500; % km
r_earth = 6371; % km
a = altitude + r_earth;
mu_earth = 3.986e5;
period = 2*pi*sqrt(a^3/mu_earth);
t = 0:dt:period;
rmag = 1;
x0 = [0 -10 0 0 0 0]';
[r_step,r_zero,r_piece,r_MO1,r_MO2,r_MO3,t_r_MO3,r_MO3_zeros,r_y_neg_10] = r_t_generator(rmag,t,dt,x0,period);
% plot(t,r_MO1)

%% Open Loop
[OL_poles,y1] = OL_Response(A,sys_OL,r_step,r_zero,r_piece,r_MO1,r_MO2,r_MO3,r_MO3_zeros,t_r_MO3,t,x0);

%% Closed Loop Reference Tracking Feedback Control
% Setting CL poles
% CL_poles = [-1 -2 -3 -4 -5 -6];
CL_poles = [-15 -20 -5 -25 -10 -30]; % Satisfies specs for all r(t), u(t) way too high, will adjust after require are specified;

% 2nd and 3rd pole affects y response only
% 1st and 4th pole effects z response only
% 5th and 6th poles effects z response only 

% % Feedback control for r(t) = [step step step]
r = [r_step r_step+r_y_neg_10 r_step];
fig_num = 1;
CL_Ref_Track_Cont(A,B,C,D,CL_poles,t,r,x0,fig_num);

% Feedback control for r(t) = [step 0 0]
r = [r_step r_zero+r_y_neg_10 r_zero];
fig_num = 2;
CL_Ref_Track_Cont(A,B,C,D,CL_poles,t,r,x0,fig_num);

% Feedback control for r(t) = [0 step 0]
r = [r_zero r_step+r_y_neg_10 r_zero];
fig_num = 3;
CL_Ref_Track_Cont(A,B,C,D,CL_poles,t,r,x0,fig_num);
% 
% % Feedback control for r(t) = [0 0 step]
r = [r_zero r_zero+r_y_neg_10 r_step];
fig_num = 4;
CL_Ref_Track_Cont(A,B,C,D,CL_poles,t,r,x0,fig_num);

% Feedback control for MO1, r(t) = [r_MO1 0 0]
r = [r_MO1 r_zero+r_y_neg_10 r_zero];
fig_num = 5;
CL_Ref_Track_Cont(A,B,C,D,CL_poles,t,r,x0,fig_num);

% % Feedback control for MO1, r(t) = [0 r_MO2 0]
r = [r_zero r_MO2 r_zero];
fig_num = 6;
CL_Ref_Track_Cont(A,B,C,D,CL_poles,t,r,x0,fig_num); 
% 
% % Feedback control for MO1, r(t) = [r_MO1 0 0]
r = [r_MO3_zeros r_MO3_zeros+r_y_neg_10(1:length(r_MO3_zeros),:) r_MO3];
fig_num = 7;
[~, K, F] = CL_Ref_Track_Cont(A,B,C,D,CL_poles,t_r_MO3,r,x0,fig_num);

% Does no coupling n states make sense? Ask Reade and Conner 

%% Luenberger observer 
P_L = [-3, -4, -5, -6, -7, -8];
r = [r_step r_step+r_y_neg_10 r_step];
x0_true = [x0; zeros(6,1)];
x0_error = 1*ones(6,1);
x0_guess = [x0 + x0_error; 5.*x0_error];


% Simulate with zero initial error
fprintf("Simulating closed loop with observer, zero error...\n");
leunberger(A,B,C,F,K,P_L,t,r,x0_true, "Images/obs_zero_init_err");

% Simulate with non-zero initial error
fprintf("Simulating closed loop with observer, nonzero error...\n");
leunberger(A,B,C,F,K,P_L,t,r,x0_guess, "Images/obs_nonzero_init_err");

%% LQR Optimal State Feedback WITHOUT Integral control and WITHOUT Observer

t = t';
r = [r_zero r_MO2 r_zero];

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

%% LQR WITH integral control and observer

Aaug = [A zeros(6,3); -C zeros(3,3)];
Baug = [B; zeros(3,3)];
Caug = [C zeros(3,3)];
Daug = zeros(size(Caug,1),size(Baug,2));
augOLsys = ss(Aaug,Baug,Caug,Daug);

%Define closed-loop plant poles via LQR (only vary awts for now)
awts = [ones(1,n/2)*100, ones(1,n/2),50,50,50]; %initial design: relative penalties
bwts = [1,1,1];
rho = 12;

awts = awts./sum(awts);
bwts = bwts./sum(bwts);
xmax = [10, 10, 10, 1, 1, 1, 1^2, 1^2, 1^2];
Qaug =  diag(awts./xmax); 
Raug =  rho*diag(bwts./umax); %rho * eye(p);

[Kaug,Waug,clEvalsAug] = lqr(augOLsys,Qaug,Raug);

XCLO_IC = 0*ones(15,1); %change the IC to 0.1's and see what happens!!
% XCLO_IC = 0.1*ones(15,1); %change the IC to 0.1's and see what happens!!

[CLaugsys,Y_CLOaug,U_CLOaug,Faug] = simLQR(sys_OL,augOLsys,Kaug,P_L,...
    t,r,XCLO_IC,umax,'/Images/LQR_refx_initErrx');

