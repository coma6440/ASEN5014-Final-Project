%% Housekeeping
close all
clear
clc

%% Constants
n = sqrt(298600/(6778^3)); %Mean motion [1/s]
% Actuator constraints
umax = 0.0005; % linear acceleration [km/s^2]
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
f = figure();
pzmap(sys_OL,'r');
title("");
saveas(f, "Images/pzmap.png")
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
fprintf("Generating r(t) profiles...\n");
[r_step,r_zero,r_piece,r_MO1,r_MO2,r_MO3,t_r_MO3,r_MO3_zeros,r_y_neg_10] = r_t_generator(rmag,t,dt,x0,period);


%% Open Loop
fprintf("Simulating open loop response...\n");
[OL_poles,y1] = OL_Response(A,sys_OL,r_step,r_zero,r_piece,r_MO1,r_MO2,r_MO3,r_MO3_zeros,t_r_MO3,t,x0, "Images/OL");

%% Closed Loop Reference Tracking Feedback Control
% Setting CL poles
% CL_poles = [-1 -2 -3 -4 -.02 -.011]; % x tuned well;
% CL_poles = [-1 -.017 -.025 -4 -.02 -.011]; % x and y tuned well;
CL_poles = [-.016 -.017 -.025 -.015 -.02 -.011]; % x y and z tuned well;

% CL_poles = [-0.0001 -0.0002 -1 -2 -3 -4]; 

% 1st pole - z
% 2nd pole - y
% 3rd pole - y
% 4th pole - z
% 5th pole - x - less sensitive
% 6th pole - x - more sensitive


% % Feedback control for r(t) = [step step step]
fprintf("Simulating closed loop response to r1(t)...\n");
r1 = [r_step r_step+r_y_neg_10 r_step];
t1 = t;
CL_Ref_Track_Cont(A,B,C,D,CL_poles,t1,r1,umax,x0, "Images/CL_r1");

% Feedback control for MO1, r(t) = [r_MO1 0 0]
fprintf("Simulating closed loop response to r2(t)...\n");
r2 = [r_MO1 r_zero+r_y_neg_10 r_zero];
t2 = t;
CL_Ref_Track_Cont(A,B,C,D,CL_poles,t2,r2,umax,x0, "Images/CL_r2");

% % Feedback control for MO1, r(t) = [0 r_MO2 0]
fprintf("Simulating closed loop response to r3(t)...\n");
r3 = [r_zero r_MO2 r_zero];
t3 = t;
CL_Ref_Track_Cont(A,B,C,D,CL_poles,t3,r3,umax,x0, "Images/CL_r3"); 
% 

% % Feedback control for MO1, r(t) = [r_MO1 0 0]
fprintf("Simulating closed loop response to r4(t)...\n");
r4 = [r_MO3_zeros r_MO3_zeros+r_y_neg_10(1:length(r_MO3_zeros),:) r_MO3];
t4 = t_r_MO3;
[~, K, F] = CL_Ref_Track_Cont(A,B,C,D,CL_poles,t4,r4,umax,x0, "Images/CL_r4");

%% Luenberger observer 
P_L = [-0.01, -0.02, -0.03, -0.04, -0.05, -0.06];
x0_true = [x0; zeros(6,1)];
% do these magnitudes make sense? 100m and 1 m/s error
x0_error = [0.1*ones(3,1); 0.001*ones(3,1)];
x0_guess = [x0 + x0_error; x0_error];

% Output and states are both the estimated quantity, not true

% Simulate with zero initial error
fprintf("Simulating closed loop with observer, zero error for r1(t)...\n");
leunberger(A,B,C,F,K,P_L,t1,r1,umax,x0_true, "Images/obs_zero_init_err_r1");
fprintf("Simulating closed loop with observer, zero error for r2(t)...\n");
leunberger(A,B,C,F,K,P_L,t2,r2,umax,x0_true, "Images/obs_zero_init_err_r2");
fprintf("Simulating closed loop with observer, zero error for r3(t)...\n");
leunberger(A,B,C,F,K,P_L,t3,r3,umax,x0_true, "Images/obs_zero_init_err_r3");
fprintf("Simulating closed loop with observer, zero error for r4(t)...\n");
leunberger(A,B,C,F,K,P_L,t4,r4,umax,x0_true, "Images/obs_zero_init_err_r4");

% Simulate with non-zero initial error
fprintf("Simulating closed loop with observer, nonzero error for r1(t)...\n");
leunberger(A,B,C,F,K,P_L,t1,r1,umax,x0_guess, "Images/obs_nonzero_init_err_r1");
fprintf("Simulating closed loop with observer, nonzero error for r2(t)...\n");
leunberger(A,B,C,F,K,P_L,t2,r2,umax,x0_guess, "Images/obs_nonzero_init_err_r2");
fprintf("Simulating closed loop with observer, nonzero error for r3(t)...\n");
leunberger(A,B,C,F,K,P_L,t3,r3,umax,x0_guess, "Images/obs_nonzero_init_err_r3");
fprintf("Simulating closed loop with observer, nonzero error for r4(t)...\n");
leunberger(A,B,C,F,K,P_L,t4,r4,umax,x0_guess, "Images/obs_nonzero_init_err_r4");


%% LQR Optimal State Feedback WITHOUT Integral control and WITHOUT Observer
% fprintf("Simulating LQR without integral and without observer...\n");
% t = t';
% r = [r_zero r_MO2 r_zero];
% 
% 
% n = size(A,1); %#nominal states
% m = size(B,2); %#nominal inputs
% p = size(C,1); %#nominal outputs
% 
% %Define closed-loop plant poles via LQR (only vary awts for now)
% awts = [ones(1,n/2)*100, ones(1,n/2)]; %initial design: relative penalties
% bwts = ones(1,p);
% rho = 1;
% 
% awts = awts./sum(awts);
% xmax = [10 10 10 .1 .1 .1];
% Q =  diag(awts./xmax); 
% R =  rho*diag(bwts./umax); %rho * eye(p);
% 
% OLsys = ss(A,B,C,D);
% [K,W,clEvals] = lqr(OLsys,Q,R); %get optimal K, W
% 
% F = inv(C/(-A+B*K)*B);
% 
% %CL system
% Acl = A - B*K;
% Bcl = B*F;
% Ccl = C;
% Dcl = D;
% CLsys = ss(Acl,Bcl,Ccl,Dcl);
% 
% %%Check that closed-loop system specs met; change rho o'wise
% %%get response to first reference input profile:
% [Y_CL1,~,X_CL1] = lsim(CLsys,r',t');
% %compute resulting actuator efforts in each case, where u = -Kx + Fr
% U_CL1 = -K*X_CL1' + F*r';
% 
% % Plot states
% plot_states(t,X_CL1,r,"Images/LQR");
% %%Plot output response for each input/output channel; compare to desired
% %%reference positions
% plot_outputs(t,Y_CL1,r,"Images/LQR");
% %%Plot actuator efforts and compare to constraints on u
% plot_controls(t,U_CL1,umax,"Images/LQR");

%% LQR WITH integral control and observer

n = size(A,1); %#nominal states
m = size(B,2); %#nominal inputs
p = size(C,1); %#nominal outputs

Aaug = [A zeros(6,3); -C zeros(3,3)];
Baug = [B; zeros(3,3)];
Caug = [C zeros(3,3)];
Daug = zeros(size(Caug,1),size(Baug,2));
augOLsys = ss(Aaug,Baug,Caug,Daug);

%Define closed-loop plant poles via LQR (only vary awts for now)
awts = [ones(1,n/2)*100, 250*ones(1,n/2),50*ones(1,n/2)]; %initial design: relative penalties
bwts = [1,1.5,1];
rho = 3000;

awts = awts./sum(awts);
bwts = bwts./sum(bwts);
xmax = [10*ones(1,n/2), 1/1000*ones(1,n/2), 60^2*ones(1,n/2)];
Qaug =  diag(awts./xmax); 
Raug =  rho*diag(bwts./umax); %rho * eye(p);

[Kaug,Waug,clEvalsAug] = lqr(augOLsys,Qaug,Raug);

% TODO: I think we need to change these?
% XCLO_IC = [x0_guess;0*ones(3,1)];
XCLO_IC = [0*ones(3,1);x0_guess(4:end);0*ones(3,1)];
% XCLO_IC = 0*ones(15,1);
% XCLO_IC = 0.1*ones(15,1); %change the IC to 0.1's and see what happens!!
fprintf("Simulating LQR with integral and with observer for r1(t)...\n");
[CLaugsys,Y_CLOaug,U_CLOaug,Faug] = simLQR(sys_OL,augOLsys,Kaug,P_L,...
    t1,r1,XCLO_IC,umax,'Images/LQR_r1');

fprintf("Simulating LQR with integral and with observer for r2(t)...\n");
[CLaugsys,Y_CLOaug,U_CLOaug,Faug] = simLQR(sys_OL,augOLsys,Kaug,P_L,...
    t2,r2,XCLO_IC,umax,'Images/LQR_r2');

fprintf("Simulating LQR with integral and with observer for r3(t)...\n");
[CLaugsys,Y_CLOaug,U_CLOaug,Faug] = simLQR(sys_OL,augOLsys,Kaug,P_L,...
    t3,r3,XCLO_IC,umax,'Images/LQR_r3');

fprintf("Simulating LQR with integral and with observer for r4(t)...\n");
[CLaugsys,Y_CLOaug,U_CLOaug,Faug] = simLQR(sys_OL,augOLsys,Kaug,P_L,...
    t4,r4,XCLO_IC,umax,'Images/LQR_r4');

