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