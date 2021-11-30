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

%% Reachability and Observability
P = ctrb(A,B);
O = obsv(A,C);
fprintf("Rank of controllability matrix = %d\n",rank(P));
fprintf("Rank of observability matrix = %d\n", rank(O));

%% Open Loop
[OL_poles] = OL_Response(A,sys_OL);

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