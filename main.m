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