function [CLaugsys,Y_CLOaug,U_CLOaug,Faug] = simLQR(sys,augsys,Kaug,...
    L_poles,t,r,x0,umax,fname)

[A,B,C,~] = ssdata(sys);
[Aaug,Baug,~,~] = ssdata(augsys);
L = place(A', C', L_poles)';

%%Define observer error augmented closed-loop dynamics with integral states 
%%(15 states total: 9 from LQR-Integral + 6 from observer errors)
Faug = [zeros(size(B)); 
          eye(3);
         zeros(6,3)]; %augment for observer error states
AaugCLO = [(Aaug - Baug*Kaug), Baug*Kaug(:,1:6); %ignore Ki terms
           zeros(6,9), A-L*C]; %plug in observer error dynamics
BaugCLO = Faug;
CaugCLO = [C zeros(3,9)];            
DaugCLO = zeros(size(CaugCLO,1),size(BaugCLO,2));
CLaugsys = ss(AaugCLO,BaugCLO,CaugCLO,DaugCLO);

%simulate system response
[Y_CLOaug,~,X_CLOaug] = lsim(CLaugsys,r',t',x0);
%%compute actuator efforts in each case, where u_claug = -Kaug*xaug
U_CLOaug = -[Kaug, Kaug(:,1:6)]*X_CLOaug';

%% Plot states
plot_states(t,X_CLOaug,r,fname)

%% Plot output response for each input/output channel; compare to desired
%%reference positions
plot_outputs(t,Y_CLOaug,r,fname)

%% Plot actuator efforts and compare to constraints on u
plot_controls(t,U_CLOaug,umax,fname)

%% CL Poles
f = figure('Visible', 'off');
pzmap(CLaugsys,'r');
grid on
title("")
saveas(f, fname + "_pzmap.png");

end