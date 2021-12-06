function [CLaugsys,Y_CLOaug,U_CLOaug,Faug] = simLQR(sys,augsys,Kaug,L_poles,t,r,x0,umax,fname)

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

%% Plot output response for each input/output channel; compare to desired
%%reference positions
figure()
sgtitle('Closed Loop Response for LQR Controller','FontSize',18)
    subplot(3,1,1), hold on; grid on
plot(t,Y_CLOaug(:,1),'b','linewidth',1.5)
plot(t,r(:,1),'k--','linewidth',1.5)
legend('y(t)','reference')
title('y_1 vs. u_1 for r(t)','FontSize',14)
xlabel('t (secs)','FontSize',14)
ylabel('displacement (km)','FontSize',14)
    subplot(3,1,2), hold on; grid on
plot(t,Y_CLOaug(:,2),'b','linewidth',1.5)
plot(t,r(:,2),'k--','linewidth',1.5)
legend('y(t)','reference')
title('y_2 vs. u_1 for r(t)','FontSize',14)
xlabel('t (secs)','FontSize',14)
ylabel('displacement (km)','FontSize',14)
    subplot(3,1,3), hold on; grid on
plot(t,Y_CLOaug(:,3),'b','linewidth',1.5)
plot(t,r(:,3),'k--','linewidth',1.5)
legend('y(t)','reference')
title('y_3 vs. u_1 for r(t)','FontSize',14)
xlabel('t (secs)','FontSize',14)
ylabel('displacement (km)','FontSize',14)
exportgraphics(gcf,[pwd fname '_output.png']);

%% Plot actuator efforts and compare to constraints on u
figure()
sgtitle('Closed Loop Control Input Response for LQR Controller','FontSize',18)
hold on; grid on
plot(t, U_CLOaug(1,:),'r','linewidth',1.5)
plot(t, U_CLOaug(2,:),'b','linewidth',1.5)
plot(t, U_CLOaug(3,:),'g','linewidth',1.5)
plot(t,umax*ones(size(t)),'k--')
plot(t,-umax*ones(size(t)),'k--')
xlabel('t (secs)','FontSize',14)
ylabel('u effort (km/s^2)','FontSize',14)
legend('u_1','u_2','u_3','u constraint')
exportgraphics(gcf,[pwd fname '_input.png']);

%% CL Poles
figure
pzmap(CLaugsys,'r');
grid on
exportgraphics(gcf,[pwd fname 'pzPlot.png']);

end