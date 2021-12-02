function [sys_CL,K,F] = CL_Ref_Track_Cont(A,B,C,D,CL_poles,t,r,x0)
    K = place(A,B,CL_poles);
    F = inv(C*inv(-A+B*K)*B);

    ACL = A-B*K;
    BCL = B*F;
    CCL = C;
    DCL = D;

    sys_CL = ss(ACL,BCL,CCL,DCL);

    [y1,t,x1] = lsim(sys_CL,r,t,x0);
    
    x_r1 = y1(:,1);
    y_r1 = y1(:,2);
    z_r1 = y1(:,3);
    
    for i = 1:length(r)
        u_act_r1(:,i) = -K*x1(i,:)'+F*r(i,:)';
    end
    
    ux = u_act_r1(1,:);
    uy = u_act_r1(2,:);
    uz = u_act_r1(3,:);
    
    figure()
    h = sgtitle('Closed Loop Response for RF Controller, F Tuned');
    set(h,'FontSize',20)
    subplot(3,2,1)
    plot(t,x_r1, 'linewidth', 2)
    hold on 
    plot(t,r(:,1), 'linewidth', 1)
    ylabel('x (m)')
    xlabel('Time (s)')
    title('x vs time for r(t)')
    set(gca,'FontSize',15)
    grid on 
    legend('x(t)','Reference')

    subplot(3,2,3)
    plot(t,y_r1, 'linewidth', 2)
    hold on 
    plot(t,r(:,2), 'linewidth', 1)
    ylabel('y (m)')
    xlabel('Time (s)')
    title('y vs time for r(t)')
    set(gca,'FontSize',15)
    grid on 
    legend('y(t)','Reference')

    subplot(3,2,5)
    plot(t,z_r1, 'linewidth', 2)
    hold on 
    plot(t,r(:,3), 'linewidth', 1)
    ylabel('z (m)')
    xlabel('Time (s)')
    title('z vs time for r(t)')
    set(gca,'FontSize',15)
    grid on 
    legend('z(t)','Reference')

    subplot(3,2,2)
    plot(t,ux, 'linewidth', 3)
    ylabel('ux (m/s)')
    xlabel('Time (s)')
    title('u_x vs time')
    set(gca,'FontSize',15)
    grid on 
    
    subplot(3,2,4)
    plot(t,uy, 'linewidth', 3)
    ylabel('uy (m/s)')
    xlabel('Time (s)')
    title('u_y vs time')
    set(gca,'FontSize',15)
    grid on     
    
    subplot(3,2,6)
    plot(t,uz, 'linewidth', 3)
    ylabel('uz (m/s)')
    xlabel('Time (s)')
    title('u_z vs time')
    set(gca,'FontSize',15)
    grid on 

end

