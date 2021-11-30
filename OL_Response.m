function [OL_poles] = OL_Response(A,sys_OL)
    %% Determining OL Response 
    OL_poles = eig(A);

    t = 0:0.001:50;
    rmag = 1;
    r = rmag*sign(double(t>1));
    % all step inputs
    r1 = [r(:),r(:),r(:)]; 
    [y1,t,x1] = lsim(sys_OL,r1,t);

    x_r1 = y1(:,1);
    y_r1 = y1(:,2);
    z_r1 = y1(:,3);

    % Plotting
    figure(1)
    h = sgtitle('Open Loop Response');
    set(h,'FontSize',20)
    subplot(3,1,1)
    plot(t,x_r1, 'linewidth', 3)
    hold on 
    plot(t,r1(:,1))
    ylabel('x (m)')
    xlabel('Time (s)')
    title('x vs u_x')
    set(gca,'FontSize',15)
    grid on 
    legend('x(t)','Reference')

    subplot(3,1,2)
    plot(t,x_r1, 'linewidth', 3)
    hold on 
    plot(t,r1(:,2))
    ylabel('y (m)')
    xlabel('Time (s)')
    title('y vs u_y')
    set(gca,'FontSize',15)
    grid on 
    legend('y(t)','Reference')

    subplot(3,1,3)
    plot(t,x_r1, 'linewidth', 3)
    hold on 
    plot(t,r1(:,3))
    ylabel('z (m)')
    xlabel('Time (s)')
    title('z vs u_z')
    set(gca,'FontSize',15)
    grid on 
    legend('z(t)','Reference')

    %% Simulating Plant Response 



end

