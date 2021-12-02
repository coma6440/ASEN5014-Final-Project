function [OL_poles,y1] = OL_Response(A,sys_OL,r_step,r_zero,r_piece,r_MO1,r_MO2,r_MO3,r_MO3_zeros,t_r_MO3,t,x0);
    %% Determining OL Response 
    OL_poles = eig(A);
% 
%     rmag = 1;
%     r = rmag*sign(double(t>1));
    % all step inputs
    r1 = [r_zero r_MO2 r_zero]; 
    [y1,t,x1] = lsim(sys_OL,r1,t,x0);

    x_r1 = y1(:,1);
    y_r1 = y1(:,2);
    z_r1 = y1(:,3);

    % Plotting
    figure(1)
    h = sgtitle('Open Loop Response');
    set(h,'FontSize',20)
    subplot(3,1,1)
    plot(t,x_r1, 'linewidth', 1)
    hold on 
    plot(t,r1(:,1), 'linewidth', 2)
    ylabel('x (m)')
    xlabel('Time (s)')
    title('x vs u_x')
    set(gca,'FontSize',15)
    grid on 
    legend('x(t)','Reference')

    subplot(3,1,2)
    plot(t,y_r1, 'linewidth', 1)
    hold on 
    plot(t,r1(:,2), 'linewidth', 1)
    ylabel('y (m)')
    xlabel('Time (s)')
    title('y vs u_y')
    set(gca,'FontSize',15)
    grid on 
    legend('y(t)','Reference')

    subplot(3,1,3)
    plot(t,z_r1, 'linewidth', 1)
    hold on 
    plot(t,r1(:,3), 'linewidth', 2)
    ylabel('z (m)')
    xlabel('Time (s)')
    title('z vs u_z')
    set(gca,'FontSize',15)
    grid on 
    legend('z(t)','Reference')

    %% Simulating Plant Response 



end

