function [OL_poles,y1] = OL_Response(A,sys_OL,r_step,r_zero,r_piece,r_MO1,r_MO2,r_MO3,r_MO3_zeros,t_r_MO3,t,x0, fname);
    %% Determining OL Response 
    OL_poles = eig(A);
% 
%     rmag = 1;
%     r = rmag*sign(double(t>1));
    % all step inputs
    r1 = [r_MO3_zeros r_MO3_zeros r_MO3];
%     [y1,t,x1] = lsim(sys_OL,r1,t,x0);
    [y1,t,x1] = lsim(sys_OL,r1,t_r_MO3,x0);

    % Plotting
    plot_states(t,x1,r1,fname);
    plot_outputs(t,y1,r1,fname);
end

