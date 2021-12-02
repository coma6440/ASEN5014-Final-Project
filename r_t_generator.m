function [r_step,r_zero,r_piece,r_MO1,r_MO2,r_MO3,t_r_MO3,r_MO3_zeros,r_y_neg_10] = r_t_generator(rmag,t,dt,x0)
    % Basic step desired
    r_step = zeros(1,length(t));
    r_step(t>=5) = rmag;
    r_step = r_step';
    % Zero desired 
    r_zero = zeros(1,length(t))';

    % Piecewise desired
    r_piece = zeros(length(t),1); 
    r_piece(t>=5) = rmag;
    r_piece(t>=35) = 0;

    % Mission Obj 1
    r_MO1 = zeros(length(t),1); 
    r_MO1(t>=5) = rmag;
    r_MO1(t>=35) = 0;

    % Mission Obj 2
    ramp_time = 20;
    r_MO2 = zeros(1,length(t));
    r_MO2(t>5) = rmag/ramp_time*t(5/dt:end-2)-t(5/dt)*rmag/ramp_time;
    r_MO2(t>5+ramp_time/rmag) = rmag;
    r_MO2 = r_MO2'+x0(2);
    
    % Mission Obj 3
    dur = 10;
    Tf = t(end);
    [r_MO3_2,t2] = gensig("square",dur,Tf);
    u = 2*r_MO3_2-rmag;
    zero_time = 1;

    r_MO3 = [zeros(1,zero_time*round(length(t)/length(u))) zeros(1,length(u))];
    r_MO3(zero_time*round(length(t)/length(u))+1:end) = u';
    r_MO3 = [r_MO3(1:end-1) zeros(1,zero_time*round(length(t)/length(u)))]';
   
    t_r_MO3 = 0:Tf/length(r_MO3):Tf-Tf/length(r_MO3);
    r_MO3_zeros = zeros(length(t_r_MO3),1);
    
    %Stay at y IC of -10 km 
    r_y_neg_10 = ones(length(t),1)*-10;

end





