function [r_step,r_zero,r_piece,r_MO1,r_MO2,r_MO3,t_r_MO3,r_MO3_zeros,r_y_neg_10] = r_t_generator(rmag,t,dt,x0,period)
%%    
% Basic step desired
    r_step = zeros(1,length(t));
    r_step(t>=period/10) = rmag;
    r_step = r_step';
    % Zero desired 
    r_zero = zeros(1,length(t))';

    % Piecewise desired
    r_piece = zeros(length(t),1); 
    r_piece(t>=period/10) = rmag;
    r_piece(t>=period/5) = 0;

    
    % Mission Obj 1
    MO2_mag = 2;
    r_MO1 = zeros(length(t),1); 
    r_MO1(t>=period/10) = MO2_mag*rmag;
    r_MO1(t>=period/1.5) = 0;
    
    % Mission Obj 2
    ramp_time = period/2;
    M02_mag = 9;
    r_MO2 = zeros(1,length(t));
    r_MO2(t>200) = rmag*M02_mag/ramp_time*t(200/dt:end-2)-t(200/dt)*rmag*M02_mag/ramp_time;
    r_MO2(t>200+ramp_time/rmag) = rmag*M02_mag;
    r_MO2 = r_MO2'+x0(2);
    
    %%
    
    % Mission Obj 3
    dur = period/4;
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





