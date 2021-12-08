function [sys_CL,K,F] = CL_Ref_Track_Cont(A,B,C,D,CL_poles,t,r,umax,x0,fname)
    K = place(A,B,CL_poles);
    F = inv(C*inv(-A+B*K)*B);

    ACL = A-B*K;
    BCL = B*F;
    CCL = C;
    DCL = D;

    sys_CL = ss(ACL,BCL,CCL,DCL);

    [y1,t,x1] = lsim(sys_CL,r,t,x0);
    
    for i = 1:length(r)
        u(:,i) = -K*x1(i,:)'+F*r(i,:)';
    end
    
%     %% Delete Later (using to tune without saving figures to save time)
%     subplot(3,2,1)
%     plot(t,y1(:,1))
%     hold on 
%     plot(t,r(:,1))
%     
%     subplot(3,2,3)
%     plot(t,y1(:,2))
%     hold on 
%     plot(t,r(:,2))
% 
%     subplot(3,2,5)
%     plot(t,y1(:,3))
%     hold on 
%     plot(t,r(:,3))
%     
%     subplot(3,2,2)
%     plot(t,u(1,:)*1000)
%     
%     subplot(3,2,4)
%     plot(t,u(2,:)*1000)
% 
%     subplot(3,2,6)
%     plot(t,u(3,:)*1000)
% 
%     

    
    plot_states(t,x1,r,fname);
    plot_controls(t,u,umax,fname);
    plot_outputs(t,y1,r,fname);
end

