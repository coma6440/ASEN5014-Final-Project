function [ex, ey, ez, et] = control_effort(t,u)
ex = trapz(t, abs(u(1,:)));
ey = trapz(t, abs(u(2,:)));
ez = trapz(t, abs(u(3,:)));
et = ex + ey + ez;
fprintf("Control Effort: x = %f, y = %f, z = %f, total = %f\n", ex, ey, ez, et);
end