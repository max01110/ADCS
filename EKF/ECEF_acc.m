function [a_ECEF] = ECEF_acc(t,a_ECI,v_ECI,r_ECI)
% Compute acceleration of body w.r.t ECEF frame resolved in ECEF axes

omega_ie = 7.292115E-5;  % Earth rotation rate (rad/s)
% Calculate ECI to ECEF coordinate transformation matrix using (2.145)
C_i_e = [cos(omega_ie * t), sin(omega_ie * t), 0;...
        -sin(omega_ie * t), cos(omega_ie * t), 0;...
                         0,                 0, 1];
OMEGA_ie = skew([0;0;omega_ie]);
a_ECEF = C_i_e * (a_ECI - 2*OMEGA_ie*v_ECI + OMEGA_ie*OMEGA_ie*r_ECI);
end

