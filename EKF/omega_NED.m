function omega_NAV = omega_NED(C_ni, omega_ECI,omega_en)

% Rotation matrix from Inertial ECI to NED
omega_ie = 7.292115E-5;  % Earth rotation rate (rad/s)
omega_NAV = C_ni*(omega_ECI-[0;0;omega_ie]) - omega_en;

end

