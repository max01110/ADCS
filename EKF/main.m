%%% Clear workspace
clear
clc
close all

%%%Globals
global BxI ByI BzI BxB ByB BzB

%%% Setup IGRF model

addpath 'igrf/'
addpath 'GNSSMaster/'

%%%Get Earth Parameters for orbit
planet

%%%Cubesat parameters
global m invI I
m = 2.6; %%mass in kilograms
inertia

%%%Initial Conditions Position and Velocity
altitude = 600*1000; %%meters
x0 = R + altitude;
y0 = 0;
z0 = 0; 
xdot0 = 0;
inclination = deg2rad(56);
semi_major = norm([x0;y0;z0]); %%Orbit Radius for circular orbit
vcircular = sqrt(mu/semi_major); %%Orbital Speed from Vis-Viva Equation with a=r
ydot0 = vcircular*cos(inclination);
zdot0 = vcircular*sin(inclination);

%%%Initial Conditions Attitude and Angular velocity
%Initialize Euler Angles
phi0 = 0;
theta0 = 0;
psi0 = 0;
ptp0 = [phi0;theta0;psi0];
q0123_0 = Eu2Quat(ptp0); %Transform to quat initial conditions

%Initial Angular Velocity (Body Frame)
p0=0.01;
q0=0;
r0=0;

%Initial Conditions state vector
stateinitial = [x0;y0;z0;xdot0;ydot0;zdot0;q0123_0;p0;q0;r0];

%%%Orbit time parameters Circular Orbit
period = 2*pi/sqrt(mu)*semi_major^(3/2); %%Tcircular
number_of_orbits = 1;
tfinal = period*number_of_orbits;
tspan = 0:1:tfinal;

%%%This is where we integrate the equations of motion
[tout,stateout] = ode45(@Satellite,tspan,stateinitial);

%%%Loop through stateout to extract Magnetic Field

BxIout=0*stateout(:,1);
ByIout=BxIout;
BzIout=BxIout;

% Instansiate IMU block
IMU = imuSensor('accel-gyro-mag');
fs = 1; % 1 sample per second
traj = kinematicTrajectory('SampleRate',fs);
for idx = 1:length(tout)
    dstatedt = Satellite(tout(idx),stateout(idx,:)');
    BxIout(idx)=BxI;
    ByIout(idx)=ByI;
    BzIout(idx)=BzI;
    lin_acc(1:3,idx)= dstatedt(4:6,1);
    BxBout(idx)=BxB;
    ByBout(idx)=ByB;
    BzBout(idx)=BzB;
    
    % Frame transformation from ECI to ECEF
    [r_ecef(1:3,idx),v_ecef(1:3,idx),T_EB] = ECI_to_ECEF(tout(idx),stateout(idx,1:3)',...
        stateout(idx,4:6)',TIBquat(stateout(idx,7:10)'));
    
    % Frame transformation from ECEF to NED local navigation frame
    [L_b(idx),lambda_b(idx),h_b(idx),v_ned(1:3,idx),T_nedB,C_e_n,...
        omega_en,C_ni] = ECEF_to_NED(r_ecef(1:3,idx),...
        v_ecef(1:3,idx),T_EB,tout(idx));
    
    % Compute Acceleration in ECEF frame
    a_ECEF(1:3,idx) = ECEF_acc(tout(idx),lin_acc(:,idx),stateout(idx,4:6)',stateout(idx,1:3)');
    
    % Compute Acceleration in NED frame
    a_NED(1:3,idx) = C_e_n*a_ECEF(:,idx);
    
    %%% Compute Acceleration w.r.t ECEF in body frame
    a_Body(1:3,idx) = T_nedB.' * (a_NED(1:3,idx)-[0;0;0]);
    
    % Compute angular velocity of body w.r.t NED resolved in NED
    % ECI inertial components of angular velocity
    ptp = Quat2Eu(stateout(idx,7:10)');
    H_mat = [1, sin(ptp(1))*tan(ptp(2)), cos(ptp(1))*tan(ptp(2));...
             0, cos(ptp(1)), -sin(ptp(1));...
             0,sin(ptp(1))/cos(ptp(2)), cos(ptp(1))/cos(ptp(2))];
    omega_ECI(1:3,idx) = H_mat*stateout(idx,11:13)';
    
    % Compute NED components of angular velocity
    omega_NAV(1:3,idx) = omega_NED(C_ni, omega_ECI(1:3,idx),omega_en);
    
    %%% Compute Acceleration w.r.t ECEF in body frame
    omega_Body(1:3,idx) = T_nedB.' * omega_NAV(1:3,idx);
    
    % Compute UVW
    T_mat = TIBquat(stateout(idx,7:10)').'; % Transponse as we need UVW
    UVW(1:3,idx) = T_mat * stateout(idx,4:6)';
    
    % Compute UVWdot
    pqr_mat = [0,-stateout(idx,13),stateout(idx,12);...
           stateout(idx,13),0,-stateout(idx,11);...
           -stateout(idx,12),stateout(idx,11),0];
    UVWdot(1:3,idx) = - pqr_mat*UVW(1:3,idx);
    
    % Orientation matrix Body to NED
    orientationNB(:,:,idx) = T_nedB.';    
    
end
%[~,orientationNED,~,accNED,angVelNED] = traj(a_Body',[omega_Body(3,:)',omega_Body(2,:)',omega_Body(1,:)']);
[~,orientationNED,~,accNED,angVelNED] = traj(a_Body',omega_Body');
[accelReading,gyroReading,magReading] = IMU(a_NED.',omega_NAV.',orientationNB);

%%%Convert state to kilometers
stateoutOrbit(:,1:6) = stateout(:,1:6)/1000;

%%%Extract the state vector
xout = stateoutOrbit(:,1);
yout = stateoutOrbit(:,2);
zout = stateoutOrbit(:,3);
q0123out = stateout(:,7:10);
ptpout = Quat2Eu(q0123out);
pqrout = stateout(:,11:13);
%%%Make an Earth
[X,Y,Z] = sphere(100);
X = X*R/1000;
Y = Y*R/1000;
Z = Z*R/1000;
%{
%%%Plot 3D orbit
fig = figure();
set(fig,'color','white')
plot3(xout,yout,zout,'r-','LineWidth',3)
xlabel('X')
ylabel('Y')
zlabel('Z')
grid on
hold on
earth_sphere()
axis equal

%%%Plot Quaternions
fig2 = figure();
set(fig2,'color','white')
plot(tout,q0123out,'-','LineWidth',2);
grid on
xlabel('Time (sec)')
ylabel('Quaternions')
legend('q0','q1','q2','q3')
xlim([0,2000])

%%%Plot Euler Angles
fig3 = figure();
set(fig3,'color','white')
plot(tout,ptpout*180/pi,'-','LineWidth',2);
grid on
xlabel('Time (sec)')
ylabel('Euler Angles (deg)')
legend('Phi','Theta','Psi')

%%%Plot Angular Velocity
fig4 = figure();
set(fig4,'color','white')
plot(tout,pqrout,'-','LineWidth',2);
grid on
xlabel('Time (sec)')
ylabel('Angular Velocity (rad/s)')
legend('p','q','r')


%%%Angular velocity in ECI inertial and NAV NED
figure
subplot(2,1,1)
plot(tout,angVelNED,'-','LineWidth',2);
grid on
xlabel('Time (sec)')
ylabel('Angular Velocity FUNC')
subplot(2,1,2)
plot(tout,omega_NAV,'-','LineWidth',2);
grid on
xlabel('Time (sec)')
ylabel('Angular Velocity OURS')

%%%plot the magnetic field
fig5=figure();
set(fig5,'color','white');
plot(tout,BxIout/1000,'b-','Linewidth',2);
hold on
grid on
plot(tout,ByIout/1000,'r-','Linewidth',2);
hold on
plot(tout,BzIout/1000,'g-','Linewidth',2);
xlabel('Time (sec)');
ylabel('Mag Field (uT)');
legend('x','y','z')
%xlim([0,2000])

%%%Magnetic Field Norm
Bnorm = sqrt(BxBout.^2 + ByBout.^2 + BzBout.^2);
fig3 = figure();
set(fig3,'color','white')
plot(tout,Bnorm,'LineWidth',2)
xlabel('Time (sec)')
ylabel('Norm of Magnetic Field (T)')
grid on

%%%plot the magnetic field
fig5=figure();
set(fig5,'color','white');
plot(tout,BxBout/1e9,'b-','Linewidth',2);
hold on
grid on
plot(tout,ByBout/1e9,'r-','Linewidth',2);
hold on
plot(tout,BzBout/1e9,'g-','Linewidth',2);
xlabel('Time (sec)');
ylabel('Mag Field (uT) in Body Frame');
legend('x','y','z')
%xlim([0,2000])


%%%plot linear acceleration ECI frame
fig6=figure();
set(fig6,'color','white');
plot(tout,lin_acc,'-','LineWidth',2)
grid on
xlabel('Time (sec)')
ylabel('Acc ECI')
legend('x','y','z')
%xlim([0,2000])

% Plot Position in ECI and ECEF
fig7 = figure();
set(fig7,'color','white');
subplot(2,1,1);
plot(tout,stateout(:,1:3)','-','LineWidth',2)
grid on
xlabel('Time (sec)')
ylabel('Position ECI')
legend('x','y','z')
% Plot position in ECEF
subplot(2,1,2);
plot(tout,r_ecef,'-','LineWidth',2)
grid on
xlabel('Time (sec)')
ylabel('Position ECEF')
legend('x','y','z')

% Plot Velocity in ECI and ECEF
fig8 = figure();
set(fig8,'color','white');
subplot(2,1,1);
plot(tout,stateout(:,4:6)','-','LineWidth',2)
grid on
xlabel('Time (sec)')
ylabel('Velocity ECI')
legend('x','y','z')
% Plot position in ECEF
subplot(2,1,2);
plot(tout,v_ecef,'-','LineWidth',2)
grid on
xlabel('Time (sec)')
ylabel('Velocity ECEF')
legend('x','y','z')

% Plot Velocity in NED
fig9 = figure();
set(fig9,'color','white');
plot(tout,v_ned,'-','LineWidth',2)
grid on
xlabel('Time (sec)')
ylabel('Velocity NED')
legend('x','y','z')

%%%plot linear acceleration ECEF frame
fig10=figure();
set(fig10,'color','white');
plot(tout,a_ECEF,'-','LineWidth',2)
grid on
xlabel('Time (sec)')
ylabel('Acc ECEF')
legend('x','y','z')
%xlim([0,2000])

%%%plot linear acceleration NED frame
fig11=figure();
set(fig11,'color','white');
plot(tout,a_NED,'-','LineWidth',2)
grid on
xlabel('Time (sec)')
ylabel('Acc OURS')
legend('x','y','z')
%xlim([0,2000])
%%%plot linear acceleration NED frame
fig11=figure();
set(fig11,'color','white');
plot(tout,accNED,'-','LineWidth',2)
grid on
xlabel('Time (sec)')
ylabel('Acc FUNCTION')
legend('x','y','z')
%xlim([0,2000])


%%%plot Velocity BODY frame
fig12=figure();
set(fig12,'color','white');
plot(tout,UVW,'-','LineWidth',2)
grid on
xlabel('Time (sec)')
ylabel('Velocity')
legend('U','V','W')
%xlim([0,2000])
%%%plot linear acceleration BODY frame
fig13=figure();
set(fig13,'color','white');
plot(tout,UVWdot,'-','LineWidth',2)
grid on
xlabel('Time (sec)')
ylabel('Acc Body')
legend('x','y','z')
%xlim([0,2000])
%}
figure
subplot(3,1,1)
plot(tout,accelReading)
legend('X-axis','Y-axis','Z-axis')
ylabel('Acceleration (m/s^2)')
title('Accelerometer Readings')

subplot(3,1,2)
plot(tout,gyroReading)
legend('X-axis','Y-axis','Z-axis')
title('Gyroscope Readings')
ylabel('Angular Velocity (rad/s)')

subplot(3,1,3)
plot(tout,magReading)
legend('X-axis','Y-axis','Z-axis')
ylabel('Magnetic Field (\muT)')
xlabel('Time (s)')
title('Magnetometer Readings')

figure
subplot(2,1,1)
plot(tout,accelReading)
legend('X-axis','Y-axis','Z-axis')
ylabel('Acceleration (m/s^2)')
title('Acc Readings IMU')

subplot(2,1,2)
plot(tout,a_Body)
legend('X-axis','Y-axis','Z-axis')
ylabel('Acceleration (m/s^2)')
title('Acc Readings MODEL')

figure
subplot(2,1,1)
plot(tout,gyroReading)
legend('X-axis','Y-axis','Z-axis')
title('Gyro Readings IMU')
ylabel('Angular Velocity (rad/s)')

subplot(2,1,2)
plot(tout,omega_Body)
legend('X-axis','Y-axis','Z-axis')
title('Gyro Readings MODEL')
ylabel('Angular Velocity (rad/s)')

figure
subplot(2,1,1)
plot(tout,magReading)
legend('X-axis','Y-axis','Z-axis')
ylabel('Mag Field (\muT)')
xlabel('Time (s)')
title('Mag Readings IMU')
subplot(2,1,2)
plot(tout,[BxBout;ByBout;BzBout]/1000)
legend('X-axis','Y-axis','Z-axis')
ylabel('Mag Field (\muT)')
xlabel('Time (s)')
title('Mag Readings MODEL')
