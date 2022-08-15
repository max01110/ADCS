function dstatedt = Satellite(t,state)
%%genericState = [x0;y0;z0;xdot0;ydot0;zdot0;q0123_0;p0;q0;r0];
global m invI I BxI ByI BzI BxB ByB BzB

%%select states
x = state(1);
y = state(2);
z = state(3);
q0123 = state(7:10);
ptp = Quat2Eu(q0123)';
p = state(11);
q = state(12);
r = state(13);
pqr = state(11:13);


%%%Translational Kinematics
vel = state(4:6); %%Velocity

%%%Rotational Kinematics
%Derivative of Quaternions
PQRMAT = [0 -p -q -r; p 0 r -q; q -r 0 p; r q -p 0];
q0123dot = 0.5*PQRMAT*q0123;

%%%Gravity Model (Newton's law universal gravitation)
%%Load earth parameters
planet
r_xyz = state(1:3); %% r = [x;y;z]
rho = norm(r_xyz);
rhat = r_xyz/rho;
Fgrav = -(G*M*m/rho^2)*rhat;

%%%Call the Magnetic Field
%%%Convert Cartesian XYZ into Lat, Long, Alt
phiE=0;
thetaE=acos(z/rho);
psiE=atan2(y,x);
latitude=90-thetaE*180/pi;
longitude=psiE*180/pi;
rhoKM=(rho)/1000;
%%rhoKM=(rho)/1000;
[BN, BE, BD] = igrf('01-Jan-2020',latitude,longitude,rhoKM,'geocentric');

%%% Inertial Frame
BNED=[BN; BE; BD];
BI=TIB(phiE,thetaE+pi,psiE)*BNED;
BxI=BI(1);
ByI=BI(2);
BzI=BI(3);
%%% Just need to also check if we need transformation too instead of only
%%% rotation
BB = TIBquat(q0123)'*BI;
BxB=BB(1);
ByB=BB(2);
BzB=BB(3);

%%%Translational Dynamics
F = Fgrav;
accel = F/m;

%%%Rotational Dynamics
%%%Total External Disturbance Moments
LMN = [0;0;0];
H = I*pqr;
pqrdot = invI*(LMN - cross(pqr,H));

%%%Return Derivatives State
dstatedt = [vel;accel;q0123dot;pqrdot];

%%%Discretization (check this)
%dt = 1; 
%dstatedt_dis = state + dstatedt*dt;
end