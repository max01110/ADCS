function [sys,x0,str,ts] = s_function(t,x,u,flag)
% S-function sf_aerodyn.M
% This S-function represents the nonlinear aircraft dynamics

% Copyright 1986-2007 The MathWorks, Inc. 


switch flag,

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0,
    [sys,x0,str,ts]=mdlInitializeSizes;

  %%%%%%%%%%%%%%%
  % Derivatives %
  %%%%%%%%%%%%%%%
  case 1,
    sys=mdlDerivatives(t,x,u);

  %%%%%%%%%%
  % Update %
  %%%%%%%%%%
  case 2,
    sys=mdlUpdate(t,x,u);

  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  case 3,
    sys=mdlOutputs(t,x,u);

  %%%%%%%%%%%%%%%%%%%%%%%
  % GetTimeOfNextVarHit %
  %%%%%%%%%%%%%%%%%%%%%%%
  case 4,
    sys=mdlGetTimeOfNextVarHit(t,x,u);

  %%%%%%%%%%%%%
  % Terminate %
  %%%%%%%%%%%%%
  case 9,
    sys=mdlTerminate(t,x,u);

  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
    otherwise
        ctrlMsgUtils.error('Controllib:general:UnexpectedError',['Unhandled flag = ',num2str(flag)]);

end

% end sfuntmpl


%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,x0,str,ts]=mdlInitializeSizes

%
% call simsizes for a sizes structure, fill it in and convert it to a
% sizes array.
%
% Note that in this example, the values are hard coded.  This is not a
% recommended practice as the characteristics of the block are typically
% defined by the S-function parameters.
%
sizes = simsizes;

sizes.NumContStates  = 13;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 13;
sizes.NumInputs      = 0;
%sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);

%%%% This will define all of our planet parameters

R = 6.371e6; %%radius of earth inmeters
M = 5.972e24; %%mass of earth kg
G = 6.67e-11; %%%universal gravitational constant in SI units
mu = G*M; %%gravitational constant with respect to the main body 

%%%Moments of Inertia Cubesat
Ix = 0.9;
Iy = 0.9;
Iz = 0.3;
I = [Ix,0,0;0,Iy,0;0,0,Iz]; %%kg-m^2
invI = inv(I);

%
%%%Initial Conditions Position and Velocity
altitude = 600*1000; %%meters
x_0 = R + altitude;
y_0 = 0;
z_0 = 0; 
xdot0 = 0;
inclination = deg2rad(56);
semi_major = norm([x_0;y_0;z_0]); %%Orbit Radius for circular orbit
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
x0 = [x_0;y_0;z_0;xdot0;ydot0;zdot0;q0123_0;p0;q0;r0];

%
% str is always an empty matrix
%
str = [];

%
% initialize the array of sample times
%
ts  = [0 0];

% end mdlInitializeSizes

%
%=============================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%=============================================================================
%
function sys=mdlDerivatives(t,x,u)
%%select states
q0123 = x(7:10);
ptp = Quat2Eu(q0123)';
p = x(11);
q = x(12);
r = x(13);
pqr = x(11:13);
m = 2.6; %%mass in kilograms

%%%Moments of Inertia Cubesat
Ix = 0.9;
Iy = 0.9;
Iz = 0.3;
I = [Ix,0,0;0,Iy,0;0,0,Iz]; %%kg-m^2
invI = inv(I);



%%%Translational Kinematics
vel = x(4:6); %%Velocity

%%%Rotational Kinematics
%Derivative of Quaternions
PQRMAT = [0 -p -q -r; p 0 r -q; q -r 0 p; r q -p 0];
q0123dot = 0.5*PQRMAT*q0123;

%%%Gravity Model (Newton's law universal gravitation)
%%Load earth parameters
planet
r_xyz = x(1:3); %% r = [x;y;z]
rho = norm(r_xyz);
rhat = r_xyz/rho;
Fgrav = -(G*M*m/rho^2)*rhat;

%{
%%%Call the Magnetic Field
%%%Convert Cartesian XYZ into Lat, Long, Alt
phiE=0;
thetaE=acos(z/rho);
psiE=atan2(y,x);
latitude=90-thetaE*180/pi;
longitude=psiE*180/pi;
rhoKM=(rho+R)/1000;
[BN, BE, BD] = igrf('01-Jan-2020',latitude,longitude,rhoKM,'geocentric');

%%% Inertial Frame
BNED=[BN; BE; -BD];
BI=TIB(phiE,thetaE+pi,psiE)*BNED;
BxI=BI(1);
ByI=BI(2);
BzI=BI(3);
%}

%%%Translational Dynamics
F = Fgrav;
accel = F/m;

%%%Rotational Dynamics
%%%Total External Disturbance Moments
LMN = [0;0;0];
H = I*pqr;
pqrdot = invI*(LMN - cross(pqr,H));

%%%Return Derivatives State
sys = [vel;accel;q0123dot;pqrdot];

% end mdlDerivatives

%
%=============================================================================
% mdlUpdate
% Handle discrete state updates, sample time hits, and major time step
% requirements.
%=============================================================================
%
function sys=mdlUpdate(t,x,u)

sys = [];

% end mdlUpdate

%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(t,x,u)

sys = x;

% end mdlOutputs

%
%=============================================================================
% mdlGetTimeOfNextVarHit
% Return the time of the next hit for this block.  Note that the result is
% absolute time.  Note that this function is only used when you specify a
% variable discrete-time sample time [-2 0] in the sample time array in
% mdlInitializeSizes.
%=============================================================================
%
function sys=mdlGetTimeOfNextVarHit(t,x,u)

sampleTime = 0.1;    %  Example, set the next hit to be one second later.
sys = t + sampleTime;

% end mdlGetTimeOfNextVarHit

%
%=============================================================================
% mdlTerminate
% Perform any end of simulation tasks.
%=============================================================================
%
function sys=mdlTerminate(t,x,u)

sys = [];

% end mdlTerminate