clear 
close all
%%%Call the Magnetic Field
%%%Convert Cartesian XYZ into Lat, Long, Alt
R = 6371000;
x = R+600*1000;
y=0;
z=0;
rho = norm([x;y;z]);
phiE=0;
thetaE=acos(z/rho);
psiE=atan2(y,x);
latitude=90-thetaE*180/pi;
longitude=psiE*180/pi;
rhoKM=(rho)/1000;
[BN, BE, BD] = igrf('01-Jan-2020',latitude,longitude,rhoKM,'geocentric');

%%% Inertial Frame
BNED=[BN; BE; BD];
BI=TIB(phiE,thetaE+pi,psiE)*BNED;
vpa(BI,6)