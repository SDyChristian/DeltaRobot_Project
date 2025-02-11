function theta = InverseKin(x0,y0,z0)

% Author: Christian Trejo SD&C
% This function receives end effector positions (x0,y0,z0) and delivers angular positions (thetaX) 
rf = 0.204;   % Link length attached to actuator
re = 0.575;   % Length of passive link
f  = 0.110;   % Length of the upper triangle (base)
e  = 0.055;   % Length of the bottom triangle (end effector)

% Parameters of circle equations
yF1 = -f;
zE1 = z0;
yE1 = y0 - e;
r12  = rf^2;               % radius 1 squared
r22  = -x0.^2 + re^2;      % radius 2 squared

%%%%      Variables zJ1 y yJ1      %%%%
a = (r12 - r22 - yF1^2 + yE1.^2 + zE1.^2 ) ./ (2*zE1) ;
b = (yF1-yE1) ./ zE1;

% Discriminant
D=-(a+b.*yF1)*(a+b.*yF1)+rf*(b.*b*rf+rf);
 
    if D <= 0
        theta=0;
    else
        yJ1 = (yF1-a.*b-sqrt(D))./((b.^2 + 1));
        zJ1 = a + b.*yJ1;
        theta = atan2d(-zJ1,(yF1-yJ1)); 
    end
end