function [x0,y0,z0] = ForwardKin(theta1,theta2,theta3)

% Author: Christian Trejo SD&C
% This function receives angular positions (thetaX) and delivers end effector positions (x0,y0,z0) 
rf = 0.204;   % Link length attached to actuator
re = 0.575;   % Length of passive link
f  = 0.110;   % Length of the upper triangle (base)
e  = 0.055;   % Length of the bottom triangle (end effector)

u = (f-e);
% Sphere coordinates J'1
y1 = -(u + rf*cosd(theta1));
z1 = -rf*sind(theta1);

% Sphere coordinates J'2
y2 = (u + rf*cosd(theta2))*sind(30);
x2 = (u + rf*cosd(theta2))*cosd(30);
z2 = -rf*sind(theta2);

% Sphere coordinates J'3
y3 = (u + rf*cosd(theta3))*sind(30);
x3 =-(u + rf*cosd(theta3))*cosd(30);
z3 = -rf*sind(theta3);

% Denominator
d = (y2-y1).*x3-(y3-y1).*x2;

Auxiliary variables
w1 = y1.^2 + z1.^2;
w2 = x2.^2 + y2.^2 + z2.^2;
w3 = x3.^2 + y3.^2 + z3.^2;

% x = (a1*z + b1)
a1 = (z2-z1).*(y3-y1)-(z3-z1).*(y2-y1);
b1 = -((w2-w1).*(y3-y1)-(w3-w1).*(y2-y1))/2;
% y = (a2*z + b2)
a2 = -(z2-z1).*x3+(z3-z1).*x2;
b2 = ((w2-w1).*x3 - (w3-w1).*x2)/2;

%a*z^2 + b*z + c = 0
 a = a1.^2 + a2.^2 + d.^2;
 b = 2*(a1.*b1 + a2.*(b2-y1.*d)-z1.*d.^2);
 c = (b2-y1.*d).^2 + b1.^2 + d.^2.*(z1.^2 - re^2);
 
 %Discriminant D
 D = b.^2 - 4*a.*c;
 if D <= 0 % unreached positions
        x0=0;
        y0=0;
        z0=0;
 else
        z0 = (-b-sqrt(D))./(2*a) ;
        x0 = (a1.*z0 + b1)./d;
        y0 = (a2.*z0 + b2)./d;
 end
end

