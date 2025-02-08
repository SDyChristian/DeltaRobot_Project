%% This algorithm plots robot workspace

% Author: Christian Trejo SD&C
% How it works: The algorithm creates incremental circumferences along Z direction 
% and one point of the circumference is sent to InverseKin function. 
% If the point is out of workspace (inverseKin returns zeros) then no more circumferences are ploted.

% Initialization 
theta=1;
r=0;

figure
hold on
grid on

t = 0 : 0.01 : 2*pi; 

for z = -0.2 : -.005 : -0.8 
    % While theta is different to zero then radius is incremented
    while theta ~= 0
        r= r+ 0.001;  % radius incremented  
        x0 = r*cos(t);
        y0 = r*sin(t);
        z0 = z*ones(1,length(t));
        theta = InverseKin(x0(1),y0(1),z0(1));
    end
    % Plot last circumference reached
    plot3(x0,y0,z0,'bo','Markersize',1)
    r=0;
    theta=1;
end
xlabel('x [meters]')
ylabel('y [meters]')
zlabel('z [meters]')
title('Delta Robot Workspace')