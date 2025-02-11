function [X,Y,Z,E] = PickPlace_Traj(ID, x, y, z);

% Author: Christian Trejo SD&C

% This function receives object ID and position (x,y,x) and returns the trajectory 
% needed to take object and carry it to the container. Also a vector of flags for 
% electromagnet activation or deactivation is returned.

% In this function a set of containers can be defined. Coordinates of each container are specified in Cx vector.
% In this case, three containers are considered but more containers
% can be added (Cx)

% Coordinates of containers
C1 = [0.27 0.27 -.5]; C2 = [0.27 0.27 -.5]; C3 = [0.27 0.27 -.5];
Za = -.5; % Height for robot secure motion 

c=0; % auxiliary variable
X=[]; Y=[]; X=[];
for i=1 : length(x)
    % End effctor positioned above object 
    X(i+c) = x(i); Y(i+c) = y(i); Z(i+c) = Za; E(i+c)=1;
    % End effector takes objects
    X(i+c+1) = x(i); Y(i+c+1) = y(i); Z(i+c+1) = z(i); E(i+c+1)=1; 
    % End effector lift the object
    X(i+c+2) = x(i); Y(i+c+2) = y(i); Z(i+c+2) = Za; E(i+c+2)=1;
    
    % Go to container
    % Depending on ID object, a container position is assigned
    if ID(i) == 1
    X(i+c+3) = C1(1); Y(i+c+3) = C1(2); Z(i+c+3) = C1(3); E(i+c+3)=1;
    X(i+c+4) = C1(1); Y(i+c+4) = C1(2); Z(i+c+4) = C1(3); E(i+c+4)=0;
    end
    if ID(i) == 2
    X(i+c+3) = C2(1); Y(i+c+3) = C2(2); Z(i+c+3) = C2(3); E(i+c+3)=1;
    X(i+c+4) = C2(1); Y(i+c+4) = C2(2); Z(i+c+4) = C2(3); E(i+c+4)=0;
    end
    if ID(i) == 3
    X(i+c+3) = C3(1); Y(i+c+3) = C3(2); Z(i+c+3) = C3(3); E(i+c+3)=1;
    X(i+c+4) = C3(1); Y(i+c+4) = C3(2); Z(i+c+4) = C3(3); E(i+c+4)=0;
    end
    c=c+4;
end
end