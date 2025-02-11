% Author: Christian Trejo from SD&C
% This algorithm allows delta robot to follow a path extracted from an image
% Such image should be in the path folder and must be specified in closed_path file
% It requires robot hardware and USB2Dynamixel device
% Important: Port COM should be specified depending on your computer assignation
% For this case COM port is 52

Dynamixel = MX64(52,57142); % Communication enabled

% Setting Speed (It can takes values from 1 up to 1500)
% For user safey, it is recommended to not exceed 500 
Dynamixel.setSpeed(1, 100)
Dynamixel.setSpeed(2, 100)
Dynamixel.setSpeed(3, 100)

% Extract trajectory from image
Zmin = -0.61;
Zmax = -0.51;
[Tray] = closed_path(Zmin,Zmax);
for a=1 : 1 : length(Tray(1,:))
    % Fe vales and offset are defined to adjust image position and scale w.t.r. robot camera
    Fe = 0.0008; % Scale factor of im 
    x_off = -0.055; y_off = 0.14; 
    x0 = Tray(1,a)*Fe + x_off;
    y0 = Tray(2,a)*Fe + y_off;
    z0 = Tray(3,a);
    
    %% Inverse Kinematics
    % Rotation of 120° around Z
    x1 = x0*cosd(120) + y0*sind(120);
    y1 =-x0*sind(120) + y0*cosd(120);
    % Rotation of -120° around Z
    x2 = x0*cosd(120) - y0*sind(120);
    y2 = x0*sind(120) + y0*cosd(120);
     
    % Compute angles
    theta1 = InverseKin(x0,y0,z0);
    theta2 = InverseKin(x1,y1,z0);
    theta3 = InverseKin(x2,y2,z0);
    
    %% Send angles to motors 
    % Conversion from angles to motor positions and offsets
    A1 = -floor(theta1*(4095/360))+1024;
    A2 = -floor(theta2*(4095/360))+2048;
    A3 = -floor(theta3*(4095/360))+2048;
    Dynamixel.position(1,A1);
    Dynamixel.position(2,A2);
    Dynamixel.position(3,A3);
end

%% Home position
    pause(1)
    Dynamixel.setSpeed(1, 30)
    Dynamixel.setSpeed(2, 30)
    Dynamixel.setSpeed(3, 30)
    Dynamixel.position(1,1536);
    Dynamixel.position(2,2560);
    Dynamixel.position(3,2560);
    clear Dynamixel