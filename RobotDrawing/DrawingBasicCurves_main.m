% Author: Christian Trejo from SD&C
% This algorithm allows delta robot to follow basic curves as a circle or polar star
% It requires robot hardware and USB2Dynamixel device
% Important: Port COM should be specified depending on your computer assignation
% For this case COM port is 52

Dynamixel = MX64(52,57142); % Communication enabled

% Setting Speed (It can takes values from 1 up to 1500)
% For user safey, it is recommended to not exceed 500 
Dynamixel.setSpeed(1, 35)
Dynamixel.setSpeed(2, 35)
Dynamixel.setSpeed(3, 35)

i=0; % counter variable
     
for t=0 : .04 : 3*pi
    i=i+1;
    %% Trajectory
%%%%%%%%%%% Circle %%%%%%%%%%%%%%%%%%

    x0 = 0.3*cos(t);
    y0 = 0.3*sin(t);
    z0 = -0.4;

%%%%%%%%%%% Polar Star %%%%%%%%%%%%

%     xc = 0; yc = 0; k = 5; % Offsets and gain for frequency of sin function
%     r = sin(6*(k/7)*t); % radius
%     x0 = xc + (r*(sin(k*t)^3))*.09;
%     y0 = yc + (r*(cos(k*t)^3))*.09;
%     z0 = -.55;
    
%   x_d, y_d and z_d: these variables saves desired trajectory
    x_d(i)=x0; y_d(i)=y0; z_d(i)=z0;
    
    %% Inverse Kinematics
    % Rotation of 120° around Z
    x1 = x0*cosd(120) + y0*sind(120);
    y1 =-x0*sind(120) + y0*cosd(120);
    % Rotation of -120° around Z
    x2 = x0*cosd(120) - y0*sind(120);
    y2 = x0*sind(120) + y0*cosd(120);
     
    % Computes angles
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
    
    % Saving angles reached by motors
    q1(i) = Dynamixel.getPresentPosition(1);
    q2(i) = Dynamixel.getPresentPosition(2);
    q3(i) = Dynamixel.getPresentPosition(3);
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

%% Results
% Conversion from motor position to angle 
 q1=q1*(360/4096)-90;
 q2=q2*(360/4096)-180;
 q3=q3*(360/4096)-180;
 
% Plot of folowed trajectory  
[xx,yy,zz] = ForwardKin(-q1,-q2,-q3);
plot3(xx,yy,zz)
hold on 
plot3 (x_d,y_d,z_d,'r-','MarkerSize',5)
grid on
xlabel('X'); ylabel('Y') ; zlabel('Z')
legend('Followed Trajectory','Desired Trajectory')