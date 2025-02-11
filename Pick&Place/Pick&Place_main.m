% Author: Christian Trejo SD&C
% This algorithm performs pick & place application by detecting objects with a camera 
% and takes them using an electromagnet
% It requires robot hardware and USB2Dynamixel device
% Important: Port COM should be specified depending on your computer assignation
% For this case COM port is 52
% A serial port is needed to control electromagnet through an Arduino

Dynamixel = MX64(52,57142); % Communication enabled

% Setting Speed (It can takes values from 1 up to 1500)
% For user safey, it is recommended to not exceed 500 
Dynamixel.setSpeed(1, 60)
Dynamixel.setSpeed(2, 60)
Dynamixel.setSpeed(3, 60)

% Robot is placed in a position that does not obstruct camera's field of view
Dynamixel.position(1,760);
Dynamixel.position(2,2870); 
Dynamixel.position(3,1848);
pause(5); % Wait 5 seconds

%% Objects Detection
% Objects_Detection function enables camera, performs image processing to detect objects
% and returns two sets of vectors (px,py) that contain objects centroid coordinates   
[px,py] = Objects_Detection
py=py-20; px=px+20;  % Offset 

% ID is a vector that defines which kind of object was detected
% In this case all objects are assumed to be of the same kind (all corresponding to 1 value) 
% but other labels could be assiged (2,3,4,...,N) 
ID=ones(1,length(px)); 

%% Camera calibration
z = -0.675; % all objects share same z-position
% and a vector of all z-positions is created
pz = ones(1,length(px))*z;  

% Equivalences between workspace and image 
 % for X-axis ---- 0.865 m = 640 pixels
 % for Y-axis ---- 0.635 m = 480 pixels
 % Then proportion is C = 0.00135156
 C = 0.00135156;
% then pixels are transformed to meters using C value
 px=px*C; py=py*C; 

% Workspace and camera coordinates are aligned
OffsetX = -0.432; % meters
OffsetY = -0.324; % meters
px=px+OffsetX; py=py+OffsetY;
% and oriented
py=-py; px=-px;
z=pz; x=py; y=px; 

% PickPlace_Traj function receives object ID and position and returns the trajectory 
% needed to take object and carry it to the container
% E contains flag for electromagnet activation or deactivation
[X, Y, Z, E]=PickPlace_Traj(ID, x, y, z);

%% Electomagnet configuration
% A port for Arduino is initialized
delete(instrfind( {'Port'},{'COM59'} ) );
Serial_port = serial('COM59');
Serial_port.BaudRate = 115200;
warning('off' , 'Matlab:serial:fscanf:unsuccessfulRead');
fopen(Serial_port);

%%% It is important to first write in port to be able to read it (It is a matlab bug) %%%
fscanf (Serial_port,'d');

%% Pick&Place sequence
for i=1 : length(X)
    
    x0=X(i); y0=Y(i); z0=Z(i); % read trajectory values
    
    % "isMoving" command returns 1 if motoris in motion and 0 if it is static.
    % B1, B2 y B3 are flags that inidcates motor state 
    B1= Dynamixel.isMoving(1);
    B2= Dynamixel.isMoving(2);
    B3= Dynamixel.isMoving(3);
    
    % Pick&place sequence is "stopped" until motors reach the last position requested
    while B1==1 || B2==1 || B3==1
        B1= Dynamixel.isMoving(1);
        B2= Dynamixel.isMoving(2);
        B3= Dynamixel.isMoving(3);
        
    end
    
    %%% Inverse Kinematics
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
    
    % Once z-position is reached P&P sequence is "stopped" by two seconds
    if z0==z 
        pause(2);
    end
    fwrite(Serial_port,E(i),'uint8'); % Then, electromaget command is sent to Arduino  
end
% Close ports
fclose(Serial_port);
delete(Serial_port);
delete(Dynamixel)
clear Dynamixel