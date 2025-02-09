% Author: Christian Trejo from SD&C
% This algorithm is used to set motor speed and write angular positions 

% Servomotor Initialization
Dynamixel = MX64(52,57142);

% Setting Speed (It can takes values from 1 up to 1500
% For user safey, it is recommended to not exceed 500 
Dynamixel.setSpeed(1,40)
Dynamixel.setSpeed(2,40)
Dynamixel.setSpeed(3,40)

% Write angular positions (de 0 hasta 4096)
% 0 --> 0° ; 2048 ---> 180° ; 4096 ---> 360°
Dynamixel.position(1,200);
Dynamixel.position(2,200);
Dynamixel.position(3,200);

delete(Dynamixel);
clear