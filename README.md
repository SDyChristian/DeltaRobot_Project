<div align="center">
 
# Delta Robot Project

<img alt="Robot Delta" src="https://github.com/SDyChristian/DeltaRobot_Project/blob/main/Images/Delta_Robot.PNG" width="500" />

</div>

In this repository you will find three main sections regarding this project:
 - Robot Kinematics
 - Drawing Robot
 - Pick & Place Application

## Robot kinematics

<img alt="Kinematics" src="https://github.com/SDyChristian/DeltaRobot_Project/blob/main/Images/Kinematics_Diagram.JPG" width="500" />

This section includes functions for computing both forward and inverse kinematics.
- Kinematics is tested in MATLAB-Simulink, where a trajectory in Euclidean space (x, y, z) is defined. This trajectory is input into the inverse kinematics function to calculate the robot's joint angles. Subsequently, these angles are used in the forward kinematics function to determine the position of the end effector (x, y, z). As a result, the output from the forward kinematics should match the defined trajectory.
- Additionally, the inverse kinematics function is utilized to plot the robot's workspace.

## Drawing Robot 

<img alt="Drawing Robot" src="https://github.com/SDyChristian/DeltaRobot_Project/blob/main/Images/DrawingRobot.png" width="500" />

This section includes programs to:

- Generate robot trajectories from an image (no robot hardware required), and
- Use inverse kinematics to follow trajectories on a real robot (robot hardware required).

Additionally, you will find:

- A program to test Dynamixel motors, and
- A program to follow time-parametrized curves, such as circles and polar stars.

## Pick&Place application

<img alt="PickandPlace" src="https://github.com/SDyChristian/DeltaRobot_Project/blob/main/Images/Pick%26Place_Sequence.PNG" width="500" />

Pick-and-Place is one of the most common tasks performed by a delta robot. In this section, you will find programs to:

Extract objects from an image (objects to be relocated into a container), which serve as input to
An algorithm that generates trajectories for transporting the object and relocating it into a container.

Both algorithms are integrated into a main program that manages robot control (robot hardware required).

