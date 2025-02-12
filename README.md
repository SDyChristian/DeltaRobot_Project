# Delta Robot Project
In this repository you will find three main sections regarding this project:
 - Robot Kinematics
 - Drawing Robot
 - Pick & Place Application

## Robot kinematics
This section includes functions for computing both forward and inverse kinematics.
- Kinematics is tested in MATLAB-Simulink, where a trajectory in Euclidean space (x, y, z) is defined. This trajectory is input into the inverse kinematics function to calculate the robot's joint angles. Subsequently, these angles are used in the forward kinematics function to determine the position of the end effector (x, y, z). As a result, the output from the forward kinematics should match the defined trajectory.
- Additionally, the inverse kinematics function is utilized to plot the robot's workspace.

## Drawing Robot 
This section includes programs to:

- Generate robot trajectories from an image (no robot hardware required), and
- Use inverse kinematics to follow trajectories on a real robot (robot hardware required).

Additionally, you will find:

- A program to test Dynamixel motors, and
- A program to follow time-parametrized curves, such as circles and polar stars.

## Pick&Place application
Pick-and-Place is one of the most common tasks performed by a delta robot. In this section, you will find programs to:

Extract objects from an image (objects to be relocated into a container), which serve as input to
An algorithm that generates trajectories for transporting the object and relocating it into a container.

Both algorithms are integrated into a main program that manages robot control (robot hardware required).

