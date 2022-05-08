# Robotics Project 1

10571078 - Alessandro Cogollo
10685242 - Mario Cela

## Folders & Files:

**/cfg**: contains **parameters.cfg**, containing parameters set-up. Used to define an enum containing a switch for the dynamic reconfiguration.
**/launch**: contains **up.launch**, used to launch all nodes and init params.
**/msg**: contains **wheels_rpm_msg.msg**, used as a custom msg to post on topic */wheels_rpm*.
**/src**: contains nodes
**/srv**: contains **reset.srv**, used to reset the position of given robot.
**CMakeLists.txt**: 
**package.xml**:

## Parameters:

## init params
**gearRatio**: ratio between wheels and motor
**wheelRadius**: used to set the radius of wheels
**wheelAlongX**: used to set the distance between the center of the robot and the middle of the space between the wheels
**wheelAlongY**: used to set the distance between the center of the robot and wheel axis
**encoderResolution**: used to set the resolution of the encoders

##  TF tree:
We weren't able to complete the project with the TF tree, but we would have used a tree composed by: **world** (as the default TF), **odom**,  as the frame based on the body of the robot, and **base_link**.

## Custom messages:
wheels_rpm_msg consists of four floats that indicate each wheel velocity in rpm

## Usage instructions:
up.launch file in launch folder has to be run.

Relevant topics are:
**/cmd_vel**: robot's linear and angular velocity, calculated after reading from wheel_states topic
**/odom**: robot's pose, calculated with Euler or RK formulas, depending on the configuration
**/wheel_rpm**: robot's wheels rpm, calculated after reagind from cmd_vel

## Other infos:



