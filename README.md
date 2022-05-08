# Robotics Project 1

10571078 - Alessandro Cogollo
XXXXXXX - Mario Cela

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
structure of any custom message

## Usage instructions:

description of how to start/use the nodes
## Other infos:



