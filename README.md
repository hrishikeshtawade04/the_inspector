<h1 align=center> ENPM808X Final Project </h1>
<h1 align=center>
[![Build Status](https://travis-ci.org/hrishikeshtawade04/the_inspector.svg?branch=master)](https://travis-ci.org/hrishikeshtawade04/the_inspector)
[![Coverage Status](https://coveralls.io/repos/github/hrishikeshtawade04/the_inspector/badge.svg?branch=master)](https://coveralls.io/github/hrishikeshtawade04/the_inspector?branch=master)
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
</h1>
<h1 align=center> The Inspector </h1>

<p align="center">
<img src="images_for_readme/TheInspector.gif" width="50%" height="50%">
</p>

## Project Overview
The Inspector product by Acme robotics is one of its flagship products. It performs warehouse infrastructure leakage inspection and gives the location of the leak in the environment and also a photo of the leakage. The user can then decide based on the photo how serious the leakage is actually and based on the leakage coordinates can track down the leakage. The Inpector runs utilizes the famous turtlebot platform, equipped with a LIDAR sensor to find distance from the wall and a RGBD camera to take photos of the leakages.

## Presentation
- Presentation slides [LINK]()
- Presentation Video - Presentation [LINK]()|Building packages [LINK]()| Demo [LINK]() |Saving the map [LINK]().

## License
This project is under the [BSD License](https://github.com/hrishikeshtawade04/the_inspector/blob/master/LICENSE).

## Agile Development
This product for Acme Robotics has is been developed by following the Solo Iterative Process (SIP), which a agile development process. You can take a look at the log details by going on this [LINK](https://docs.google.com/spreadsheets/d/1YTPVK5r-ZWE2yvQfAulh54kC3Dr284mhhSaxrUf1OMI/edit?usp=sharing).

Planning notes are given in this [LINK](https://docs.google.com/document/d/1v37j9J9pUYrncYB3P4gCmah4dpz-IAK4ekfOWanukgU/edit).


#### Authors
- Hrishikesh Tawade
- Kapil Rawal

## Dependencies
The Inspector requires following dependencies.
- ROS Kinetic kame
- Turtlebot ROS packages
- Gazebo 7.x
- Turtlebot Gazebo packages
- Rviz (Optional)
- Mapserver
- Googletest
- OpenCV
- Ubuntu 16.04

## Dependencies Installations

- To Install ROS Kinetic Kame, follow the instructions in this link [LINK](http://wiki.ros.org/kinetic/Installation/Ubuntu).

- To install Turtlebot simulation stack type, run the following after installing ROS Kinetic on your ubuntu 16.04.
```
$ sudo apt-get install ros-kinetic-turtlebot-gazebo ros-kinetic-turtlebot-apps ros-kinetic-turtlebot-rviz-launchers
```

- To Install Gazebo 7.x follow this link  [LINK](http://gazebosim.org/tutorials?tut=install_ubuntu&ver=7.0).

- To Install OpenCV follow this link  [LINK](https://github.com/kyamagu/mexopencv/wiki/Installation-(Linux,-Octave,-OpenCV-3).

- You can install Google Test Framework by going on this  [LINK](https://www.eriksmistad.no/getting-started-with-google-test-on-ubuntu/).
We are using google test frame work for testing of our classes and their methods.

- To install map_server use the folllowing commands
```
$ sudo apt-get install ros-kinetic-map-server
```
- To install Rviz go through this link [LINK](http://wiki.ros.org/rviz/UserGuide).

## Operation
To be uploaded

## Program installation
```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
$ source devel/setup.bash
$ cd src/
$ git clone --recursive https://github.com/hrishikeshtawade04/the_inspector.git
$ cd ..
$ catkin_make
```
Before building the model we need to move the my_Wall file which is in World folder to .gazebo/models/.
Execute below commands in new terminal.
```
$ cd ~/.gazebo/models/
$ mv <path to repository>/Wall/my_Wall ./
```
## Instructions to run program

To run code using launch command, open a new terminal window and run following command
```
$ cd <path to catkin_ws>
$ source devel/setup.bash
$ roslaunch the_inspector runTheInspector.launch
```

## Running ROS test via command-line
The test is written using gtest and rostest. Close all the running processes before executing the commands below to run the rostest.
```
$ cd <path to catkin Workspace>
$ source devel/setup.bash
$ catkin_make run_tests_the_inspector
```
Or test using launch file
```
$ cd <path to catkin Workspace>
$ source devel/setup.bash
$ rostest the_inspector inspect.launch
```
## Demo
To be uploaded

## Known issues/ Bugs
To be uploaded

## API and other developer documentation
To be uploaded
