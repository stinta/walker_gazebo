# 808X Week12S Submission: Walker_Gazebo Simulation
## Overview

This project levearges the  TurtleBot simulation and adds a simple walker algorithm much like a Roomba robot vacuum cleaner. The robot moves forward until it reaches an obstacle then rotates in place until the way ahead is clear.  Note that the robot makes subble rotations in order to avoid the obstable.

A launch file is provided to start a TurtleBot with the corridor.world from gazebo package.  The launch file also enables recording a bag if the enableRosBag is set to true.  Additionaly a walker_gazebo node is started that reads laserScan values and publishes velocity commands.
The record.bag file was uploaded to google drive; link provided below.

##Dependencies
The code in this repository has a dependency on gazebo environment installed and setup.  Additionally, the gazebo turtleBot packages should be installed

```
sudo apt-get install ros-kinetic-turtlebot-gazebo ros-kinetic-turtlebot-apps ros-kinetic-turtlebot-rviz-launchers
```
Verify that gazebo was installed properly
```
roslaunch turtlebot_gazebo turtlebot_world.launch
```

## Standard install clonning from git

On a catkin workspace src folder:
```
git clone --recursive https://github.com/stinta/walker_gazebo.git
```
On a catkin workspace
```
catkin_make
```

## Running and Starting walker_gazebo package
```
cd <path to repository>
source devel/setup.sh
```
roslaunch walker_gazebo walker_gazebo.launch
```

Note that an argument can be provided to record a rosbag in the /results folder 
```
roslaunch walker_gazebo walker_gazebo.launch enableRosBag:=true
```

## Inspecting the rosbag file
Bag file can be located at: [here](https://drive.google.com/file/d/1cRpIyNQf9hf_aEnLnw0K1w7JiLt4rd__/view?usp=sharing)
To view highlevel details
```
rosbag info record.bag
```

To replay the bag file ensure that roscore is running
```
rosbag play record.bag
```

## Code Checks

cppcheck
```
cppcheck --std=c++11 $(find . -name \*.cpp -or -name \*.srv | grep -vE -e "^./
build/" -e "^./results/")
```
cpplint
```
 cpplint $(find . -name \*.cpp | grep -vE -e "^./build/" -e "^./results")
```
## Author
Sandra P. Tinta

## License
[BSD-3](https://opensource.org/licenses/BSD-3-Clause)
