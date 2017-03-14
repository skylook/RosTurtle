## Game with known markers
Solution created for the ROS Challenge as part of the Minor Adaptive Robotics by Fontys Eindhoven.

### Features
The game_known_markers solution assumes the markers located in the arena are already known and mapped out. The solution game_unknown_markers can be used when there is no prior marker knowledge.

### Usage
This solution consists of two parts: a python program that performs the calculations and communicates with the ROS nodes, and a launch file.

#### Launch file
The launch file is located in [launch/game_turtle.launch](/game_known_markers/launch/game_turtle.launch) and has to be executed on the Turtlebot system. This will initiate the Turtlebot bringup and start the mapserver, amcl, move_base and the ar_track alvar. The launch file can be executed with the following command.
```
roslaunch ~/<path to launch file dir>/game_turtle.launch
```
#### Python program
The python program is located in [python/](/game_known_markers/python/). This folder contains two files: a library and an executable. The [game_known_markers.py](/game_known_markers/python/game_known_markers.py) has to be run to start the program. This can be done with the following command (assuming the terminal is pointed to the directory of the python file).
```
python game_known_markers.py
```
### Requirements
#### Turtlebot
This solution is created to be used with a Turtlebot 2 robot and assumes the Turtlebot is already configured correctly.

#### ar_track_alvar
Turtlebot AR marker tracking is perfomed using the [ar_track_alvar](http://wiki.ros.org/ar_track_alvar) package

Install the following package on the turtlebot laptop.
```
sudo apt-get install ros-indigo-ar-track-alvar
```
