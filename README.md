# turtle_sim

ROS2 node to let turtlesim follow the mouse.

## prerequisites

- [ROS2 humble](http://docs.ros.org/en/humble/Installation.html) (might work with other versions as well)
- [pyautogui](https://pypi.org/project/PyAutoGUI/)
- [xwininfo](https://github.com/freedesktop/xwininfo)

## build

1. First (like usually) ROS2 needs to be sourced 
```sh
source /opt/ros/humble/setup.bash # for bash
# or 
source /opt/ros/humble/setup.zsh  # for zsh
```
2. If not already done, install the [turtlesim](https://github.com/ros/ros_tutorials/tree/humble/turtlesim) package
```sh
sudo apt install ros-humble-turtlesim
```
2. Clone this repository into your ROS2 workspace `src/` folder
```sh
cd $ROS2_WS/src # or change direction manually
git clone https://github.com/meikse/turtle_motion -b main
```
3. Go back to your root of your workspace and build it via [colcon](http://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)
```sh
cd ..
colcon build --symlink-install turtle_motion
```
4. After building the package source the workspace as an overlay
```sh
source ./install/setup.zsh # for bash
# or
source ./install/setup.bash # for zsh
```

## run

For easy start-up purposes, a launch file is provided.
```sh
ros2 launch turtle_motion bring_up.launch.yaml
```
This launch file will bring up 5 nodes:

| package                                                                 | executable                             | description                               |
|-------------------------------------------------------------------------|----------------------------------------|-------------------------------------------|
| [turtlesim](https://github.com/ros/ros_tutorials/tree/humble/turtlesim) | turtlesim_node                         | ROS2 native turtle simulator              |
| turtle_motion                                                           | [window.py](./turtle_motion/window.py) | find position of turtle simulation window |
| turtle_motion                                                           | [mouse.py](./turtle_motion/mouse.py)   | tracks position of the mouse              |
| turtle_motion                                                           | [transform](./src/transform)           | transforms mouse frame to turtle frame    |
| turtle_motion                                                           | [control](./src/control)               | publishes velocity commands to the turtle |

Of course, all executables can be run individually for e.g. debugging purposes.

After launching the turtle will try to trace the mouse. The greater the distance between mouse and turtle, the faster the turtle moves.

The frames coordinates ca be echoed as well, e.g.
```sh
ros2 topic echo /window/center
```
will display the current position of the turtlesim_node window on the screen. The screen can be moved during simulation as well

Following topics are available:
| topic             | description                                                                       |
|-------------------|-----------------------------------------------------------------------------------|
| /mouse/distance   | distance between mouse and turtle
| /mouse/location   | absolute position relative to top-left screen corner
| /window/center    | center position of the turtlesim_node window on screen
| /window/dimension | dimensions of the turtlesim_node window
| /window/location  | distance between the turtlesim_node window and screen (both viewed from top-left)

No services or actions are available.

## TODO

- switching mouse vertical position around $pi$ will rotate turtle the long way around

