# Package: trajcontrol (version: trajcontrol_jhu)

## Overview
This repository contains:

- ROS2 trajcontrol package (see nodes and message exchanges in [Communication Diagram](#comm_diagram))
- Launch files for different control tasks for Lisa robot (see details in [Usage](#usage))

## Usage <a name="usage"></a>

Create a workspace and to the src folder, commit the following repositories:
- [trajcontrol](https://github.com/maribernardes/trajcontrol_jhu)
- [ros2_needle_guide_robot](https://github.com/SmartNeedle/ros2_needle_guide_robot)
- [ros2_needle_shape_publisher](https://github.com/SmartNeedle/ros2_needle_shape_publisher.git)
- [ros2_hyperion_interrogator](https://github.com/SmartNeedle/ros2_hyperion_interrogator.git)
- [ros2_igtl_bridge](https://github.com/tokjun/ros2_igtl_bridge)

Remember to install [OpenIGTLink](https://github.com/openigtlink/OpenIGTLink)

To build system packages:
```bash
  colcon build --cmake-args -DOpenIGTLink_DIR:PATH=<insert_path_to_openigtlink>/OpenIGTLink-build --symlink-install
```

To run in debug mode, include:
```bash
  --ros-args --log-level debug
```

#### Registration procedure:
To be defined

#### Jacobian experimental initialization:
To be defined

#### Manually move the robot:
You may want to manually position the robot (in horizontal and vertical directions) using the keyboard. 

To use robot in manual mode, open 3 terminals:
1. Launch PlusServer with configFile 'PlusDeviceSet_Server_NDIAurora_1Needle.xml'
```bash
  sudo /opt/PlusBuild-bin/bin/./PlusServerLauncher
```
2. Launch robot in manual:
```bash
  ros2 launch trajcontrol manual.launch.py
```
3. Run keyboard node:
```bash
  ros2 run trajcontrol keypress
```
and use arrows from the numeric keyboard (2,4,6,8) to move robot up-down/left-right
No experimental data is recorded.

#### Move the robot to predefined positions:
You may need to move the robot to a pre-defined sequence of positions (waits 3.0s at each position before automatically moving to the next one)

To use robot in sequence mode, open 2 terminals:
1. Launch PlusServer with configFile 'PlusDeviceSet_Server_NDIAurora_1Needle.xml'
```bash
  sudo /opt/PlusBuild-bin/bin/./PlusServerLauncher
```
2. Launch robot in manual:
```bash
  ros2 launch trajcontrol sequence.launch.py filename:=NAME
```
Defining filename (default=my_data) is optional.
The file defined by 'filename' is a csv with all experimental data and is it saved as 'data/NAME.csv'

#### Move robot to a fixed horizontal position:
You may need to position the robot at a pre-defined X (horizontal) position with Z (vertical) in manual mode.
This is useful to perform the insertions at the same position with respect to the Aurora (and avoid parts of the measuring volume that are problematic).

To move robot to a fixed X, open 3 terminals:
1. Launch PlusServer with configFile 'PlusDeviceSet_Server_NDIAurora_1Needle.xml'
```bash
  sudo /opt/PlusBuild-bin/bin/./PlusServerLauncher
```
2. Launch robot in manual:
```bash
  ros2 launch trajcontrol init.launch.py
```
3. Run keyboard node:
```bash
  ros2 run trajcontrol keypress
```
and use arrows from the numeric keyboard (2,8) to move robot up-down
No experimental data is recorded.

#### Control the robot using the data-driven MPC lateral compensation:

To run the trajectory control with MPC, open 3 terminals:
1. Launch PlusServer with configFile 'PlusDeviceSet_Server_NDIAurora_1Needle.xml'
```bash
  sudo /opt/PlusBuild-bin/bin/./PlusServerLauncher
```
2. Launch robot in manual:
```bash
  ros2 launch trajcontrol mpc_step.launch.py H:=4 filename:=NAME 
```
Defining H (default=5) and filename (default=my_data) are optional.

3. Run keyboard node:
```bash
  ros2 run trajcontrol keypress
```
and use SPACE key from the keyboard to signal each insertion step.
Defining H(default:=5) and filename (default=my_data) are optional.
The file defined by 'filename' is a csv with all experimental data and is it saved as 'data/NAME.csv'


## Communication diagram <a name="comm_diagram"></a>

![alternative text](http://www.plantuml.com/plantuml/proxy?cache=no&src=https://raw.github.com/maribernarde/trajcontrol_jhu/main/comm_diagram.txt)

# For 2023 demo:
## Launch trajectory control package (robot, needle, sensor_processing, save_file, estimator and controller nodes)
``ros2 launch trajcontrol mcp_step.launch.py filename:=name_for_data_file`` 
## Run keypress node in a different terminal
``ros2 run trajcontrol keypress`` 

# For March 2022 demo:
## Launch trajectory control package (savefile, estimator and controller nodes)
``ros2 launch trajcontrol real_test.launch.py filename:=name_for_data_file`` 
## Run keypress node in a different terminal
``ros2 run trajcontrol keypress`` 
