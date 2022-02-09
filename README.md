# trajcontrol dependencies
``python3 -m pip install numpy-quaternion`` 

``pip install transforms3d``

``pip3 install opencv-python``

# Communication diagram
![alternative text](http://www.plantuml.com/plantuml/proxy?cache=no&src=https://raw.github.com/maribernardes/trajcontrol_jhu/main/comm_diagram.txt)

# For system integration demo:
## Launch interrogator and shape publisher
``ros2 launch hyperion_interrogator hyperion_demo.launch.py numCHs:=3 numAAs:=3`` 

``ros2 launch needle_shape_publisher sensorized_shapesensing_needle_decomposed.launch.py needleParamFile:=/home/mariana/ROS2/ws_jhu/src/ros2_needle_shape_publisher/needle_data/needle_3CH_4AA/needle_params_2021-08-16_Jig-Calibration_weighted_weights.json`` 

``ros2 run hyperion_interrogator calibrate_sensors –ros-args -r __ns:=/needle`` 

## Launch robot
<INSERT INSTRUCTIONS HERE>

## Launch trajectory control package (estimator and controller nodes)
``ros2 launch trajcontrol demo.launch.py`` 
