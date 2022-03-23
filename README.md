# trajcontrol dependencies
``python3 -m pip install numpy-quaternion`` 

``pip install transforms3d``

``pip3 install opencv-python``


# Communication diagram
![alternative text](http://www.plantuml.com/plantuml/proxy?cache=no&src=https://raw.github.com/maribernardes/trajcontrol_jhu/main/comm_diagram.txt)

# For March 2022 demo:
## Launch trajectory control package (savefile, estimator and controller nodes)
``ros2 launch trajcontrol real_test.launch.py filename:=name`` 
## Run keypress node in a different terminal
``ros2 run trajcontrol keypress`` 
