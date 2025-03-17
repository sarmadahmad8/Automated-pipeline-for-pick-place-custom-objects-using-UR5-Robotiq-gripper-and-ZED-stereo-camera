## INSTALLATION
```
mkdir ur5_ws
cd ur5_ws
git clone -b main https://github.com/sarmadahmad8/Automated-pipeline-for-fruit-plucking-using-UR5-Robotiq-gripper-and-ZED-stereo-camera.git
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```
## DEPENDENCIES
### ENVIRONMENT DEPENDENCIES
Ubuntu 22.04\
ROS2 Humble\
CUDA 11.8

### ROS2 DEPENDENCIES
Moveit2\
moveit-visual-tools\
ros-dev-tools

### PYTHON DEPENDENCIES
Python 3.10\
ZED api (pyzed)\
inference-gpu==0.37.1\
opencv-python==4.10.0.84

## START NODES
### IDLE/BACKGROUND NODES
```
#optional ursim node
#ros2 run ur_client_library start_ursim.sh -m ur5
1. ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5 robot_ip:=192.168.1.102
2. ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5 launch_rviz:=true
3. ros2 run robotiq_2f_urcap_adapter robotiq_2f_adapter_node.py --ros-args -p robot_ip:=192.168.1.102
```
### ACTIVE NODES
```
1. ros2 run detection_publisher move_group_publisher
2. ros2 run moving_ur5 move_group_reciever --ros-args --params-file {HOME}/ur5_ws/ros_ur_driver/Universal_Robots_ROS2_Driver/ur_moveit_config/config/kinematics.yaml
#optional realtime detection node
#ros2 run moving_ur5 move_realtime --ros-args --params-file {HOME}/ur5_ws/ros_ur_driver/Universal_Robots_ROS2_Driver/ur_moveit_config/config/kinematics.yaml
```
## RUNNING
The ur.control.launch.py node (background node #1) activates communication between the real robot and the ros environment. It will spawn a rviz wondow displaying the robot in the exact position it is currently in real life. Alternatively this node can be used to communicate with a simulated robot on ursim (optional background node). \
The ur.moveit.launch.py node (background node #2) launches the motion planning for the robot in a seperate rviz window. Here you can experiment with custom position using the interative movement markers for the robot and plan/execute the movement. You can also set up a custom position using joint angles in the joints tab of the motion planning side tab. \
The robotiq_2f_adapter_node node (background node #3) establishes communication with the robotiq gripper (both 2f-85 and 2f-140 are supported). It publishes two states, ```inactive``` and ```active``` to a topic named ```gripper_status```. When the gripper it as its fully open position the state will be ```inactive``` (do not get confused the gripper is still connected and working its just not currently gripping anything), when the gripper moves beyond a certain threshold (currently 0.82) it will publish as ```acive```. This mechanism is added for  custom scripts to run upon the gripper closing and opening, we used it as start/stop for custom data collection scripts. The threshold can be adjusted in the robotiq_2f_adapter_node.py file in the ```robotiq_gripper_control/robotiq_2f_urcap_adapter/scripts```.\
The move_group_publisher node (active node #1) publishes x,y,z coordinates of detected object to topic ```target_position```, width of detected object using the bounding box to topic ```width_object```, and distance from camera to detected object to topic ```distance_pub```.\
The move_group_reciever node (active node #2) subscribes to the published topics of ```target_position```, ```width_object```, ```distance_pub```. On running the node, the robot will move to its start position, it then moves robot to the ```target_position``` of object. After the robot reaches the object the gripper closes with 40N of force and width ```width_object``` to grab the object. The robot then moves back to its start position.\
The start position and gripper force are configurable in ```move_group_reciever.cpp``` in the ```moving_ur5/src``` folder.\
A demonstration node has been created for real time robot movement named ```move_realtime``` (optional active node), it moves the robot to the location of the object dynamically. Currently the node runs at 5hz but can run upto 15hz however it should be used at this frequency under your own risk as awkward inverse kinematic solutions can be dangerous for the robot.

## Troubleshooting
### Q. On running active node #1 the robot shows planned path but does not execute.
A. In the moveit nodes rviz window (background node 2 rviz window). Go to panels and add 'moveit visual tools' panel. When prompted to ```press next in the rviz visual tools window``` in the moveit terminal (background node 2 terminal window), press next to move the robot to the planned path. Always supervise the planned path before pressing ```Next```.
### Q. How do i add my custom roboflow model to this pipeline?
A. Simply add you ```model_id``` and ```api_key``` of the roboflow project to the ```move_group_publisher.py``` in the ```detection_publisher_1/src/detection_publisher/detection_publisher``` folder under the header ```#Initialize the Roboflow model``` and rebuild.
### Q. I made changes to some code/ implemented a new function however its not working when running the nodes.
A. Always rebuild the nodes when finalizing any new changes. Go to ```ur5_ws``` and enter ```colcon build``` and then ```source install/setup.bash```.
