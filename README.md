# INSTALLATION
```
mkdir ur5_ws
cd ur5_ws
git clone -b main https://github.com/sarmadahmad8/Automated-pipeline-for-fruit-plucking-using-UR5-Robotiq-gripper-and-ZED-stereo-camera.git
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```
# DEPENDENCIES
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

# START NODES
### IDLE/BACKGROUND NODES
```
#optional ursim node
#ros2 run ur_client_library start_ursim.sh -m ur5
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5 robot_ip:=192.168.1.102
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5 launch_rviz:=true
ros2 run robotiq_2f_urcap_adapter robotiq_2f_adapter_node.py --ros-args -p robot_ip:=192.168.1.102
```
### ACTIVE NODES
```
ros2 run detection_publisher move_group_publisher
ros2 run moving_ur5 move_group_reciever --ros-args --params-file {HOME}/ur5_ws/ros_ur_driver/Universal_Robots_ROS2_Driver/ur_moveit_config/config/kinematics.yaml
#optional realtime detection node
#ros2 run moving_ur5 move_realtime --ros-args --params-file {HOME}/ur5_ws/ros_ur_driver/Universal_Robots_ROS2_Driver/ur_moveit_config/config/kinematics.yaml
```
# RUNNING
move_group_publisher node publishes x,y,z coordinates of detected object to topic ```target_position```, width of detected object using the bounding box to topic ```width_object```, and distance from camera to detected object to topic ```distance_pub```.
move_group_reciever subscribes to the published topics of ```target_position```, ```width_object```, ```distance_pub```. On running the node, the robot will move to its start position, it then moves robot to the ```target_position``` of object. After the robot reaches the object the gripper closes with 40N of force and width ```width_object``` to grab the object. The robot then moves back to its start position.
The start position and gripper force are configurable in ```move_group_reciever.cpp``` in the ```moving_ur5/src``` folder. A demonstration node has been created for real time robot movement named ```move_realtime```, it moves the robot to the location of the object dynamically. Currently the node runs at 5hz but can run upto 15hz however it should be used at this frequency under your own risk as awkward inverse kinematic solutions can be dangerous for the robot.

In the moveit nodes rviz window. Go to panels and add 'moveit visual tools' panel. When prompted to 'press next in the rviz visual tools window', press next to move the robot to the planned path. Always supervise the planned path before pressing `'Next'.
