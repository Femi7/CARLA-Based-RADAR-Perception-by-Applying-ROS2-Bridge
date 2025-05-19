# CARLA-Based-RADAR-Perception-by-Applying-ROS2-Bridge
# CARLA Installation and Configuration
# To set up CARLA, execute the following commands:
mkdir ~/carla && cd ~/carla
wget https://github.com/carla-simulator/carla/releases/download/0.9.13/CARLA_0.9.13.tar.gz
tar -xvzf CARLA_0.9.13.tar.gz
cd CARLA_0.9.13
./CarlaUE4.sh

# Setting Up ROS2 and the ROS2 Bridge
# Install ROS2
sudo apt update && sudo apt install ros-humble-desktop
source /opt/ros/humble/setup.bash
#  Clone and Build the ROS2 Bridge
mkdir -p ~/ros2_carla_ws/src && cd ~/ros2_carla_ws/src
git clone https://github.com/carla-simulator/ros-bridge.git
cd ~/ros2_carla_ws
colcon build --symlink-install
source install/setup.bash
#  Connect ROS2 to CARLA
#  launch the ROS2 bridge:
 ''''bash
 ros2 launch carla_ros_bridge carla_ros_bridge.launch.py
 ## Usage 
 Run the Ros2 node:
 ''''bash
 ros2 run radar_perception radar_perception
