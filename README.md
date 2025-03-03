# PX4 formation simulation
A ROS2 package providing the high-level commands for PX4 UAV
## Requirements
* Ubuntu 22.04 LTS
* Gz sim Garden 7.9.0
* ROS2 humble

## Installation
* Install [ROS2 humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
* Install Micro XRCE-DDS
  ```
  cd ~
  git clone -b 2.4.2 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
  cd Micro-XRCE-DDS-Agent
  mkdir build
  cd build
  cmake ..
  make
  sudo make install
  sudo ldconfig /usr/local/lib/
  ```
* Install formation simulation package
  ```
  cd ~/
  mkdir -p /formation_ws/src
  cd ~/formation_ws/src 
  git clone git@github.com:TigerWuu/PX4_formation_sim.git --recursive
  cd ..
  colcon build --packages-select px4_msgs px4_ros_com
  colcon build
  ```
* Quick test
  ```
  source /opt/ros/humble/setup.bash && source ~/formation_ws/install/local_setup.bash
  ros2 launch control joy.xml 
  ```


  
