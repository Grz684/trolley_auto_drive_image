## ROS2 driver for 2d laser scanners


**Required platform:**
Ubuntu 20.04 / ROS Foxy OR Ubuntu 20.04 / ROS Galactic  OR Ubuntu 22.04 / ROS Humble

  
**Install the missing dependencies:**
```
export ROS_DISTRO=foxy OR export ROS_DISTRO=galactic OR export ROS_DISTRO=humble
cd <path/to/workspace>
rosdep update --include-eol-distros
rosdep install --from-paths src --ignore-src --rosdistro=$ROS_DISTRO -y
```
  
**Build the workspace:**
```
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install
source install/setup.bash
```
  
**Usage:**
Now you are ready to use the driver. Make the necessary power and ethernet connections. Make sure your computer's IP address is on the same subnet mask as that of the device. Change the `scanner_ip` `port` `version`value in the respective yaml config file that can be found in: `src/olei_driver/config/`. You can now launch one of the drivers in the following manner: 

Note:`version` is the communication protocol version definition of the device. 2 means the communication protocol of v2.x, and 3 means the communication protocol of v3.x. This value is filled in according to the protocol version of the currently connected device.
 
```
ros2 launch olei_driver vf.launch.py

rviz2 -f scanner_link //equal to config yaml frame_id
RViz topic switch Reliability Policy to 'Best Effort'
```
