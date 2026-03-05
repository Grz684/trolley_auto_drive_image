## 用于 2d 激光扫描仪的 ROS2 驱动程序

**所需平台：**
Ubuntu 20.04 / ROS Foxy 或 Ubuntu 20.04 / ROS Galactic 或 Ubuntu 22.04 / ROS Humble

**安装缺少的依赖项：**
```
export ROS_DISTRO=foxy 或 export ROS_DISTRO=galactic 或 export ROS_DISTRO=humble
cd <path/to/workspace>
rosdep update --include-eol-distros
rosdep install --from-paths src --ignore-src --rosdistro=$ROS_DISTRO -y
```

**构建工作区：**
```
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install
source install/setup.bash
```

**用法：**
现在您已准备好使用驱动程序。进行必要的电源和以太网连接。确保您的计算机的 IP 地址与设备的 IP 地址位于同一子网掩码上。在相应的 yaml 配置文件中更改 `scanner_ip` `port` `version` 值，该文件位于：`src/olei_driver/config/`。您现在可以按以下方式启动其中一个驱动程序：

注意：`version` 是设备的通信协议版本定义。2 表示 v2.x 的通信协议，3 表示 v3.x 的通信协议。此值根据当前连接的设备的协议版本填写。

```
ros2 launch olei_driver vf.launch.py

rviz2 -f scanner_link //‘scanner_link’等于 config yaml frame_id
RViz 界面中topic主题将可靠性策略切换为'Best Effort'
```
