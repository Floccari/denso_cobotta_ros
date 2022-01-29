Porting of denso cobotta drivers to ros2. Only rviz + moveit works for now.

https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html

\
Dependencies (for ros-foxy-desktop)
```bash
sudo apt-get install python3-colcon-ros ros-foxy-moveit ros-foxy-realtime-tools libyaml-cpp-dev ros-foxy-controller-manager ros-foxy-transmission-interface ros-foxy-xacro ros-foxy-joint-state-controller ros-foxy-joint-trajectory-controller ros-foxy-ros2-control ros-foxy-gazebo-ros ros-foxy-gazebo-ros2-control
```
\
Launch rviz, moveit and gazebo
```bash
source /opt/ros/foxy/setup.bash
cd denso_cobotta_ros
colcon build
source install/setup.sh
ros2 launch denso_cobotta_bringup simulation.launch.py
```

If the model does not show up in gazebo, create a link to denso_cobotta_descriptions under ~/.gazebo/models

