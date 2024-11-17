# Torque Controllers

This is package to explore the torque controllers for robotic manipulators

## Steps for installation
Requirements: Ubuntu 24.04, ROS 2 Jazzy Jalisco, pinocchio, ros2 controllers, gazebo harmonic
This steps assumes that ros2_control, ros2_controllers, gazebo sim (harmonic), gz_ros2_control are already installed. In addition one needs
```
sudo apt install ros-jazzy-pinocchio
```

Preparation:
1. Clone Git Repo
```
git clone https://github.com/Shravista/torque_controllers.git
```
2. compile packages
```
colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```
3. launch file -> bring robot simulation
```
ros2 launch example_robots iiwa14_bringup_launch.py
```
4. example tests
```
ros2 run torque_control_tests iDControlVer2 --ros-args --params-file  <WORKSPACE>/torque_controllers/torque_control_tests/config/inverseDynamicsControl.yaml
```
### To Do List
1. Documentation
2. Tutorials
3. integration with real robots
