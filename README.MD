# Torque Controllers

This package is made for the torque controllers for robotic manipulators. For inutition behind the source code, please visit https://shravista.github.io/blog/torque-controllers-for-manipulators.html

## Steps for installation
Requirements: Ubuntu 22.04, ROS 2 Humble, franka_ros2.

<b>This branch is very dedicated to franka simulation and hardware. This is still under development.</b>

#### Note: Compilation with pinocchio takes significant build time. But the runtime execution is pretty fast.

Preparation:
1. Clone Git Repo
```
git clone https://github.com/Shravista/torque_controllers.git
```
2. compile packages
```
colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

### To Do List
1. integration with real robots

