SIMULATED MOBILE ROBOT — ROS 2 WORKSPACE
=======================================

A multi-package ROS 2 workspace that creates a simulated mobile robot using C++ and Python nodes. The workspace demonstrates inter-package communication, a custom service interface, URDF-based robot description, and launch-driven simulation workflows.

CONTENTS / HIGH-LEVEL OVERVIEW
------------------------------
- custom_interfaces: custom service definition(s) (.srv) that describe service I/O.
- robot_bringup: launch files, robot URDF, and RViz configuration; also contains the Python controller and teleop launch files.
- robot_simulator_cpp: C++ nodes (for example, an odometry node) that run in simulation.
- Additional utilities and configuration files to support simulation and visualization.

PROJECT FOLDER STRUCTURE
------------------------
Root workspace (ros2_ws_winter_devlin)

/home/john/ros2_ws_winter_devlin
├── src
│   ├── custom_interfaces
│   │   └── srv/
│   ├── robot_bringup
│   │   ├── launch/
│   │   ├── urdf/
│   │   └── rviz/
│   ├── robot_simulator_cpp
│   │   └── src/
│   └── ... (other packages or nodes)
├── build/
├── install/
└── log/

KEY FEATURES
------------
- Multi-language nodes (C++ and Python)
- Custom service interface for inter-node communication
- Launch files to start teleop or finite state machine (FSM) simulations quickly
- URDF model and RViz configurations for visualization

PREREQUISITES
-------------
- A supported ROS 2 distribution installed and sourced
- colcon build
- Required system dependencies for Python and C++ packages

BUILD AND RUN
-------------
1. Build the workspace
   ```
   cd ~/ros2_ws_winter_devlin
   colcon build
   ```

2. Source the workspace setup
   ```
   source install/setup.bash
   ```

3. Launch options
   - Teleoperation
     ```
     ros2 launch robot_bringup robot_teleop.launch.py
     ```
     After launching, a new terminal may open. Focus that terminal and follow the on-screen keyboard instructions to control the robot. Note: strafing is not supported.

   - Finite State Machine simulation (FSM)
     ```
     ros2 launch robot_bringup robot_simulation.launch.py
     ```

PACKAGE SUMMARIES
-----------------
- custom_interfaces — contains .srv definitions used by other packages.
- robot_bringup — provides launch files, the URDF model, RViz configurations, and Python controller/teleop nodes.
- robot_simulator_cpp — contains the C++ odometry node and other simulation-related C++ code.

TROUBLESHOOTING
---------------
- If a launch fails, confirm you sourced install/setup.bash from the workspace you just built.
- Use `ros2 topic list`, `ros2 service list`, and `ros2 node list` to inspect runtime communication.
- Review console output and files in `log/` for errors and stack traces.


