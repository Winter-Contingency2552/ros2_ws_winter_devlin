SIMULATED MOBILE ROBOT - ROS 2 WORKSPACE
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

```
/home/john/ros2_ws_winter_devlin
├── README.md
├── README_ANSWERS.md
├── build/
├── install/
├── log/
└── src
├── aruco_detection
│ ├── aruco_detection
│ │ └── aruco_detection.py
│ ├── LICENSE
│ ├── package.xml
│ ├── setup.cfg
│ ├── setup.py
│ └── test/
├── aruco_interfaces
│ ├── CMakeLists.txt
│ ├── package.xml
│ └── msg/
│ ├── ArucoMarker.msg
│ └── ArucoMarkerArray.msg
├── custom_interfaces
│ ├── CMakeLists.txt
│ ├── package.xml
│ └── srv/
│ └── ResetPosition.srv
├── feature_correspondance
│ ├── feature_correspondance
│ │ ├── feature_correspondance.py
│ │ └── feature_correspondence.py
│ ├── LICENSE
│ ├── package.xml
│ ├── setup.cfg
│ ├── setup.py
│ └── test/
├── robot_bringup
│ ├── CMakeLists.txt
│ ├── package.xml
│ ├── launch/
│ │ ├── project_test.launch.py
│ │ ├── robot_simulation.launch.py
│ │ └── robot_teleop.launch.py
│ ├── rviz/
│ │ ├── basic_robot.rviz
│ │ └── simulation.rviz
│ ├── sdf/
│ │ ├── model.config
│ │ └── model.sdf
│ └── urdf/
│ └── robot.urdf
├── robot_simulator_cpp
│ ├── CMakeLists.txt
│ ├── LICENSE
│ ├── package.xml
│ └── src/
│ └── publisher_member_function.cpp
├── robot_simulator_py
│ └── robot_simulator_py
│ └── controller_node.py
└── state_machine
├── state_machine
│ └── state_node.py
├── LICENSE
├── package.xml
├── setup.cfg
└── setup.py
```



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
- you will need to run
  ```
  git clone https://github.com/cvg/LightGlue.git
  ```
- to install lightglue
- in feature correspondence there is a line of code in the imports that adds it to path, you will have to change this to work on your machine
  

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


   - Finite State Machine simulation (FSM)
     ```
     ros2 launch robot_bringup final_mission.launch.py
     ```
PACKAGE SUMMARIES
---------------
-aruco_detection: detects ArUco markers from a camera stream; publishes human-readable reports and marker counts for downstream nodes.
-aruco_interfaces: defines custom message types for ArUco data (ArucoMarker.msg, ArucoMarkerArray.msg) used by vision packages.
-custom_interfaces: contains service definitions used across the workspace (e.g., ResetPosition.srv) for simulation/control utilities.
-feature_correspondance: compares two image streams to find matched features (SURF/ORB), detects movement/correspondences, and publishes movement coordinates.
-robot_bringup: provides launch files, the URDF model, RViz configurations, and related resources to start the simulated robot and sensor/TF publishers.
-robot_simulator_cpp: contains C++ example code (publisher/odometry examples) and package scaffolding for C++ nodes.
-robot_simulator_py: contains the TF-based controller_node.py that implements the differential-drive square-path navigation logic and publishes cmd_vel.
-state_machine: example finite-state machine node and helper code for task-level sequencing and behavior orchestration.


TROUBLESHOOTING
---------------
- If a launch fails, confirm you sourced install/setup.bash from the workspace you just built.
- Use `ros2 topic list`, `ros2 service list`, and `ros2 node list` to inspect runtime communication.
- Review console output and files in `log/` for errors and stack traces.

Notes
---------------
This workspace heavily intaracts with a evaluator workspace and will not work on its own

