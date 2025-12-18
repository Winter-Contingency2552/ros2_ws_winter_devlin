SIMULATED MOBILE ROBOT - ROS 2 WORKSPACE
=======================================

A multi-package ROS 2 workspace that creates a simulated mobile robot using C++ and Python nodes. The workspace demonstrates inter-package communication, a custom service interface, URDF-based robot description, and launch-driven simulation workflows.


an additional standalone file has been added to the repo to demonstrate feature correspondence separate from the full simulation. The image paths will need to be hardcoded for this to run

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
- you will need to run this to install lightglue
  ```
  git clone https://github.com/cvg/LightGlue.git
  ```
  - in 'feature_correspondence' there is a line of code in the imports that adds it to path, you will have to change this to work on your machine
  

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


   - Launch command
     ```
     ros2 launch robot_bringup final_mission.launch.py
     ```
PACKAGE SUMMARIES
---------------
Brief descriptions of packages in this ROS2 workspace.

- **aruco_detection**  
  Detects ArUco markers from a camera stream.  
  - Publishes human-readable marker reports and marker counts for downstream nodes.
  - Consumes image/TF topics from robot bringup/sensors.

- **aruco_interfaces**  
  Custom message definitions for ArUco data.  
  - Messages: `ArucoMarker.msg`, `ArucoMarkerArray.msg`.
  - Used by vision packages to standardize marker payloads.

- **custom_interfaces**  
  Shared service definitions for simulation and control utilities.  
  - Example: `ResetPosition.srv`.
  - Used across nodes that need simulation/robot control services.

- **feature_correspondance**  
  Compares two image streams to find matched features (SURF/ORB).  
  - Detects correspondences and movement.
  - Publishes matched feature coordinates and movement vectors for downstream processing.

- **robot_bringup**  
  Robot startup resources and configs.  
  - Launch files, URDF model, RViz configs.
  - TF and sensor publisher nodes to initialize the simulated robot environment.

- **robot_simulator_cpp**  
  C++ examples and scaffolding.  
  - Example publisher and odometry nodes.
  - Useful as a template for C++ ROS2 node development.

- **robot_simulator_py**  
  Python-based TF controller and motion node.  
  - `controller_node.py` implements differential-drive square-path navigation and publishes `cmd_vel`.

- **state_machine**  
  Finite-state machine utilities and example node.  
  - Task-level sequencing and behavior orchestration helpers.
  - Example implementations to drive higher-level autonomy.

TROUBLESHOOTING
---------------
- If a launch fails, confirm you sourced install/setup.bash from the workspace you just built.
- Use `ros2 topic list`, `ros2 service list`, and `ros2 node list` to inspect runtime communication.
- Review console output and files in `log/` for errors and stack traces.

Notes
---------------
This workspace heavily intaracts with a evaluator workspace and will not work on its own
Here is an overview of the code in the repo
```
-----------------------------------------------------------------------------------
Language                         files          blank        comment           code
-----------------------------------------------------------------------------------
JSON                                80              0              0          19188
CMake                              185           1203           1482           8964
D                                   31              0              0           5662
Python                              44            810            557           4708
C                                   16            467            531           3839
C++                                 17            804            352           3634
C/C++ Header                        49            653            717           2272
make                                 7            650            628           1591
Bourne Shell                        39            227            616           1424
PowerShell                          25            140            415            963
zsh                                 15            112            185            471
INI                                  5            110              0            415
Bourne Again Shell                  15             97            155            358
XML                                 11             50              6            349
YAML                                 2              0              0            294
Markdown                             2             20              0             97
TypeScript                          39              0              0             78
IDL                                  3              8              0             39
Gencat NLS                           4              1              0              6
Windows Resource File                1              0              0              1
-----------------------------------------------------------------------------------
SUM:                               590           5352           5644          54353
-----------------------------------------------------------------------------------

```

