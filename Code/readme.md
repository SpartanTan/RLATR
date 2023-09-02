# Code base

This folder contains the source code produced in this project. For the moment, we included two types ros code (ROS1/ROS2), and python.

## Matlab_scripts
The /Matlab_scripts contains some matlab scripts for data visualization or equation verification.

Current files
- inverseJacobian.mlx  
  Derive the inverse Jacobian matrix of the kinematic model of the differential driven robot.

## python_tools
Currently nothing inside.

## ros_ws
Contains source code of ATR project and Reinforcement Learning controller source code.

```
.
├── factory_db
│   ├── config
│   └── launch
├── gpss_atr
│   ├── atr_controller
│   ├── atr_demo
│   ├── atr_description
│   ├── atr_driver
│   ├── atr_factory_state
│   ├── atr_joy
│   ├── atr_nona
│   ├── atr_path_generator
│   ├── atr_trajectory_generator
│   └── atr_utils
├── gpss_interfaces
│   ├── ar_tag_msgs
│   ├── atr_camera_state_msgs
│   ├── atr_error_msgs
│   ├── atr_formation_msgs
│   ├── atr_human_msgs
│   ├── atr_job_msgs
│   ├── atr_object_msgs
│   ├── atr_path_msgs
│   ├── atr_predicted_object_msgs
│   ├── atr_signal_msgs
│   ├── atr_srvs
│   └── atr_state_msgs
└── rl_atr
    └── kinematic_controller
```