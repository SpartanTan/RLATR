# First time setting up

After download the source code of ATR project, you need to solve one dependency issue. 
```
cd Code/ros_ws/src/gpss_atr/atr_controller
gedit CMakeLists.txt
```
Go to line 50~51, add "atr_srvs" at the bottom of set() like this

<pre>
set(dependencies
    rclcpp
    rclcpp_components
    rclcpp_lifecycle
    lifecycle_msgs
    sensor_msgs
    atr_state_msgs
    atr_utils
    std_msgs
    <b>atr_srvs</b>)
</pre>

Then you should have this
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
└── gpss_interfaces
    ├── ar_tag_msgs
    ├── atr_camera_state_msgs
    ├── atr_error_msgs
    ├── atr_formation_msgs
    ├── atr_human_msgs
    ├── atr_job_msgs
    ├── atr_object_msgs
    ├── atr_path_msgs
    ├── atr_predicted_object_msgs
    ├── atr_signal_msgs
    ├── atr_srvs
    └── atr_state_msgs
```