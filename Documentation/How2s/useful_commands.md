# Useful commands

- tree -d -L 2   
  Get tree structure of the folder. Copy paste them into readme file afterwards.

- colcon build --symlink-install --merge-install --packages-select atr_launch
- ros2 topic pub /joint_command_1 atr_state_msgs/msg/ATRJointCommand "wheel_velocity: [1.0, 1.0]"  

## For training 
- First navigate to this directory. Assuming you are now at /atr_rl
  ```
  cd Code/python_tools/FactoryEnv/tests
  ``` 
- the `test.py` file supports three tasks
  ```
  sudo /home/zhicun/miniconda3/envs/ros2_env/bin/python test.py --which-test 0 --which-option 0
  ```
  The options for `--which-test` are:
  - `0`: keyboard control
  - `1`: benchmark
  - `2`: using neural network to control ATR

  The options for `--which-option` are:
  - `0`: run a new path but do not save it
  - `1`: run a new path and save it
  - `2`: load a saved path
  
- Using PPO to train a network
  ```
  python PPO.py
  ```
  Possible options:
  - `--capture-video`: will record videos during training
  The weights of the network will be perserved in the /agents folder.

- Visualize the training process
  ```
  tensorboard --logdir=runs
  ```
  You will see a link in the termminal after you type in the command. Ctrl+click the link will open the browser and you will see the tensorboard interface.
  
# Custom messages

- atr_state_msgs/msg/ATRJointCommand
    ```
    Example topic name: /joint_command_1
    
    # This is the cammanded wheel velocities for the ATR
    # Notice it's [wr, wl]
    float64[2] wheel_velocity
    ```
- atr_state_msgs/msg/ATRJointState
  ```
  Example topic name: atr_driver_state_1
  This message subscribed by controller to close the loop. [x,y,\theta_z,v_x,v_y,\omega_z]

  Notice that is information using the world origin as reference frame which means if the starting point of ATR is modified in atr_demo's config param. e.g.
   initial_state: [1.0, 1.0, 0.0]
  Then the x,y position will start from [1, 1] as well.

  # This is the ATR's full state (2D pose and 2D linear and angular velocitites)
  # to be used in the controller [x,y,\theta_z,v_x,v_y,\omega_z]
  # We need this structure to pre-allocate the message in a pool.
  float64[6] full_state
  ```
- atr_state_msgs/msg/ATRStateStamped
  ```
    Example topic name: /atr_state_1
    This message subscribed by factory to publish combineed message with other ATRs

    # Header with Time and frame_id
      std_msgs/Header header
        builtin_interfaces/Time stamp
          int32 sec
          uint32 nanosec
        string frame_id

    # ATR State
    atr_state_msgs/ATRState state
      int8 atr_id
      atr_state_msgs/ATRPoseData pose
        geometry_msgs/Pose odom
          Point position
            float64 x
            float64 y
            float64 z
          Quaternion orientation
            float64 x 0
            float64 y 0
            float64 z 0
            float64 w 1
        geometry_msgs/Pose optom
          Point position
            float64 x
            float64 y
            float64 z
          Quaternion orientation
            float64 x 0
            float64 y 0
            float64 z 0
            float64 w 1
        geometry_msgs/Pose fused_odom
          Point position
            float64 x
            float64 y
            float64 z
          Quaternion orientation
            float64 x 0
            float64 y 0
            float64 z 0
            float64 w 1
        int16 pose_id
        string name
        string description
        atr_state_msgs/ATRPoseType type
          int8 id
          int8 UNDEFINED=0
          int8 LOADING=1
          int8 UNLOADING=2
          int8 WAITING=3
          int8 IDLE=4
      atr_state_msgs/ATRMotionData vel
        geometry_msgs/Twist odom
          Vector3  linear
            float64 x
            float64 y
            float64 z
          Vector3  angular
            float64 x
            float64 y
            float64 z
        geometry_msgs/Twist optom
          Vector3  linear
            float64 x
            float64 y
            float64 z
          Vector3  angular
            float64 x
            float64 y
            float64 z
        geometry_msgs/Twist fused_odom
          Vector3  linear
            float64 x
            float64 y
            float64 z
          Vector3  angular
            float64 x
            float64 y
            float64 z
      atr_state_msgs/ATRMotionData acc
        geometry_msgs/Twist odom
          Vector3  linear
            float64 x
            float64 y
            float64 z
          Vector3  angular
            float64 x
            float64 y
            float64 z
        geometry_msgs/Twist optom
          Vector3  linear
            float64 x
            float64 y
            float64 z
          Vector3  angular
            float64 x
            float64 y
            float64 z
        geometry_msgs/Twist fused_odom
          Vector3  linear
            float64 x
            float64 y
            float64 z
          Vector3  angular
            float64 x
            float64 y
            float64 z
      bool full_state false
      int8 pose_source
      geometry_msgs/Pose goal
        Point position
          float64 x
          float64 y
          float64 z
        Quaternion orientation
          float64 x 0
          float64 y 0
          float64 z 0
          float64 w 1
      atr_state_msgs/ATRStateOverall overall
        int8 status
        int8 UNDEFINED=0                #
        int8 OUT_OF_SERVICE=1           #
        int8 RETURNING_TO_STATION=2     #
        int8 CHARGING=3                 #
        int8 DOCKED=4                   #
        int8 AVAILABLE=5                #
        int8 ON_MISSION=6               #
        int8 ERROR=7                    #
        int8 COLLISSION_AVOIDANCE=8     #
                                        # command
      atr_state_msgs/ATRStateMission mission
        int8 status
        int8 UNDEFINED=0    #
        int8 IN_TRANSIT=1   #
        int8 ARRIVED=2      #
        int8 NOT_IN_USE=3   #
        int8 FAILED=4       #
      atr_state_msgs/ATRStateLoad load
        int8 status
        int8 UNDEFINED=0    #
        int8 LOADING=1
        int8 LOADED=2
        int8 UNLOADING=3
        int8 UNLOADED=4
      atr_state_msgs/ATRStateSignals signal
        int8[] types
        int8 MOVING_FORWARD=0
        int8 MOVING_BACKWARD=1
        int8 MOVING_RIGHT=2
        int8 MOVING_LEFT=3
        int8 STOPPED=4
        int8 IDLE=5
        int8 EMERGENCY=6
        int8 CHARGING=7
        int8 SOUND=8
      atr_state_msgs/ATRStateActuator actuator
        int8 type
        int8 status
        float32 value
        int8 LINEAR=0
        int8 LIFTER=1
        int8 IDLE=0
        int8 OPEN=1
        int8 CLOSED=2
        int8 EXTENDED=3
        int8 RETRACTED=4
      bool emerg_stop
      atr_state_msgs/ATRBatteryState battery
        int8 status
        float32 charge_state
        float32 life_time
        float32 health_state
        int8 UNDEFINED=0
        int8 FULL=1
        int8 HIGH=2
        int8 MID=3
        int8 LOW=4
        int8 EMPTY=5
      atr_state_msgs/ATRCollisionState[] collisions
        int8 status
        float32 distance
        string description
        atr_state_msgs/ATRCollisionSensor[] sensors
          int8 type
          int8 id
          float32 data
          int8 BUMPER=0
          int8 ULTRA_SONIC=1
          int8 LIDAR=2
        int8 NONE=0
        int8 CLOSE_COLLISION=1
        int8 IN_COLLISION=2
      atr_error_msgs/ATRError error
        uint8 id 0
        string message
        uint8 NO_ERROR = 0
        uint8 DATA_NOT_AVAILABLE = 1
        uint8 CONNECTIVITY = 2
        uint8 INVALID_ARGUMENT = 3
        uint8 TIMEOUT = 4
        uint8 TRANSFORM = 5
        uint8 EMERGENCY_STOP = 6
        uint8 SAFETY_ERROR = 7
      int8 OPTOM=0
      int8 ODOM=1
      int8 FUSED_ODOM=2
      int8 FULL_POSE=3
  ```