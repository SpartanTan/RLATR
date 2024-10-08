# This message defines the current state of the ATR

# ATR ID
int8 atr_id

# ATR Pose: continuous (odo, opto, fused), discrete (pose_id, pose_type)
atr_state_msgs/ATRPoseData pose

# ATR Velocity continuous (linear and angular)
atr_state_msgs/ATRMotionData vel

# ATR Acceleration continuous (linear and angular)
atr_state_msgs/ATRMotionData acc 

# Defines if the ATR state provides already all the pose types (odom, optom, fuse_odom). 
# false: the ATR state contains only optom pose or (odom+fused_odem) poses. This flag informs a subscriber that the other poses should not be used.
# true: the ATR state is fully populated. This means the ATRs already published their own information and this has been used to create this message. All the pose types can be used now. 
bool full_state false

# Available pose in the state, see Pose Source Types below
int8 pose_source 

## GOAL and General State
# ATR Goal, final spose2d for the ATR
geometry_msgs/Pose goal

# General status of the ATR, e.g. ON_MISSION, DOCKED, etc.
atr_state_msgs/ATRStateOverall overall

# Mission status of the ATR, e.g. ARRIVED.
atr_state_msgs/ATRStateMission mission

# Loading state of the ATR, e.g. LOADING, UNLOADING, etc.
atr_state_msgs/ATRStateLoad load

# ATR Signal States. To define the signal type for the LED signaling system
atr_state_msgs/ATRStateSignals signal

# ATR Actuator. Information of the ATR's actuator
atr_state_msgs/ATRStateActuator actuator

# True if emergency stop is engaged for this robot
bool emerg_stop

# battery status
atr_state_msgs/ATRBatteryState battery

# collision state
atr_state_msgs/ATRCollisionState[] collisions

# error state
atr_error_msgs/ATRError error

# Pose Source types
# optom: only optom pose provided
int8 OPTOM=0
# odom: fused_odom/odom pose provided
int8 ODOM=1
# fused: fused_odom/odom pose provided
int8 FUSED_ODOM=2
# full: all the pose sources are available 
int8 FULL_POSE=3

