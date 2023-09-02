# This is the ATR's full state (2D pose and 2D linear and angular velocitites) 
# to be used in the controller [x,y,\theta_z,v_x,v_y,\omega_z]
# We need this structure to pre-allocate the message in a pool. 
float64[3] pose
float64[3] velocity
float64[3] desired_pose
float64[3] desired_velocity
float64[3] pose_error
float64[3] velocity_error
float64[3] cmd_vel

# Internal PID Data
float64[2] wheel_velocity
float64[2] desired_wheel_velocity
float64[2] p_error
float64[2] i_error
float64[2] d_error