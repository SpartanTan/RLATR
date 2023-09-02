# This is the ATR's full state (2D pose and 2D linear and angular velocitites) 
# to be used in the controller [x,y,\theta_z,v_x,v_y,\omega_z]
# We need this structure to pre-allocate the message in a pool. 
float64[6] full_state