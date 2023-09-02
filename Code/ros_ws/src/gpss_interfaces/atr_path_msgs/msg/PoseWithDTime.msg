# ATR pose, linear (x,y,z) and angular (quaternion)
geometry_msgs/Pose pose

# Time to reach "pose" in [sec]
float64  delta_time

# TODO: Add Node ID to track which node this pose belongs to.
# Each time a pose crosses a Node the Node ID should be changed to that Node
# At the beggining, the Node ID should be the starting Node