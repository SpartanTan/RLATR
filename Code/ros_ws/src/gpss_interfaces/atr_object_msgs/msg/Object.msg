# Polygon with the 2D points forming the object
# Th obejct can be described by the 2D footprint or the 3D bounding box
geometry_msgs/Polygon polygon

# Object Centre (Polygon Centroid)
geometry_msgs/Point32 centroid

# 6D pose of the object. The frame id is usally taken from the header
geometry_msgs/Pose pose

# Object class, e.g. forklift, human, unknown, etc.
atr_object_msgs/ObjectClass object_c

# Type of object, e.g., static, dynamic, non-accesible area (nonaa), etc
atr_object_msgs/ObjectType object_t

# Object ID
int16 object_id

# Index of this object in the array of Objects. This variable can be use to create an <index, objec_id> map 
int16 object_idx