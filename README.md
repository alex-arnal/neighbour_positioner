# Neighbour Positioner

Node to publish the relative position of the robot (*base_frame*) with respect to a *fixed_frame* and to publish the position of other robots as prohibited areas.

---

## Parameters

- **neighbours_topic**: Name of the topic where the position of the neighbours realtive to the *fixed_frame* are published.
- **local_position_topic**: Topic where the transformation between the *fixed_frame* and the *base_frame* is published.
- **fixed_frame**: Fixed frame used as reference (world, map, odom, etc.). It's shared by all the neighbours.
- **base_frame**: Frame of the robot used to get the relative position (base_link, base_footrpint, etc.).
- **prohibition_layer_srv**: Name of the service to publish the position of the neighbours as prohibition areas.