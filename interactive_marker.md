---
layout: default
title: Inertactive-marker
---

## Interactive Marker

### Purpose:
control tf broadcasting in rviz via interactive marker.

### Usage:

1. bring up the rviz only mode (dragon example)

```
$ roslaunch dragon bringup.launch real_machine:=false simulation:=false headless:=false
```

2. run the interactive marker
```
$ rosrun aerial_robot_model interactive_marker_tf_broadcaster
```

3. manipulate in rviz:
![interactive marker](images/interactive_marker.gif)

### Advanced:
The default reference frame is called `fixed_frame`, and the target frame is assigned to `root`.
You can change the reference frame to any fixed frame, such as the target end effector point in environment.
You can also change the target frame to the end-effector.




