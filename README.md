# `localization_switch`

*ROS localization method multiplexer*

This is a slim ROS library written in Python able to bundle multiple localization methods. It subscribes to their output topic, keeps track of their state and applies the pose transformations of the best (based on priority) currently available method to a resulting pose which is republished on every update.

## Running this node

See the [example launch file](launch/default.launch), which contains default values for all parameters except `subscribers`.

### Published topic

`/localization_switch/pose` (`geometry_msgs/PoseStamped`)  
The resulting pose.

### Node parameters

`output_pose_topic` (`string`, default: `~pose`)  
Overrides the output topic name.

`plot_mode` (`bool`, default: `false`)  
Opens a window to draw the paths of all subscribed localization methods and the path of the resulting pose in real time.

`subscribers` (`string`, default: empty string)  
A YAML string defining the subscriber setup. The order of appearance resembles their priority with the first entry being the most important one. An entry is a dictionary, defining at least a valid localization method `type` and optionally any additional parameters the method allows. Currently only three localization methods are supported: `PoseSubscriber`, `ORBSLAM2Subscriber` and `GPSSubscriber`.

### PoseSubscriber parameters

`pose_topic` (`string`, default: `/pose`)  
Specifies the pose topic to subscribe to.  
**Note**: Only `PoseWithCovarianceStamped` messages are supported at the moment.

### ORBSLAM2Subscriber parameters

`timeout_reset` (`float`, default: `5.0`)  
Specifies the time needed for the ORBSLAM2 state not to be `OK` (mainly `LOST`) to request a ORBSLAM2 restart.

## Extending this node

Additional subscriber types may be added in `src/plugins/` by inheriting from `AbstractLocalizationSubscriber` and adding them to the [plugin index](src/plugins/index.py).

## Dependencies

* Python 2, NumPy, matplotlib, basic ROS packages
* `geonav_transform` ROS package for the `GPSSubscriber`
* [ORBSLAM2 with ROS extensions](https://gitlab.tubit.tu-berlin.de/breakdowncookie/ORB_SLAM2.git)
