# `localization_switch`

*ROS localization method multiplexer*

This is a slim ROS library written in Python able to bundle multiple localization
methods. It subscribes to their output topic, keeps track of their state and chooses
the best currently available one based on priority or error. The pose transformations
of this method are applied intrinsically to a resulting pose, which is republished on
every update.

## Running this node

See the [example launch file](launch/default.launch), which contains default values
for all parameters and a basic set of `subscribers` managed by an `ImuController`.

### Published topic

`/localization_switch/pose` (`geometry_msgs/PoseStamped`)  
The resulting pose.

### Node parameters

`controller` (`string`, default: empty string)  
A YAML inline object string defining the controller setup including its subscribers.
Each object defines at least its `type` and optionally additional attributes, see
the controller parameter sections respectively.  
Currently available controller types are `PriorityController` and `ImuController`.

`use_initial_mavros_pose` (`bool`, default: `true`)  
When enabled, the internal pose is initialized to the TF `local_origin -> fcu`.

`set_global_orientation` (`bool`, default: `false`)  
If enabled, whenever switching to a localization subscriber providing global
(IMU-like) orientation, the internal pose's orientation is overwritten by that
orientation to reset orientation drift accumulated by non-global orientation methods.

`output_pose_topic` (`string`, default: `~pose`)  
Overrides the output topic name.

`output_pose_frame_id` (`string`, default: `world`)  
Sets the `frame_id` in the output pose header.

`plot_mode` (`bool`, default: `false`)  
Opens a window to draw the trajectories of all subscribed localization methods and the
trajectory of the resulting pose in real time.

### PriorityController parameters

`subscribers` (`string`, default: empty string)  
A YAML inline array string defining the localization subscribers to set up. The order
of appearance resembles their priority with the first entry being the most important
one. Each entry is an object defining at least a valid localization method `type` and
optionally any additional parameters the method allows.  
Currently available localization method types are `ORBSLAM2PoseSubscriber`,
`PoseSubscriber`, `ORBSLAM2Subscriber`, `GPSSubscriber` and `ImuSubscriber`.

### ImuController parameters

`subscribers` (`string`, default: empty string)  
See `PriorityController`.

`imu_subscriber` (`string`, default: empty string)  
A YAML inline object string defining an `ImuSubscriber` used to compute rotation
errors between the subscribers and the IMU.

`window` (`float`, default: 2.5)  
Time window in seconds for which to sum up rotations.

`threshold` (`float`, default: 0.03)  
Error threshold from which on to suspend a localization method and switch to the next
one in line. If the error of all subscribers is exceeding the threshold, the
subscriber with the smallest error is chosen. The error is computed as the mean
squared error over the three euler angles in radians per second. 0.03 corresponds to
an error of 10 degrees per second around one axis.

### PoseSubscriber parameters

`pose_topic` (`string`, default: `/pose`)  
Specifies the pose topic to subscribe to.

`is_with_covariance` (`bool`, default: `false`)  
Specifies the pose type. Set to true for `PoseWithCovarianceStamped` or
`PoseWithCovariance`.

`is_stamped` (`bool`, default: `false`)  
Specifies the pose type. Set to true for `PoseWithCovarianceStamped` or
`PoseStamped`.

`imu_subscriber` (`string`, default: empty string)  
A YAML inline object string defining an `ImuSubscriber` with `imu_log_duration > 0`.
When set, roll and pitch of the processed pose are replaced by the IMU's roll and
pitch. The `PoseSubscriber` then qualifies to provide global orientation (see
`set_global_orientation`).

### ORBSLAM2PoseSubscriber parameters

(This is currently the preferred method to process ORBSLAM2 localization.)

`pose_topic` (`string`, default: `/orb_slam2/pose`)  
Specifies the ORBSLAM2 pose topic to subscribe to.

`state_topic` (`string`, default: `/orb_slam2/state`)  
Specifies the ORBSLAM2 state topic to subscribe to.

`base_link_tf` (`string`, default: `base_link`)  
Specifies the vehicle's root frame.

`camera_tf` (`string`, default: `camera`)  
Specifies the frame of the camera used for ORBSLAM2.

`timeout_reset` (`float`, default: `5.0`)  
Specifies the time needed for the ORBSLAM2 state not to be `OK` (mainly `LOST`) to
request a ORBSLAM2 restart.

### ORBSLAM2Subscriber parameters

`timeout_reset` (`float`, default: `5.0`)  
See `ORBSLAM2PoseSubscriber`.

### ImuSubscriber parameters

(This subscriber currently does not provide translation, only rotation.)

`imu_topic` (`string`, default: `/mavros/imu/data`)  
Specifies the IMU topic to subscribe to.

`imu_log_duration` (`float`, default: `0.0`)  
Length of IMU message queue in seconds. The IMU queue is used for global orientation
synchronization.

### GPSSubscriber parameters

`gps_topic` (`string`, default: `/mavros/global_position/global`)  
Specifies the `NavSatFix` topic to subscribe to.

`initial_heading` (`float`, default: `0.0`)  
Sets the yaw component of the initial orientation (in radians) to manually align the
GPS trajectory with other ones.

## Extending this node

Additional subscriber and controller types may be added in `src/plugins/` by
inheriting from `AbstractLocalizationSubscriber`, `AbstractController` or available
subscriber/controller classes and adding them to the
[plugin index](src/plugins/index.py).

## Dependencies

* Python 2, NumPy, matplotlib, basic ROS packages
* [ORBSLAM2 with ROS extensions](https://gitlab.tubit.tu-berlin.de/breakdowncookie/ORB_SLAM2.git)
