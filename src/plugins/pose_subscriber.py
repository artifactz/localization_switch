import rospy
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovariance, PoseWithCovarianceStamped, Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from plugin_base import AbstractLocalizationSubscriber, get_relative_transform, unpack_quaternion
import index
import plotting

import threading
import time


class PoseSubscriber(AbstractLocalizationSubscriber):
    '''subscribes to a PoseWithCovarianceStamped topic and hands in delta transforms to its callback'''
    def __init__(self, pose_topic='/pose', is_with_covariance=False, is_stamped=False, imu_subscriber=None,
                 timeout=0.5, **kwargs):
        ''':param dict imu_subscriber: ImuSubscriber yaml constructor dictionary'''
        self.is_roll_pitch_imu = imu_subscriber is not None
        super(PoseSubscriber, self).__init__(is_global_orientation=self.is_roll_pitch_imu, **kwargs)
        self.enabled = False
        # params
        self.pose_topic = pose_topic

        # set up IMU subscriber when needed
        if self.is_roll_pitch_imu:
            self.imu_subscriber = index.build_object(imu_subscriber)

        self.last_pose = None
        self.last_pose_time = None

        # choose matching pose type for the subscriber to use
        self.is_with_covariance = is_with_covariance
        self.is_stamped = is_stamped
        if is_with_covariance:
            pose_type = PoseWithCovarianceStamped if is_stamped else PoseWithCovariance
        else:
            pose_type = PoseStamped if is_stamped else Pose
        self.sub_pose = rospy.Subscriber(self.pose_topic, pose_type, self.pose_callback)

        # spawn timeout checker thread
        t = threading.Thread(target=self.check_timeout, args=(timeout,))
        t.daemon = True
        t.start()

    def is_enabled(self):
        return self.enabled

    def check_timeout(self, interval=0.5):
        while True:
            t0 = self.last_pose_time
            time.sleep(interval)
            if t0 == self.last_pose_time:  # didn't receive any new data
                # disable
                self.enabled = False
                self.last_pose = None
                self.last_pose_time = None

    def pose_callback(self, msg):
        '''handles arrival of pose messages'''
        # strip potential header and cov
        pose, stamp = (msg.pose, msg.header.stamp) if self.is_stamped else (msg, None)
        pose = pose.pose if self.is_with_covariance else pose

        # IMU enhanced mode
        if self.is_roll_pitch_imu:
            # check IMU availability
            imu = self.imu_subscriber.get_last_imu() if stamp is None else self.imu_subscriber.get_imu_at_time(stamp)
            if imu is None:
                rospy.logwarn('PoseSubscriber: IMU data unavailable. Suspending pose updates.')
                self.enabled = False
                return
            self.global_orientation = imu.orientation
            # replace roll and pitch with IMU values
            _, pitch, roll = euler_from_quaternion(unpack_quaternion(imu.orientation), axes='rzyx')
            yaw, _, _ = euler_from_quaternion(unpack_quaternion(pose.orientation), axes='rzyx')
            pose.orientation = Quaternion(*quaternion_from_euler(yaw, pitch, roll, axes='rzyx'))

        if self.last_pose is not None:
            # get difference
            delta_transform = get_relative_transform(
                self.last_pose.position, self.last_pose.orientation,
                pose.position, pose.orientation)

            self.enabled = True

            # hand in the update
            self.callback(delta_transform)
            if self.do_plot:
                plotting.add_transform(self, delta_transform)

        self.last_pose = pose
        self.last_pose_time = stamp
