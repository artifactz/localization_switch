import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Transform, Vector3
from tf.transformations import quaternion_from_euler

import threading
from collections import deque

from plugin_base import AbstractLocalizationSubscriber, get_relative_transform
import plotting


def get_quaternion(a):
    ''':param iter a: quaternion values
       :returns Quaternion: quaternion object'''
    q = Quaternion()
    q.x, q.y, q.z, q.w = a
    return q


class ImuSubscriber(AbstractLocalizationSubscriber):
    '''subscribes to an Imu topic and hands in delta transforms to its callback.'''
    def __init__(self, imu_topic='/mavros/imu/data', imu_log_duration=0, **kwargs):
        super(ImuSubscriber, self).__init__(is_global_orientation=True, **kwargs)
        # params
        self.imu_topic = imu_topic

        self.imu_queue = deque()
        self.imu_queue_mutex = threading.Lock()
        self.imu_log_duration = rospy.Duration(imu_log_duration)
        self.is_imu_log_enabled = imu_log_duration > 0

        self.last_imu = None
        self.sub_imu = rospy.Subscriber(self.imu_topic, Imu, self.imu_callback)

    def is_enabled(self):
        '''IMU is not used for localization'''
        return False

    def imu_callback(self, msg):
        '''handles arrival of Imu messages
           :type msg: Imu'''
        # maintain IMU log for `get_imu_at_time`
        if self.is_imu_log_enabled:
            with self.imu_queue_mutex:
                self.imu_queue.append(msg)
                # discard old messages
                while self.imu_queue[0].header.stamp + self.imu_log_duration < msg.header.stamp:
                    self.imu_queue.popleft()

        # provide rotation
        if self.callback is not None and self.last_imu is not None:

            # check if `orientation` is available (it's a bit smoother than `angular_velocity`)
            if self.last_imu.orientation_covariance[0] != -1 and msg.orientation_covariance[0] != -1:
                delta_transform = get_relative_transform(
                    Vector3(), self.last_imu.orientation, Vector3(), msg.orientation)

            else:
                # integrate angular velocities instead
                dt = (msg.header.stamp - self.last_imu.header.stamp).to_sec()
                translation = Vector3()  # TODO: consider integrating linear acceleration
                av = msg.angular_velocity
                rotation = quaternion_from_euler(dt * av.x, dt * av.y, dt * av.z)
                delta_transform = Transform(translation=translation, rotation=get_quaternion(rotation))

            # hand in the update
            self.callback(delta_transform)
            if self.do_plot:
                plotting.add_transform(self, delta_transform)

        self.last_imu = msg

    def get_imu_at_time(self, timestamp):
        '''picks the IMU message with best matching timestamp from the queue
           :type timestamp: rospy.Time
           :rtype: Imu'''
        with self.imu_queue_mutex:
            # abort if the queue is empty
            if len(self.imu_queue) == 0:
                return None
            # warn if the requested timestamp is out of bounds
            if timestamp < self.imu_queue[0].header.stamp or timestamp > self.imu_queue[-1].header.stamp:
                stamps = tuple(map(lambda t: t.to_sec(), [self.imu_queue[0].header.stamp, self.imu_queue[-1].header.stamp, timestamp]))
                rospy.logwarn('ImuSubscriber: timestamp of requested IMU data is out of bounds: [%.2f, %.2f] but %.2f' % stamps)
            # find best
            return min(self.imu_queue, key=lambda msg: abs(msg.header.stamp - timestamp))

    def get_last_imu(self):
        ''':rtype: Imu'''
        with self.imu_queue_mutex:
            # abort if the queue is empty
            if len(self.imu_queue) == 0:
                return None
            return self.imu_queue[-1]
