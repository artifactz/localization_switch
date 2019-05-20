# coding=utf-8
import rospy
from tf.transformations import quaternion_multiply, quaternion_inverse, euler_from_quaternion
from collections import deque
import threading
import numpy as np

from plugin_base import unpack_quaternion
from priority_controller import PriorityController
import index


class TransformQueue(object):
    def __init__(self, duration):
        self.duration = duration
        self.q = deque()
        self.q_mutex = threading.Lock()
        self.__rotation_sum__ = None  # lazy getter

    def append_transform(self, delta_transform, timestamp):
        # TODO: keep track of rotation sum by adding/removing rotations?
        with self.q_mutex:
            self.q.append((delta_transform, timestamp))
            # check time difference between last and first item
            while (self.q[-1][1] - self.q[0][1]).to_sec() > self.duration:
                self.q.popleft()
        self.__rotation_sum__ = None

    def get_rotation_sum(self):
        '''returns a tuple of a rotation Quaternion and its duration in seconds'''
        with self.q_mutex:
            if self.__rotation_sum__ is not None:
                return self.__rotation_sum__
        result = [0, 0, 0, 1]
        if len(self.q) == 0:
            return result, 0
        with self.q_mutex:
            duration = (self.q[-1][1] - self.q[0][1]).to_sec()
            rotations = [unpack_quaternion(delta_transform.rotation) for delta_transform, _ in self.q]
        for rotation in rotations:
            result = quaternion_multiply(result, rotation)
        self.__rotation_sum__ = result, duration
        return result, duration


class ImuController(PriorityController):
    def __init__(self, subscribers, imu_subscriber, threshold=0.03, window=2.5):
        '''
        :param list subscribers: subscriber dictionaries from a yaml config ordered by priority
        :param TransformProvider imu_subscriber: IMU subscriber instance
        :param float threshold: the mean squared error across the three rotational DOF (in radians per second) from
                                which on to switch to the next subscriber (0.03 = 0.175^2 + 0 + 0, 0.175 rad/s = 10°/s)
        :param float window: time frame for error averaging
        '''
        self.threshold = threshold
        self.window = window
        self.imu_transforms = TransformQueue(window)
        self.subscriber_transforms = []
        self.subscriber_errors = []
        self.active_subscriber_idx = None
        super(ImuController, self).__init__(subscribers)
        self.imu_subscriber = index.build_object(imu_subscriber)
        self.imu_subscriber.set_callback(self.imu_callback)

    def append_subscriber(self, subscriber):
        super(ImuController, self).append_subscriber(subscriber)
        self.subscriber_transforms.append(TransformQueue(self.window))
        self.subscriber_errors.append(0.)

    def imu_callback(self, delta_transform):
        '''callback for IMU updates'''
        self.imu_transforms.append_transform(delta_transform, rospy.get_rostime())

    def __callback__(self, subscriber_idx, delta_transform):
        '''master callback for all subscribers'''
        # store transform
        self.subscriber_transforms[subscriber_idx].append_transform(delta_transform, rospy.get_rostime())
        # update error
        self.update_subscriber_error(subscriber_idx)
        # submit an update if `is_active_subscriber`
        super(ImuController, self).__callback__(subscriber_idx, delta_transform)

    def update_subscriber_error(self, subscriber_idx):
        '''updates the subscriber error with the mean squared error across the three euler angles wrt. IMU readings'''
        imu_rot, imu_dur = self.imu_transforms.get_rotation_sum()
        sub_rot, sub_dur = self.subscriber_transforms[subscriber_idx].get_rotation_sum()
        delta_rot = quaternion_multiply(quaternion_inverse(imu_rot), sub_rot)
        # TODO: MSE without euler? norm - 1? https://math.stackexchange.com/a/2581782
        delta_rot_euler = euler_from_quaternion(delta_rot)
        total_err = np.dot(delta_rot_euler, delta_rot_euler) / 3.
        # error per second
        errps = np.inf if imu_dur + sub_dur == 0 else total_err / ((imu_dur + sub_dur) / 2.)
        self.subscriber_errors[subscriber_idx] = errps

        rospy.loginfo_throttle(10, 'ImuController: current errors are %s' % self.subscriber_errors)

        # also update which subscribers is the chosen one ("active")
        # first: do they actually work right now
        enabled = [sub.is_enabled() for sub in self.subscribers]
        # build list of minimum error of enabled subscribers from index to end
        min_err = [np.inf] * (len(self.subscribers) + 1)
        for i in range(len(min_err) - 2, -1, -1):
            min_err[i] = min(self.subscriber_errors[i], min_err[i + 1]) if enabled[i] else min_err[i + 1]
        # determine active subscriber
        for i in range(len(self.subscribers)):
            if enabled[i]:
                if (
                    self.subscriber_errors[i] < self.threshold      # error is ok
                    # or i == len(self.subscribers) - 1               # last subscriber has the smallest error
                    or self.subscriber_errors[i] < min_err[i + 1]   # following errors are even worse
                ):
                    if self.subscriber_errors[i] >= self.threshold:
                        rospy.loginfo_throttle(
                            1,
                            'ImuController: choosing %s despite err > threshold (lowest fallback error: %.4f)'
                            % (type(self.subscribers[i]).__name__, min_err[i + 1])
                        )
                    if self.active_subscriber_idx != i:
                        rospy.loginfo('ImuController: active subscriber is now %s' % type(self.subscribers[i]).__name__)
                    self.active_subscriber_idx = i
                    return
        # all subscribers are inactive
        self.active_subscriber_idx = None

    def is_active_subscriber(self, subscriber_idx):
        ''':returns bool: True iff the specified subscriber is the one to use'''
        return subscriber_idx == self.active_subscriber_idx
