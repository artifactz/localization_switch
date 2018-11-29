import rospy
import tf2_ros
from tf.msg import tfMessage
from orb_slam2_ros.msg import ORBState
from orb_slam2_ros.srv import ResetSystem
from geometry_msgs.msg import Quaternion, Transform, Vector3
from tf.transformations import quaternion_multiply

import threading
from collections import deque

from plugin_base import AbstractLocalizationSubscriber


def unpack_quaternion(q):
    '''to tuple'''
    return q.x, q.y, q.z, q.w

def get_list_subtraction(a, b):
    '''subtracts a Vector3-like list `b` from `a` and returns the result as an actual Vector3'''
    return Vector3(a[0] - b[0], a[1] - b[1], a[2] - b[2])

def get_vector3_subtraction(a, b):
    '''subtracts an xyz-object `b` from `a` and returns the result as Vector3'''
    return Vector3(a.x - b.x, a.y - b.y, a.z - b.z)


class ORBSLAM2Subscriber(AbstractLocalizationSubscriber):
    def __init__(self, timeout_reset=5.):
        '''`timeout_reset`: sets after how many seconds of ORBState.LOST to call a system_reset'''
        super(ORBSLAM2Subscriber, self).__init__()
        self.enabled = False
        # params
        self.base_link_frame_id = '/ORB_base_link' # '/orb_slam2/camera'
        self.world_frame_id = '/orb_slam2/map' # '/orb_slam2/world'
        self.state_topic = '/orb_slam2/state'
        self.update_rate = 30.
        self.timeout_reset = timeout_reset

        self.sub_state = rospy.Subscriber(self.state_topic, ORBState, self.state_callback)

        self.srv_reset = None
        # detach waiting for services from main thread, so everything works without them as well
        threading.Thread(target=self.__init_services__).start()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # TF2 doesn't like link names starting with "/"
        self.base_link_frame_id = self.base_link_frame_id.strip('/')
        self.world_frame_id = self.world_frame_id.strip('/')

        self.tf_queue = deque()
        # time frame of 'ok' state
        self.last_ok_time = None
        self.first_ok_time = None
        # timeout_reset cooldown counter
        self.last_reset_time = None

        # detach TF polling from main thread, so __init__() can finish
        self.tf_thread = threading.Thread(target=self.poll_tf)
        self.tf_thread.start()

    def __init_services__(self):
        '''waits and inits service proxyies'''
        rospy.wait_for_service('/orb_slam2/reset_system')
        self.srv_reset = rospy.ServiceProxy('/orb_slam2/reset_system', ResetSystem)
        rospy.loginfo('ORBSLAM2Subscriber: system_reset service ready')

    def try_reset(self):
        '''attempts to reset ORB_SLAM2'''
        try:
            self.sub_state.unregister()
            self.disable()
            self.srv_reset()
            self.sub_state = rospy.Subscriber(self.state_topic, ORBState, self.state_callback)
            self.last_reset_time = rospy.get_rostime()
            rospy.loginfo('ORBSLAM2Subscriber: requested system_reset')
        except (rospy.ServiceException, TypeError):   # self.srv_reset could be None
            rospy.logerr('ORBSLAM2Subscriber: system_reset service not ready')

    def is_enabled(self):
        # TODO: timeout for not receiving any state messages?
        return self.enabled

    def disable(self):
        if self.enabled:
            rospy.loginfo('ORBSLAM2Subscriber disabled')
        self.first_ok_time = None
        self.enabled = False
        # TODO: clear tf queue?

    def state_callback(self, msg):
        '''handles arrival of ORBState messages, validates queued TFs and processes them'''
        if msg.state == ORBState.OK:
            # update 'ok' time frame
            if self.first_ok_time is None:
                self.first_ok_time = msg.header.stamp
            self.last_ok_time = msg.header.stamp

            while len(self.tf_queue) > 1: # need two or moar for a diff
                # discard an invalid TF
                if self.tf_queue[0].header.stamp < self.first_ok_time:
                    self.tf_queue.popleft()
                    continue

                # pending 'ok' state
                if self.tf_queue[0].header.stamp > self.last_ok_time or self.tf_queue[1].header.stamp > self.last_ok_time:
                    # wait for clearance
                    return

                # assume sequential timestamps
                assert(self.tf_queue[0].header.stamp <= self.tf_queue[1].header.stamp)

                # everything seems to be working fine
                if not self.enabled:
                    rospy.loginfo('ORBSLAM2Subscriber enabled')
                self.enabled = True

                # get TFs and let's go
                tf1 = self.tf_queue.popleft()
                tf2 = self.tf_queue[0]

                # position difference
                trans = get_vector3_subtraction(tf2.transform.translation, tf1.transform.translation)

                # orientation difference
                q1_inv = tf1.transform.rotation
                q1_inv = Quaternion(q1_inv.x, q1_inv.y, q1_inv.z, -q1_inv.w)
                rot = quaternion_multiply(unpack_quaternion(q1_inv), unpack_quaternion(tf2.transform.rotation))
                # construct a transform
                delta_transform = Transform(translation=trans, rotation=rot)
                # hand in the update
                self.callback(delta_transform)

                rospy.logdebug('ORBSLAM2Subscriber: tf processed')

        else:
            # not 'ok'
            self.disable()

    def poll_tf(self):
        '''TF polling loop'''
        rate = rospy.Rate(self.update_rate)
        while not rospy.is_shutdown():
            rate.sleep()

            # long time no see? TODO: is_time_for_reset()?
            if (
                self.last_ok_time is not None and
                rospy.get_rostime().to_sec() - self.last_ok_time.to_sec() > self.timeout_reset and (
                    self.last_reset_time is None or
                    rospy.get_rostime().to_sec() - self.last_reset_time.to_sec() > self.timeout_reset
                )
            ):
                # restart SLAM
                self.try_reset()

            try:
                transform = self.tf_buffer.lookup_transform(self.world_frame_id, self.base_link_frame_id, rospy.Time(0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logdebug('ORBSLAM2Subscriber: tf exception')
                continue    # go to sleep

            if len(self.tf_queue) > 0 and self.tf_queue[-1] == transform:
                # old data: go to sleep
                rospy.logdebug('ORBSLAM2Subscriber: tf unchanged')
                continue

            self.tf_queue.append(transform)
