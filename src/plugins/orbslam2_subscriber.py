import rospy
import tf2_ros
from tf.msg import tfMessage
from orb_slam2_ros.msg import ORBState
from geometry_msgs.msg import Quaternion, Transform, Vector3
from tf.transformations import quaternion_multiply

import threading

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
    def __init__(self, timeout=1.):
        super(ORBSLAM2Subscriber, self).__init__()
        self.enabled = False
        # params
        self.base_link_frame_id = '/ORB_base_link' # '/orb_slam2/camera'
        self.world_frame_id = '/orb_slam2/map' # '/orb_slam2/world'
        self.state_topic = '/orb_slam2/state'
        self.update_rate = 30.

        self.sub_state = rospy.Subscriber(self.state_topic, ORBState, self.state_callback)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # TF2 doesn't like link names starting with "/"
        self.base_link_frame_id = self.base_link_frame_id.strip('/')
        self.world_frame_id = self.world_frame_id.strip('/')

        self.last_tf = None

        # detach TF polling from main thread, so __init__() can finish
        self.update_thread = threading.Thread(target=self.spin)
        self.update_thread.start()

    def is_enabled(self):
        # TODO: timeout
        return self.enabled

    def state_callback(self, msg):
        '''handles arrival of ORBState messages'''
        if msg.state == ORBState.OK:
            if not self.enabled:
                rospy.loginfo('ORBSLAM2Subscriber enabled')
            self.enabled = True
        else:
            self.last_tf = None # avoid jumps
            if self.enabled:
                rospy.loginfo('ORBSLAM2Subscriber disabled')
            self.enabled = False

    def spin(self):
        rate = rospy.Rate(self.update_rate)
        while not rospy.is_shutdown():
            rate.sleep()

            try:
                transform = self.tf_buffer.lookup_transform(self.world_frame_id, self.base_link_frame_id, rospy.Time(0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logdebug('ORBSLAM2Subscriber: tf exception')
                continue    # go to sleep

            if self.last_tf is not None:
                if self.last_tf == transform:
                    # old data: go to sleep
                    rospy.logdebug('ORBSLAM2Subscriber: tf unchanged')
                    continue
                # position difference
                trans = get_vector3_subtraction(transform.transform.translation, self.last_tf.transform.translation)
                # orientation difference
                q1_inv = self.last_tf.transform.rotation
                q1_inv = Quaternion(q1_inv.x, q1_inv.y, q1_inv.z, -q1_inv.w)
                rot = quaternion_multiply(unpack_quaternion(q1_inv), unpack_quaternion(transform.transform.rotation))
                # construct a transform
                delta_transform = Transform(translation=trans, rotation=rot)
                # hand in the update
                self.callback(delta_transform)

                rospy.logdebug('ORBSLAM2Subscriber: tf processed')

            self.last_tf = transform
