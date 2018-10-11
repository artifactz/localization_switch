import rospy
from tf.msg import tfMessage
from orb_slam2_ros.msg import ORBState
from geometry_msgs.msg import Quaternion, Transform, Vector3
from tf.transformations import quaternion_multiply

from plugin_base import AbstractLocalizationSubscriber


def unpack_quaternion(q):
    '''to tuple'''
    return q.x, q.y, q.z, q.w

def get_vector3_subtraction(a, b):
    '''subtracts an xyz-object `b` from `a` and returns the result as Vector3'''
    return Vector3(a.x - b.x, a.y - b.y, a.z - b.z)


class ORBSLAM2Subscriber(AbstractLocalizationSubscriber):
    def __init__(self, timeout=1.):
        super(ORBSLAM2Subscriber, self).__init__()
        self.enabled = False
        # params
        self.frame_id = '/orb_slam2/camera'
        self.state_topic = '/orb_slam2/state'
        # ROS tf listener doesn't support callbacks, so subscribe to /tf
        self.sub_tf = rospy.Subscriber('/tf', tfMessage, self.tf_callback)
        self.sub_state = rospy.Subscriber(self.state_topic, ORBState, self.state_callback)

        self.last_tf = None

    def is_enabled(self):
        # TODO: timeout
        return self.enabled

    def state_callback(self, msg):
        '''handles arrival of ORBState messages'''
        if msg.state == ORBState.OK:
            if not self.enabled:
                print 'ORBSLAM2Subscriber enabled'
            self.enabled = True
        else:
            self.enabled = False
            if self.enabled:
                print 'ORBSLAM2Subscriber disabled'

    def tf_callback(self, msg):
        '''handles arrival of tf messages'''
        for transform in msg.transforms:
            # look for an ORBSLAM2 TF
            if transform.child_frame_id == self.frame_id:
                if self.last_tf is not None:
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

                self.last_tf = transform
