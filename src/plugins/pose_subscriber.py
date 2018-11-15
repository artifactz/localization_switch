import rospy
from geometry_msgs.msg import Quaternion, Transform, Vector3, Pose, PoseWithCovarianceStamped
from tf.transformations import quaternion_multiply, quaternion_from_euler, euler_from_quaternion

from plugin_base import AbstractLocalizationSubscriber


def unpack_quaternion(q):
    '''to tuple'''
    return q.x, q.y, q.z, q.w

def get_vector3_subtraction(a, b):
    '''subtracts an xyz-object `b` from `a` and returns the result as Vector3'''
    return Vector3(a.x - b.x, a.y - b.y, a.z - b.z)


class PoseSubscriber(AbstractLocalizationSubscriber):
    def __init__(self, timeout=1., initial_yaw=None):
        super(PoseSubscriber, self).__init__()
        self.enabled = False
        # params
        self.pose_topic = '/simple_pixel_flow/pose'
        self.sub_pose = rospy.Subscriber(self.pose_topic, PoseWithCovarianceStamped, self.pose_callback)

        self.last_pose = None

    def is_enabled(self):
        return self.enabled

    def pose_callback(self, msg):
        '''handles arrival of pose messages'''
        if self.last_pose is not None:
            # position difference
            translation = get_vector3_subtraction(msg.pose.pose.position, self.last_pose.position)
            # orientation difference
            q1_inv = self.last_pose.orientation
            q1_inv = Quaternion(q1_inv.x, q1_inv.y, q1_inv.z, -q1_inv.w)    # copy and inverse
            rot = quaternion_multiply(unpack_quaternion(q1_inv), unpack_quaternion(msg.pose.pose.orientation))
            # construct a transform
            delta_transform = Transform(translation=translation, rotation=rot)
            # hand in the update
            self.callback(delta_transform)

        self.last_pose = msg.pose.pose

        self.enabled = True # TODO
