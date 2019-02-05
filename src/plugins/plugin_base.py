from geometry_msgs.msg import Quaternion, Transform, Vector3
from tf.transformations import quaternion_multiply, quaternion_inverse, quaternion_matrix
import numpy as np


def unpack_quaternion(q):
    '''to tuple'''
    return q.x, q.y, q.z, q.w

def get_relative_transform(ref_position, ref_orientation, translation, rotation):
    '''returns a transform/pose from the perspective of reference transform/pose'''
    # rotation
    q1_i = quaternion_inverse(unpack_quaternion(ref_orientation))
    q2 = unpack_quaternion(rotation)
    rotation = quaternion_multiply(q1_i, q2)
    # translation in numpy affine
    t1 = ref_position
    t2 = translation
    delta_translation = np.asarray([t2.x - t1.x, t2.y - t1.y, t2.z - t1.z, 1])
    # rotate translation
    delta_translation = np.dot(quaternion_matrix(q1_i), delta_translation)
    return Transform(
        translation=Vector3(*delta_translation[:3]),
        rotation=Quaternion(*rotation)
    )


class AbstractLocalizationSubscriber(object):
    '''base class for subscribers'''
    def __init__(self):
        self.callback = None

    def set_callback(self, callback):
        '''sets function to call whenever there is an update'''
        self.callback = callback

    def is_enabled(self):
        '''whether to use this subscriber or resort to a lower priority one'''
        raise NotImplementedError
