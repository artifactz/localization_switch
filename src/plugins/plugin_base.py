from geometry_msgs.msg import Quaternion, Transform, Vector3, Pose, Point
from tf.transformations import quaternion_multiply, quaternion_inverse, quaternion_matrix, quaternion_from_matrix
import numpy as np

import plotting


def copy_pose(pose):
    ''':param Pose pose: pose to copy
       :returns Pose: copy of pose'''
    p = pose.position
    o = pose.orientation
    return Pose(position=Point(p.x, p.y, p.z), orientation=Quaternion(o.x, o.y, o.z, o.w))


def unpack_quaternion(q):
    ''':returns tuple: quaternion values'''
    return q.x, q.y, q.z, q.w


def translation_rotation_to_matrix(translation, rotation):
    '''returns an affine transformation matrix from a Vector3 or Point and a geometry_msgs/Quaternion'''
    mat = quaternion_matrix(unpack_quaternion(rotation))
    mat[:3, 3] = [translation.x, translation.y, translation.z]
    return mat


def transform_pose(pose, delta_transform):
    '''applies a delta transform (geometry_msgs/Transform) to a pose (geometry_msgs/Pose)'''
    # convert to affine transformation matrices
    pose_mat = translation_rotation_to_matrix(pose.position, pose.orientation)
    trans_mat = translation_rotation_to_matrix(delta_transform.translation, delta_transform.rotation)
    # apply
    new_pose_mat = np.dot(pose_mat, trans_mat)
    # convert back to pose
    pose.position = Vector3(*new_pose_mat[:3, 3])
    pose.orientation = Quaternion(*quaternion_from_matrix(new_pose_mat))


def get_relative_transform(ref_position, ref_orientation, translation, rotation):
    '''returns a pose (geometry_msgs/Transform) from the perspective of a reference pose, i.e. a "delta transform".
       `ref_position` and `translation` may be given as any xyz-object (e.g. geometry_msgs/Vector3).
       `ref_orientation` and `rotation` are quaternions (geometry_msgs/Quaternion).'''
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


class TransformProvider(object):
    '''base class for anything that can hand in delta transforms (subscribers and controllers at the moment)'''
    def __init__(self):
        self.callback = None

    def set_callback(self, callback):
        ''':param function callback: callback to hand in :class:`geometry_msgs.msg.Transform` updates'''
        self.callback = callback


class AbstractLocalizationSubscriber(TransformProvider):
    '''base class for subscribers'''
    def __init__(self, plot=False):
        super(AbstractLocalizationSubscriber, self).__init__()
        self.do_plot = plot
        if self.do_plot:
            plotting.start()

    def is_enabled(self):
        '''whether to use this subscriber or not'''
        raise NotImplementedError
