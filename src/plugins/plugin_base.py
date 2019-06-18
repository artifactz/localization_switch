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
    ''':param Quaternion q: quaternion object
       :returns: quaternion values
       :rtype: list'''
    return [q.x, q.y, q.z, q.w]


def translation_rotation_to_matrix(translation, rotation):
    '''returns an affine transformation matrix from a Vector3 or Point and a geometry_msgs/Quaternion'''
    mat = quaternion_matrix(unpack_quaternion(rotation))
    mat[:3, 3] = [translation.x, translation.y, translation.z]
    return mat


def transform_pose(pose, delta_transform):
    '''applies a transform, e.g. the output of `get_relative_transform`,
       to a pose intrinsically, i.e. returns `pose` * `delta_transform`
       :type pose: Pose
       :type delta_transform: Transform'''
    # convert to affine transformation matrices
    pose_mat = translation_rotation_to_matrix(pose.position, pose.orientation)
    trans_mat = translation_rotation_to_matrix(delta_transform.translation, delta_transform.rotation)
    # apply
    new_pose_mat = np.dot(pose_mat, trans_mat)  # left to right: transform is applied intrinsically
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
    rotation = quaternion_multiply(q1_i, q2)  # q2 = q1 * rotation -> q1^-1 * q2 = rotation
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


class AbstractTransformProvider(object):
    '''base class for anything that can hand in delta transforms (subscribers and controllers at the moment)'''
    def __init__(self):
        self.callback = None

    def set_callback(self, callback):
        ''':param function callback: callback to hand in :class:`geometry_msgs.msg.Transform` updates'''
        self.callback = callback


class AbstractController(AbstractTransformProvider):
    '''base class for controllers'''
    def __init__(self):
        super(AbstractController, self).__init__()
        self.orientation_setter = None

    def set_callback(self, callback):
        super(AbstractController, self).set_callback(callback)

    def set_orientation_setter(self, orientation_setter):
        ''':param function orientation_setter: function to set an absolute orientation, e.g. when switching
                                               subscribers, to avoid position drift'''
        self.orientation_setter = orientation_setter


class AbstractLocalizationSubscriber(AbstractTransformProvider):
    '''base class for subscribers'''
    def __init__(self, is_global_orientation=False, plot=False, alias=None):
        ''':param is_global_orientation: flags this subscriber to provide absolute (IMU-like) orientation
           :param plot: flags this subscriber to be drawn in the plot
           :param alias: if set, uses this string instead of the type name to label the subscriber'''
        super(AbstractLocalizationSubscriber, self).__init__()
        self.is_global_orientation = is_global_orientation
        self.do_plot = plot
        self.alias = alias
        if self.is_global_orientation:
            self.global_orientation = None
        if self.do_plot:
            plotting.start()

    def is_enabled(self):
        '''whether to use this subscriber or not'''
        raise NotImplementedError
