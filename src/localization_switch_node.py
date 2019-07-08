#!/usr/bin/env python2

import rospy
import tf
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Point
import yaml
import sys

import plugins.plotting
import plugins.index
from plugins.plugin_base import transform_pose, unpack_quaternion


def get_mavros_pose():
    ''':returns: the TF from local_origin to fcu as a `Pose`'''
    tf_listener = tf.TransformListener()
    for _ in xrange(3):
        try:
            trans, rot = tf_listener.lookupTransform('local_origin', 'fcu', rospy.Time(0))
            return Pose(Point(*trans), Quaternion(*rot))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.sleep(.5)  # 0.5 seconds
    raise RuntimeError('cannot receive MAVROS TF')


class PoseManager(object):
    '''holds the root controller, keeps track of an internal pose and calls a callback on every update'''
    def __init__(self, controller, pose=None, callback=None, set_global_orientation=False, plot_pose=False):
        ''':param controller: controller to be used containing all necessary subscribers
           :param geometry_msgs.msg.Pose pose: initial pose (`None` initializes to 0-pose)
           :param function callback: function of :class:`Pose` to call on every update
           :param bool set_global_orientation: whether to sync the internal orientation with a subscriber providing a
                                               global orientation before using its pose updates
           :param bool plot_pose: whether to plot the internal pose'''
        # pose to which the updates are applied
        if pose is None:
            self.pose = Pose()  # orientation may be all 0, which is ok (`quaternion_matrix` still returns id)
        else:
            self.pose = pose
        # start controller
        self.controller = controller
        self.controller.set_callback(self.__callback__)
        if set_global_orientation:
            self.controller.set_orientation_setter(self.__set_orientation__)
        # external (publisher) callback function to call
        self.callback = callback
        self.do_plot = plot_pose
        if self.do_plot:
            plugins.plotting.start()

    def update_pose(self, delta_transform):
        '''applies a transformation to the internal pose
           :param geometry_msgs.msg.Transform delta_transform: pose update'''
        transform_pose(self.pose, delta_transform)
        if self.do_plot:
            plugins.plotting.add_transform(self, delta_transform)

    def __callback__(self, delta_transform):
        '''receives updates from the controller
           :param geometry_msgs.msg.Transform delta_transform: pose update'''
        # update pose
        self.update_pose(delta_transform)
        # trigger publisher callback
        if self.callback:
            self.callback(self.pose)

    def __set_orientation__(self, orientation):
        '''updates the orientation independently of the current pose
           :param Quaternion orientation: absolute orientation'''
        # copy values
        self.pose.orientation = Quaternion(*unpack_quaternion(orientation))
        if self.do_plot:
            plugins.plotting.add_orientation(self, orientation)
        rospy.loginfo("orientation set to global")


class LocalizationSwitchNode(object):
    '''reads ROS params, initializes the LocalizationSwitch and keeps publishing the internal pose'''
    def __init__(self):
        rospy.init_node('localization_switch_node')  # log_level=rospy.DEBUG
        # params
        output_pose_topic = rospy.get_param('~output_pose_topic', '~pose')
        self.pose_frame_id = rospy.get_param('~output_pose_frame_id', 'world')
        plot_pose = rospy.get_param('~plot_pose', False)
        yaml_string = rospy.get_param('~controller', '')
        use_initial_mavros_pose = rospy.get_param('~use_initial_mavros_pose', True)
        set_global_orientation = rospy.get_param('~set_global_orientation', False)

        if use_initial_mavros_pose:
            initial_pose = get_mavros_pose()
        else:
            initial_pose = None

        # build controller
        yaml_dict = yaml.load(yaml_string)
        controller = plugins.index.build_object(yaml_dict)
        if controller:
            self.localization_switch = PoseManager(
                pose=initial_pose,
                controller=controller,
                callback=self.pose_callback,
                set_global_orientation=set_global_orientation,
                plot_pose=plot_pose
            )
        else:
            sys.exit('no controller specified')

        # pose publisher
        self.pub_pose = rospy.Publisher(output_pose_topic, PoseStamped, queue_size=10)

    def pose_callback(self, pose):
        '''handles the arrival of new poses (geometry_msgs/Pose) from LocalizationSwitch'''
        # just publish the pose
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.get_rostime()  # TODO: real timestamps would be nice
        pose_stamped.header.frame_id = self.pose_frame_id
        pose_stamped.pose = pose
        self.pub_pose.publish(pose_stamped)

    def spin(self):
        rospy.spin()


if __name__ == '__main__':
    node = LocalizationSwitchNode()
    node.spin()
