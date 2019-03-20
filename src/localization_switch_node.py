#!/usr/bin/env python2

import rospy
from geometry_msgs.msg import Pose, PoseStamped
import yaml
import sys

import plugins.plotting
import plugins.index
from plugins.plugin_base import transform_pose


class PoseManager(object):
    '''holds the root controller, keeps track of an internal pose and calls a callback on every update'''
    def __init__(self, controller, callback=None, plot_pose=False):
        ''':param controller: controller to be used containing all necessary subscribers
           :param function callback: function of :class:`Pose` to call on every update
           :param bool plot_pose: whether to plot the internal pose'''
        self.controller = controller
        self.controller.set_callback(self.__callback__)
        # pose to which the updates are applied
        self.pose = Pose()  # TODO: orientation 0,0,0,1?
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
            plugins.plotting.add_pose(self, delta_transform)

    def __callback__(self, delta_transform):
        '''receives updates from the controller
           :param geometry_msgs.msg.Transform delta_transform: pose update'''
        # update pose
        self.update_pose(delta_transform)
        # trigger publisher callback
        if self.callback:
            self.callback(self.pose)


class LocalizationSwitchNode(object):
    '''reads ROS params, initializes the LocalizationSwitch and keeps publishing the internal pose'''
    def __init__(self):
        rospy.init_node('localization_switch_node')  # log_level=rospy.DEBUG
        # params
        output_pose_topic = rospy.get_param('~output_pose_topic', '~pose')
        plot_pose = rospy.get_param('~plot_pose', False)
        yaml_string = rospy.get_param('~controller', '')

        # build controller
        yaml_dict = yaml.load(yaml_string)
        controller = plugins.index.build_object(yaml_dict)
        if controller:
            self.localization_switch = PoseManager(
                controller=controller,
                callback=self.pose_callback,
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
        pose_stamped.header.frame_id = 'world'
        pose_stamped.pose = pose
        self.pub_pose.publish(pose_stamped)

    def spin(self):
        rospy.spin()


if __name__ == '__main__':
    node = LocalizationSwitchNode()
    node.spin()
