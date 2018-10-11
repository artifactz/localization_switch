#!/usr/bin/env python2

import rospy
import numpy as np
from tf.transformations import quaternion_multiply

from geometry_msgs.msg import Quaternion, Pose

from plugins.orbslam2_subscriber import ORBSLAM2Subscriber


def unpack_quaternion(q):
    '''to tuple'''
    return q.x, q.y, q.z, q.w

def get_quaternion(a):
    '''from iterable'''
    q = Quaternion()
    q.x, q.y, q.z, q.w = a
    return q

def increment_xyz(a, b):
    '''adds xyz-objects `a` and `b` and stores the result in `a`'''
    a.x += b.x
    a.y += b.y
    a.z += b.z



class LocalizationSwitch(object):
    def __init__(self, callback=None):
        '''`callback` can be a function of `Pose` to call on every update'''
        # subscriber list sorted by descending priority
        self.subscribers = []
        # pose to which the updates are applied
        self.pose = Pose()
        self.callback = callback

    def append_subscriber(self, subscriber):
        '''adds an instance of an AbstractLocalizationSubscriber subclass with.
           subscribers added earlier will have a higher priority.'''
        self.subscribers.append(subscriber)
        # set an individual callback
        idx = len(self.subscribers) - 1
        subscriber.set_callback(lambda delta: self.__callback__(idx, delta))

    def is_prior_disabled(self, idx):
        '''returns True iff all of the subscribers 0..idx-1 are disabled'''
        return all([not subscriber.is_enabled() for subscriber in self.subscribers[:idx]])

    def update_pose(self, transform):
        '''applies a `transform` to the internal pose'''
        increment_xyz(self.pose.position, transform.translation)
        self.pose.orientation = get_quaternion(quaternion_multiply(unpack_quaternion(self.pose.orientation), transform.rotation))

    def __callback__(self, subscriber_idx, delta_transform):
        '''master callback for all subscribers'''
        if self.is_prior_disabled(subscriber_idx):
            # update pose
            self.update_pose(delta_transform)
            # trigger publisher callback
            if self.callback:
                self.callback(self.pose)


class LocalizationSwitchNode(object):
    def __init__(self):
        rospy.init_node('localization_switch_node')
        # params
        self.output_pose_topic = '~pose'
        #rospy.get_param('~subscribers', '') # TODO: external subscriber config

        self.localization_switch = LocalizationSwitch(callback=self.pose_callback)
        # just add an ORBSLAM2Subscriber for now
        self.localization_switch.append_subscriber(ORBSLAM2Subscriber())

        # pose publisher
        self.pub_pose = rospy.Publisher(self.output_pose_topic, Pose, queue_size=10)

    def pose_callback(self, pose):
        '''handles the arrival of new poses from LocalizationSwitch'''
        # just publish the pose
        self.pub_pose.publish(pose)

    def spin(self):
        rospy.spin()


if __name__ == '__main__':
    node = LocalizationSwitchNode()
    node.spin()
