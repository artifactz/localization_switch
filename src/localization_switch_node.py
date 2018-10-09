#!/usr/bin/env python2

import rospy
import numpy as np
from tf.transformations import quaternion_multiply

from orb_slam2_ros.msg import ORBState
from tf.msg import tfMessage
from geometry_msgs.msg import Quaternion, Transform, Pose, Vector3


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

def get_vector3_subtraction(a, b):
    '''subtracts an xyz-object `b` from `a` and returns the result as Vector3'''
    return Vector3(a.x - b.x, a.y - b.y, a.z - b.z)


class AbstractLocalizationSubscriber(object):
    def __init__(self):
        self.callback = None

    def set_callback(self, callback):
        '''sets function to call whenever there is an update'''
        self.callback = callback

    def is_enabled(self):
        '''whether to use this subscriber or resort to a lower priority one'''
        raise NotImplementedError


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
