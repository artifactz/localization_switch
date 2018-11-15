#!/usr/bin/env python2

import rospy
import numpy as np
import matplotlib.pyplot as plt
from tf.transformations import quaternion_multiply, rotation_matrix
from geometry_msgs.msg import Quaternion, Point, Pose, Vector3
import threading

from plugins.orbslam2_subscriber import ORBSLAM2Subscriber


color_palette = ['#0055AA', '#CC0022', '#DDB800', '#007728', '#009FEE', '#AA008E']


def get_plot_color(subscriber_idx, enabled):
    '''returns a subsciber's color, which is darker/grayer when disabled.'''
    c = color_palette[subscriber_idx % len(color_palette)]
    if not enabled:
        # mix with 50% gray
        r, g, b = map(lambda s: int(s, 16), [c[1:3], c[3:5], c[5:7]])
        c = '#%X%X%X' % tuple(map(int, [.5 * r + 64, .5 * g + 64, .5 * b + 64]))
    return c

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

def transform_pose(pose, transform):
    '''adds translation to position, multiplies rotation to orientation'''
    increment_xyz(pose.position, transform.translation)
    pose.orientation = get_quaternion(quaternion_multiply(unpack_quaternion(pose.orientation), transform.rotation))

def copy_pose(pose):
    p = pose.position
    o = pose.orientation
    return Pose(position=Point(p.x, p.y, p.z), orientation=Quaternion(o.x, o.y, o.z, o.w))


class LocalizationSwitch(object):
    def __init__(self, callback=None, plot_mode=False):
        '''`callback` can be a function of `Pose` to call on every update'''
        # subscriber list sorted by descending priority
        self.subscribers = []
        # pose to which the updates are applied
        self.pose = Pose()  # TODO: orientation 0,0,0,1?
        self.callback = callback

        self.plot_mode = plot_mode
        if plot_mode:
            # internal pose (0th item) and all subscriber poses (following) over the course of time
            self.pose_history = [[copy_pose(self.pose)]]
            # status (whether it's been used as the internal pose) of all subscriber poses over the course of time
            self.enabled_history = []
            # angle to manually correct alignment
            self.plot_rotations = []
            # detach plotting from main thread
            self.plot_thread = threading.Thread(target=self.plot_positions)
            self.plot_thread.start()

    def append_subscriber(self, subscriber, plot_yaw=None):
        '''adds an instance of an AbstractLocalizationSubscriber subclass.
           subscribers added earlier will have a higher priority.
           `plot_yaw` can depict a plot rotation.'''
        self.subscribers.append(subscriber)
        if self.plot_mode:
            r = None if plot_yaw is None else rotation_matrix(plot_yaw, [0, 0, 1])
            self.plot_rotations.append(r)
            self.pose_history.append([Pose()])
            self.enabled_history.append([True])
        # set an individual callback
        idx = len(self.subscribers) - 1
        subscriber.set_callback(lambda delta: self.__callback__(idx, delta))

    def is_prior_disabled(self, idx):
        '''returns True iff all of the subscribers 0..idx-1 are disabled'''
        return all([not subscriber.is_enabled() for subscriber in self.subscribers[:idx]])

    def is_active_subscriber(self, idx):
        '''returns True iff the specified subscriber is the one to use'''
        return self.is_prior_disabled(idx) and self.subscribers[idx].is_enabled()

    def update_pose(self, transform):
        '''applies a `transform` to the internal pose'''
        transform_pose(self.pose, transform)
        if self.plot_mode:
            # store a copy of the current internal pose
            self.pose_history[0].append(copy_pose(self.pose))

    def __callback__(self, subscriber_idx, delta_transform):
        '''master callback for all subscribers'''
        if self.plot_mode == 'positions':
            # rotate transform if needed
            if self.plot_rotations[subscriber_idx] is not None:
                t = delta_transform.translation
                translation_hom = [t.x, t.y, t.z, 1.]
                transformed_hom = np.dot(self.plot_rotations[subscriber_idx], translation_hom)
                delta_transform.translation = Vector3(*(transformed_hom[:3]))
            # transform last pose to get the current one
            pose = copy_pose(self.pose_history[subscriber_idx + 1][-1])
            transform_pose(pose, delta_transform)
            # store
            self.pose_history[subscriber_idx + 1].append(pose)
            self.enabled_history[subscriber_idx].append(self.is_active_subscriber(subscriber_idx))

        if self.is_active_subscriber(subscriber_idx):
            # update pose
            self.update_pose(delta_transform)
            # trigger publisher callback
            if self.callback:
                self.callback(self.pose)

    def __plot_positions__(self, sub_idx, start_idx=0, end_idx=None, enabled=True):
        '''helper function: dispatches a plot call for a path segment'''
        if end_idx is None:
            end_idx = len(self.pose_history[sub_idx + 1]) - 1

        xs = [self.pose_history[sub_idx + 1][i].position.x for i in xrange(start_idx, end_idx + 1)]
        ys = [self.pose_history[sub_idx + 1][i].position.y for i in xrange(start_idx, end_idx + 1)]
        if sub_idx == -1:
            label = 'merged'
        else:
            label = type(self.subscribers[sub_idx]).__name__ if start_idx == 0 else None
        plt.plot(
            xs, ys,
            color=get_plot_color(sub_idx, enabled),
            label=label,
            linewidth=2
        )

    def plot_positions(self):
        '''plots a planar projection of all gathered poses'''
        plt.axes().set_aspect('equal', 'datalim')
        while True:
            # clear
            plt.cla()

            for sub_idx in range(len(self.subscribers)):
                # there is no point in drawing less then two points
                if len(self.pose_history[sub_idx + 1]) < 2:
                    continue

                # segment path based on being enabled
                start_idx = 0
                prev_enabled = None
                for pose_idx, enabled in enumerate(self.enabled_history[sub_idx]):
                    if (prev_enabled is not None and prev_enabled != enabled) or pose_idx == len(self.pose_history[sub_idx + 1]) - 1:
                        self.__plot_positions__(sub_idx, start_idx, pose_idx, prev_enabled)
                        start_idx = pose_idx
                    prev_enabled = enabled

            # plot merged path
            self.__plot_positions__(-1)

            plt.legend()
            plt.pause(1)


class LocalizationSwitchNode(object):
    def __init__(self):
        rospy.init_node('localization_switch_node')
        # params
        self.output_pose_topic = '~pose'
        #rospy.get_param('~subscribers', '') # TODO: external subscriber config

        self.localization_switch = LocalizationSwitch(callback=self.pose_callback, plot_mode=True)
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
