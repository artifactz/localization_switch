import rospy
import tf2_ros
from orb_slam2_ros.msg import ORBState
from orb_slam2_ros.srv import ResetSystem

import threading
from collections import deque

from plugin_base import AbstractLocalizationSubscriber, get_relative_transform
import plotting


class ORBSLAM2Subscriber(AbstractLocalizationSubscriber):
    '''subscribes to an ORBState topic, polls ORBSLAM2 TFs and hands in delta transforms to its callback.
       requests an ORBSLAM2 reset when lost.'''
    def __init__(self, timeout_reset=5., **kwargs):
        '''`timeout_reset`: sets after how many seconds of ORBState.LOST to call a system_reset'''
        super(ORBSLAM2Subscriber, self).__init__(**kwargs)
        self.enabled = False
        # params
        self.base_link_frame_id = '/ORB_base_link' # '/orb_slam2/camera'
        self.world_frame_id = '/orb_slam2/map' # '/orb_slam2/world'
        self.state_topic = '/orb_slam2/state'
        self.update_rate = 30.  # TODO: ros params
        self.timeout_reset = timeout_reset

        self.srv_reset = None
        # detach waiting for services from main thread, so everything works without them as well
        threading.Thread(target=self.__init_services__).start()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # TF2 doesn't like link names starting with "/"
        self.base_link_frame_id = self.base_link_frame_id.strip('/')
        self.world_frame_id = self.world_frame_id.strip('/')

        self.tf_queue = deque()
        # time frame of 'ok' state
        self.last_ok_time = None
        self.first_ok_time = None
        # timeout_reset cooldown counter
        self.last_reset_time = None

        self.sub_state = rospy.Subscriber(self.state_topic, ORBState, self.state_callback)

        # detach TF polling from main thread, so __init__() can finish
        self.tf_thread = threading.Thread(target=self.poll_tf)
        self.tf_thread.start()

    def __init_services__(self):
        '''waits and inits service proxyies'''
        rospy.wait_for_service('/orb_slam2/reset_system')
        self.srv_reset = rospy.ServiceProxy('/orb_slam2/reset_system', ResetSystem)
        rospy.loginfo('ORBSLAM2Subscriber: system_reset service ready')

    def try_reset(self):
        '''attempts to reset ORB_SLAM2'''
        try:
            self.sub_state.unregister()
            self.disable()
            self.srv_reset()
            self.sub_state = rospy.Subscriber(self.state_topic, ORBState, self.state_callback)
            self.last_reset_time = rospy.get_rostime()
            rospy.loginfo('ORBSLAM2Subscriber: requested system_reset')
        except (rospy.ServiceException, TypeError):   # self.srv_reset could be None
            rospy.logerr('ORBSLAM2Subscriber: system_reset service not ready')

    def is_enabled(self):
        # TODO: timeout for not receiving any state messages?
        return self.enabled

    def disable(self):
        if self.enabled:
            rospy.loginfo('ORBSLAM2Subscriber disabled')
        self.first_ok_time = None
        self.enabled = False
        # TODO: clear tf queue?

    def state_callback(self, msg):
        '''handles arrival of ORBState messages, validates queued TFs and processes them'''
        if msg.state == ORBState.OK:
            # update 'ok' time frame
            if self.first_ok_time is None:
                self.first_ok_time = msg.header.stamp
            self.last_ok_time = msg.header.stamp

            while len(self.tf_queue) > 1:  # need two or moar for a diff
                # discard an invalid TF
                if self.tf_queue[0].header.stamp < self.first_ok_time:
                    self.tf_queue.popleft()
                    continue

                # pending 'ok' state
                if self.tf_queue[0].header.stamp > self.last_ok_time or self.tf_queue[1].header.stamp > self.last_ok_time:
                    # wait for clearance
                    return

                # assume sequential timestamps
                assert(self.tf_queue[0].header.stamp <= self.tf_queue[1].header.stamp)

                # everything seems to be working fine
                if not self.enabled:
                    rospy.loginfo('ORBSLAM2Subscriber enabled')
                self.enabled = True

                # get TFs and let's go
                tf1 = self.tf_queue.popleft().transform
                tf2 = self.tf_queue[0].transform

                # get difference
                delta_transform = get_relative_transform(tf1.translation, tf1.rotation, tf2.translation, tf2.rotation)

                # hand in the update
                self.callback(delta_transform)
                if self.do_plot:
                    plotting.add_transform(self, delta_transform)

                rospy.logdebug('ORBSLAM2Subscriber: tf processed')

        else:
            # not 'ok'
            self.disable()

    def poll_tf(self):
        '''TF polling loop'''
        rate = rospy.Rate(self.update_rate)
        while not rospy.is_shutdown():
            rate.sleep()

            # long time no see? TODO: is_time_for_reset()?
            if (
                self.last_ok_time is not None and
                rospy.get_rostime().to_sec() - self.last_ok_time.to_sec() > self.timeout_reset and (
                    self.last_reset_time is None or
                    rospy.get_rostime().to_sec() - self.last_reset_time.to_sec() > self.timeout_reset
                )
            ):
                # restart SLAM
                self.try_reset()

            try:
                transform = self.tf_buffer.lookup_transform(self.world_frame_id, self.base_link_frame_id, rospy.Time(0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logdebug('ORBSLAM2Subscriber: tf exception')
                continue    # go to sleep

            if len(self.tf_queue) > 0 and self.tf_queue[-1] == transform:
                # old data: go to sleep
                rospy.logdebug('ORBSLAM2Subscriber: tf unchanged')
                continue

            self.tf_queue.append(transform)
