import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped
from orb_slam2_ros.msg import ORBState
from orb_slam2_ros.srv import ResetSystem

import threading

from plugin_base import AbstractLocalizationSubscriber, get_relative_transform, transform_pose
import plotting


class ORBSLAM2PoseSubscriber(AbstractLocalizationSubscriber):
    '''subscribes to the ORBSLAM2 Pose topic and hands in delta transforms to its callback'''
    def __init__(self, pose_topic='/orb_slam2/pose', state_topic='/orb_slam2/state', base_link_tf='base_link',
                 camera_tf='camera', timeout_reset=5., **kwargs):
        ''':param float timeout_reset: after how many seconds of ORBState.LOST to call a system_reset'''
        super(ORBSLAM2PoseSubscriber, self).__init__(**kwargs)
        self.enabled = False
        # params
        self.pose_topic = pose_topic
        self.state_topic = state_topic
        self.timeout_reset = timeout_reset
        self.base_link_tf = base_link_tf.strip('/')  # TF2 doesn't like frame names starting with "/"
        self.camera_tf = camera_tf.strip('/')

        self.srv_reset = None
        # detach waiting for services from main thread, so everything works without them as well
        threading.Thread(target=self.__init_services__).start()

        # get TFs once
        self.base_link_to_camera_tf = None
        self.camera_to_base_link_tf = None
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)
        nb_fails = 0
        while self.base_link_to_camera_tf is None and self.camera_to_base_link_tf is None:
            try:
                if self.base_link_to_camera_tf is None:
                    self.base_link_to_camera_tf = tf_buffer.lookup_transform(
                        self.base_link_tf, self.camera_tf, rospy.Time(0), rospy.Duration(.25)
                    )
                if self.camera_to_base_link_tf is None:
                    self.camera_to_base_link_tf = tf_buffer.lookup_transform(
                        self.camera_tf, self.base_link_tf, rospy.Time(0), rospy.Duration(.25)
                    )
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                nb_fails += 1
                if nb_fails >= 5:
                    rospy.logwarn_throttle(10, 'ORBSLAM2PoseSubscriber: unable to determine camera TF, retrying...')
        tf_listener.unregister()

        rospy.loginfo('ORBSLAM2PoseSubscriber: camera TF OK')

        self.last_ok_time = None
        self.last_reset_time = None

        # pose subscription
        self.last_cam_pose = None
        self.sub_pose = rospy.Subscriber(self.pose_topic, PoseStamped, self.pose_callback)
        # state subscription
        self.sub_state = rospy.Subscriber(self.state_topic, ORBState, self.state_callback)

        # keep ORBSLAM2 alive
        threading.Thread(target=self.check_restart).start()

    def __init_services__(self):
        '''waits and initializes service proxies'''
        rospy.wait_for_service('/orb_slam2/reset_system')
        self.srv_reset = rospy.ServiceProxy('/orb_slam2/reset_system', ResetSystem)
        rospy.loginfo('ORBSLAM2PoseSubscriber: system_reset service OK')

    def disable(self):
        if self.enabled:
            rospy.loginfo('ORBSLAM2PoseSubscriber disabled')
        self.last_cam_pose = None
        self.enabled = False

    def try_reset(self):
        '''attempts to reset ORB_SLAM2'''
        try:
            # clear message queues by destroying the subscribers
            self.sub_state.unregister()
            self.sub_pose.unregister()
            self.disable()
            self.srv_reset()
            # revive subscribers
            self.sub_state = rospy.Subscriber(self.state_topic, ORBState, self.state_callback)
            self.sub_pose = rospy.Subscriber(self.pose_topic, PoseStamped, self.pose_callback)
            self.last_reset_time = rospy.get_rostime()
            rospy.loginfo('ORBSLAM2PoseSubscriber: requested system_reset')
        except (rospy.ServiceException, TypeError):   # self.srv_reset could be None
            rospy.logerr('ORBSLAM2PoseSubscriber: system_reset service not ready')

    def pose_callback(self, msg):
        '''handles arrival of pose messages
           :type msg: PoseStamped'''
        cam_pose = msg.pose

        # two camera poses are available
        if self.last_cam_pose is not None:
            # actually not the base_link poses, but camera poses from perspective of base_link frame
            last_base_pose = tf2_geometry_msgs.do_transform_pose(
                PoseStamped(pose=self.last_cam_pose), self.base_link_to_camera_tf).pose
            base_pose = tf2_geometry_msgs.do_transform_pose(
                PoseStamped(pose=cam_pose), self.base_link_to_camera_tf).pose
            # transform to actual base_link poses (still in base_link frame)
            transform_pose(last_base_pose, self.camera_to_base_link_tf.transform)
            transform_pose(base_pose, self.camera_to_base_link_tf.transform)

            # get difference
            delta_transform = get_relative_transform(
                last_base_pose.position, last_base_pose.orientation, base_pose.position, base_pose.orientation)

            # everything seems to be working fine
            if not self.enabled:
                rospy.loginfo('ORBSLAM2PoseSubscriber enabled')
                self.enabled = True

            # hand in the update
            self.callback(delta_transform)
            if self.do_plot:
                plotting.add_transform(self, delta_transform)

        self.last_cam_pose = cam_pose

    def state_callback(self, msg):
        '''handles arrival of ORBState messages
           :type msg: ORBState'''
        if msg.state == ORBState.OK:
            self.last_ok_time = msg.header.stamp
        else:
            # not 'ok'
            self.disable()
            # TODO: not sync with pose messages, could provoke short state oscillations

    def is_enabled(self):
        return self.enabled

    def check_restart(self):
        '''keeps checking for OK state in a loop and resets ORBSLAM2 in case of a timeout'''
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            rate.sleep()

            # long time no see?
            if (
                self.last_ok_time is not None and  # state was OK at some point
                rospy.get_rostime().to_sec() - self.last_ok_time.to_sec() > self.timeout_reset and (  # timeout
                    self.last_reset_time is None or  # never reset before
                    rospy.get_rostime().to_sec() - self.last_reset_time.to_sec() > self.timeout_reset  # reset long ago
                )
            ):
                # restart SLAM
                self.try_reset()
