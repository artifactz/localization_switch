import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

from plugin_base import AbstractLocalizationSubscriber, get_relative_transform


class PoseSubscriber(AbstractLocalizationSubscriber):
    def __init__(self, pose_topic='/pose', timeout=1.):
        super(PoseSubscriber, self).__init__()
        self.enabled = False
        # params
        self.pose_topic = pose_topic
        self.sub_pose = rospy.Subscriber(self.pose_topic, PoseWithCovarianceStamped, self.pose_callback)

        self.last_pose = None

    def is_enabled(self):
        return self.enabled

    def pose_callback(self, msg):
        '''handles arrival of pose messages'''
        if self.last_pose is not None:
            # get difference
            delta_transform = get_relative_transform(
                self.last_pose.position, self.last_pose.orientation,
                msg.pose.pose.position, msg.pose.pose.orientation)

            # hand in the update
            self.callback(delta_transform)

        self.last_pose = msg.pose.pose

        self.enabled = True # TODO
