import rospy
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovariance, PoseWithCovarianceStamped

from plugin_base import AbstractLocalizationSubscriber, get_relative_transform
import plotting


class PoseSubscriber(AbstractLocalizationSubscriber):
    '''subscribes to a PoseWithCovarianceStamped topic and hands in delta transforms to its callback'''
    def __init__(self, pose_topic='/pose', is_with_covariance=False, is_stamped=False, timeout=1., **kwargs):  # TODO: implement timeout
        super(PoseSubscriber, self).__init__(**kwargs)
        self.enabled = False
        # params
        self.pose_topic = pose_topic
        # pose type
        self.is_with_covariance = is_with_covariance
        self.is_stamped = is_stamped
        if is_with_covariance:
            pose_type = PoseWithCovarianceStamped if is_stamped else PoseWithCovariance
        else:
            pose_type = PoseStamped if is_stamped else Pose
        self.sub_pose = rospy.Subscriber(self.pose_topic, pose_type, self.pose_callback)

        self.last_pose = None

    def is_enabled(self):
        return self.enabled

    def pose_callback(self, msg):
        '''handles arrival of pose messages'''
        # strip potential header and cov
        pose = msg.pose if self.is_stamped else msg
        pose = pose.pose if self.is_with_covariance else pose
        if self.last_pose is not None:
            # get difference
            delta_transform = get_relative_transform(
                self.last_pose.position, self.last_pose.orientation,
                pose.position, pose.orientation)

            # hand in the update
            self.callback(delta_transform)
            if self.do_plot:
                plotting.add_pose(self, delta_transform)

        self.last_pose = pose

        self.enabled = True
