import rospy
from geometry_msgs.msg import Quaternion, Transform, Vector3
from sensor_msgs.msg import NavSatFix
from alvinxy import alvinxy

from plugin_base import AbstractLocalizationSubscriber
import plotting


def get_vector3_subtraction(a, b):
    '''subtracts an xyz-object `b` from `a` and returns the result as Vector3'''
    return Vector3(a.x - b.x, a.y - b.y, a.z - b.z)


class GPSSubscriber(AbstractLocalizationSubscriber):
    '''subscribes to a NavSatFix topic and hands in delta transforms to its callback.
       lacks rotation due to the input only being GPS.'''
    def __init__(self, initial_heading=0.): # TODO
        super(GPSSubscriber, self).__init__(plot)
        # params
        self.gps_topic = '/mavros/global_position/global'
        self.sub_gps = rospy.Subscriber(self.gps_topic, NavSatFix, self.gps_callback)

        self.orig_ll = None
        self.last_position = Vector3()

    def is_enabled(self):
        '''GPS is only used for plotting'''
        return False

    def gps_callback(self, msg):
        '''handles arrival of NavSatFix messages'''
        if self.orig_ll is None:
            self.orig_ll = msg.latitude, msg.longitude

        # position difference
        x, y = alvinxy.ll2xy(msg.latitude, msg.longitude, *self.orig_ll)
        z = msg.altitude
        position = Vector3(x, y, z)
        translation = get_vector3_subtraction(position, self.last_position)
        # construct a transform
        delta_transform = Transform(
            translation=translation,
            rotation=Quaternion(0, 0, 0, 1) # The commonly-used unit quaternion that yields no rotation about the x/y/z axes is (0,0,0,1) [http://wiki.ros.org/tf2/Tutorials/Quaternions]
        )

        # hand in the update
        self.callback(delta_transform)
        if self.do_plot:
            plotting.add_pose(self, delta_transform)

        self.last_position = position
