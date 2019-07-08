import rospy
from geometry_msgs.msg import Quaternion, Transform, Vector3
from sensor_msgs.msg import NavSatFix
import math
from tf.transformations import quaternion_from_euler

from plugin_base import AbstractLocalizationSubscriber
import plotting


def get_vector3_subtraction(a, b):
    '''subtracts an xyz-object `b` from `a` and returns the result as Vector3'''
    return Vector3(a.x - b.x, a.y - b.y, a.z - b.z)


def latlon_to_dxdy(lat1, lon1, lat2, lon2):
    '''computes the metric difference of two latitude/longitude pairs
       :type lat1 float
       :type lon1 float
       :type lat2 float
       :type lon2 float
       :rtype tuple(float, float)'''
    return (
        (lon1 - lon2) * 40008000. * math.cos((lat1 + lat2) * math.pi / 360.) / 360.,
        (lat1 - lat2) * 40008000. / 360.
    )


class GPSSubscriber(AbstractLocalizationSubscriber):
    '''subscribes to a NavSatFix topic and hands in delta transforms to its callback.
       lacks rotation due to the input only being GPS.'''
    def __init__(self, gps_topic='/mavros/global_position/global', initial_heading=0., **kwargs):
        super(GPSSubscriber, self).__init__(**kwargs)
        # params
        self.gps_topic = gps_topic

        self.orig_ll = None
        self.last_position = Vector3()

        if self.do_plot and initial_heading != 0.:
            q = Quaternion(*quaternion_from_euler(0, 0, initial_heading))
            plotting.add_transform(self, Transform(translation=Vector3(0., 0., 0.), rotation=q))

        self.sub_gps = rospy.Subscriber(self.gps_topic, NavSatFix, self.gps_callback)

    def is_enabled(self):
        '''GPS is only used for plotting'''
        return False

    def gps_callback(self, msg):
        '''handles arrival of NavSatFix messages'''
        if self.orig_ll is None:
            self.orig_ll = msg.latitude, msg.longitude

        # position difference
        x, y = latlon_to_dxdy(msg.latitude, msg.longitude, *self.orig_ll)
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
            plotting.add_transform(self, delta_transform)

        self.last_position = position
