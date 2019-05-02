import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Transform, Vector3
from tf.transformations import quaternion_from_euler

from plugin_base import AbstractLocalizationSubscriber
import plotting


def get_quaternion(a):
    ''':param iter a: quaternion values
       :returns Quaternion: quaternion object'''
    q = Quaternion()
    q.x, q.y, q.z, q.w = a
    return q


class ImuSubscriber(AbstractLocalizationSubscriber):
    '''subscribes to an Imu topic and hands in delta transforms to its callback.'''
    def __init__(self, **kwargs):
        super(ImuSubscriber, self).__init__(**kwargs)
        # params
        self.gravity = 9.80665
        self.imu_topic = '/mavros/imu/data'
        self.last_imu_time = None
        self.sub_imu = rospy.Subscriber(self.imu_topic, Imu, self.imu_callback)

    def is_enabled(self):
        '''good guy IMU: always there for ya'''
        return True

    def imu_callback(self, msg):
        '''handles arrival of Imu messages'''
        if self.last_imu_time is None:
            self.last_imu_time = msg.header.stamp
        else:
            ts = msg.header.stamp
            dt = (ts - self.last_imu_time).to_sec()
            translation = Vector3()  # TODO: consider integrating linear acceleration
            av = msg.angular_velocity
            rotation = quaternion_from_euler(dt * av.x, dt * av.y, dt * av.z)
            delta_transform = Transform(translation=translation, rotation=get_quaternion(rotation))
            self.last_imu_time = ts

            # hand in the update
            self.callback(delta_transform)
            if self.do_plot:
                plotting.add_pose(self, delta_transform)
