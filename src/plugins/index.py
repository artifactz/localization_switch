'''module to manage available plugins/subscribers. add/remove by modifying imports and `get_subscriber_from_string` function accordingly.'''

from orbslam2_subscriber import ORBSLAM2Subscriber
from gps_subscriber import GPSSubscriber
from pose_subscriber import PoseSubscriber


def get_subscriber_from_string(name):
    '''converts a subscriber name string to its actual type'''
    types = {
        'PoseSubscriber': PoseSubscriber,
        'GPSSubscriber': GPSSubscriber,
        'ORBSLAM2Subscriber': ORBSLAM2Subscriber
    }
    try:
        return types[name]
    except KeyError:
        raise RuntimeError('unknown subscriber type: %s' % name)
