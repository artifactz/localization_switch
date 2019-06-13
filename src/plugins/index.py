'''module to manage available plugins (controllers and subscribers).
   add/remove them by modifying imports and `get_subscriber_from_string` function accordingly.'''

from orbslam2_subscriber import ORBSLAM2Subscriber
from orbslam2_pose_subscriber import ORBSLAM2PoseSubscriber
from gps_subscriber import GPSSubscriber
from pose_subscriber import PoseSubscriber
from imu_subscriber import ImuSubscriber
from priority_controller import PriorityController
from imu_controller import ImuController


def build_object(yaml_dict):
    ''':param dict yaml_dict: dictionary containing ("type": plugin_type)
       :returns: instance of `plugin_type` with all the other dict items passed as kwargs'''
    if yaml_dict is None:
        return None
    t = get_type_from_string(yaml_dict['type'])
    del yaml_dict['type']
    return t(**yaml_dict)


def get_type_from_string(name):
    '''converts a subscriber name string to its actual type'''
    types = {
        'PoseSubscriber': PoseSubscriber,
        'GPSSubscriber': GPSSubscriber,
        'ORBSLAM2Subscriber': ORBSLAM2Subscriber,
        'ORBSLAM2PoseSubscriber': ORBSLAM2PoseSubscriber,
        'ImuSubscriber': ImuSubscriber,
        'PriorityController': PriorityController,
        'ImuController': ImuController
    }
    try:
        return types[name]
    except KeyError:
        raise RuntimeError('unknown plugin type: %s' % name)
