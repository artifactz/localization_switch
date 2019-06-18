import rospy

import threading

import index
from plugin_base import AbstractController


class PriorityController(AbstractController):
    '''controller that picks the first enabled subscriber from a prioritized list'''
    def __init__(self, subscribers):
        ''':param list subscribers: subscriber dictionaries from a yaml config ordered by priority'''
        super(PriorityController, self).__init__()
        self.callback_mutex = threading.Lock()
        self.active_subscriber_idx = None
        # subscriber list sorted by descending priority
        self.subscribers = []
        for subscriber_dict in subscribers:
            self.append_subscriber(index.build_object(subscriber_dict))

    def __callback__(self, subscriber_idx, delta_transform):
        '''master callback for all subscribers'''
        with self.callback_mutex:
            self.update_active_subscriber()
            # only pass active transforms
            if self.is_active_subscriber(subscriber_idx):
                if self.callback is None:
                    rospy.logwarn('transform data is arriving, but the controller is not initialized yet')
                else:
                    self.callback(delta_transform)

    def append_subscriber(self, subscriber):
        '''adds an instance of an AbstractLocalizationSubscriber subclass.
           subscribers added earlier will have a higher priority.'''
        self.subscribers.append(subscriber)
        # set an individual callback
        idx = len(self.subscribers) - 1
        subscriber.set_callback(lambda delta: self.__callback__(idx, delta))

    def is_active_subscriber(self, subscriber_idx):
        '''returns True iff the specified subscriber is the one to use'''
        return self.active_subscriber_idx == subscriber_idx

    def update_active_subscriber(self):
        '''sets which subscriber shall be the chosen one ("active") based on priority and status'''
        for idx, subscriber in enumerate(self.subscribers):
            if subscriber.is_enabled():
                if not self.is_active_subscriber(idx):
                    self.switch_subscriber(idx)
                return
        self.active_subscriber_idx = None
        rospy.logwarn('PriorityController: no active subscriber')

    def switch_subscriber(self, subscriber_idx):
        '''sets the currently active subscriber by index'''
        assert(self.active_subscriber_idx != subscriber_idx)
        self.active_subscriber_idx = subscriber_idx
        rospy.loginfo('Controller: active subscriber is now %s' % type(self.subscribers[subscriber_idx]).__name__)
        # refresh global orientation if possible
        if self.subscribers[subscriber_idx].is_global_orientation and self.orientation_setter is not None:
            self.orientation_setter(self.subscribers[subscriber_idx].global_orientation)
