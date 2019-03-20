import index
from plugin_base import TransformProvider


class PriorityController(TransformProvider):
    def __init__(self, subscribers):
        ''':param list subscribers: subscriber dictionaries from a yaml config ordered by priority'''
        super(PriorityController, self).__init__()
        # subscriber list sorted by descending priority
        self.subscribers = []
        for subscriber_dict in subscribers:
            self.append_subscriber(index.build_object(subscriber_dict))

    def __callback__(self, subscriber_idx, delta_transform):
        '''master callback for all subscribers'''
        if self.is_active_subscriber(subscriber_idx):
            self.callback(delta_transform)

    def append_subscriber(self, subscriber):
        '''adds an instance of an AbstractLocalizationSubscriber subclass.
           subscribers added earlier will have a higher priority.'''
        self.subscribers.append(subscriber)
        # set an individual callback
        idx = len(self.subscribers) - 1
        subscriber.set_callback(lambda delta: self.__callback__(idx, delta))

    def is_prior_disabled(self, idx):
        '''returns True iff all of the subscribers 0..idx-1 are disabled'''
        return all([not subscriber.is_enabled() for subscriber in self.subscribers[:idx]])

    def is_active_subscriber(self, idx):
        '''returns True iff the specified subscriber is the one to use'''
        return self.is_prior_disabled(idx) and self.subscribers[idx].is_enabled()
