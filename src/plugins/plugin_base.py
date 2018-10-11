class AbstractLocalizationSubscriber(object):
    def __init__(self):
        self.callback = None

    def set_callback(self, callback):
        '''sets function to call whenever there is an update'''
        self.callback = callback

    def is_enabled(self):
        '''whether to use this subscriber or resort to a lower priority one'''
        raise NotImplementedError
