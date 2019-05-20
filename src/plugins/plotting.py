import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import threading

from geometry_msgs.msg import Pose

from plugin_base import copy_pose, transform_pose, AbstractLocalizationSubscriber


color_palette = iter(['#0055AA', '#CC0022', '#DDB800', '#007728', '#009FEE', '#AA008E'])


class Plotter(object):
    def __init__(self):
        self.pose_history = {}
        self.color = {}
        # status (whether it's been used as the internal pose) of all subscriber poses over the course of time
        self.enabled_history = {}

    def add_pose(self, provider, delta_transform):
        ''':param provider: subscriber or anything, mainly used as an identifier
           :param geometry_msgs.msg.Transform delta_transform: pose update'''
        if provider not in self.pose_history:
            self.pose_history[provider] = [Pose()]  # TODO: orientation 0,0,0,1?
            self.enabled_history[provider] = [False]
            self.color[provider] = color_palette.next()
        # transform last pose to get the current one
        pose = copy_pose(self.pose_history[provider][-1])
        transform_pose(pose, delta_transform)
        # store
        self.pose_history[provider].append(pose)
        # use enabled=True for anything else than subscribers to not draw them gray
        enabled = provider.is_enabled() if isinstance(provider, AbstractLocalizationSubscriber) else True
        self.enabled_history[provider].append(enabled)

    def get_plot_color(self, provider, enabled):
        ''':returns str: a provider's color'''
        # TODO: figure out a way to display active/enabled...
        return self.color[provider] if enabled else 'gray'

    def get_label(self, provider):
        return provider.alias if hasattr(provider, 'alias') and provider.alias is not None else type(provider).__name__

    def __plot_positions__(self, provider, start_idx=0, end_idx=None, enabled=True):
        '''helper function: dispatches a plot call for a path segment'''
        if end_idx is None:
            end_idx = len(self.pose_history[provider]) - 1

        xs = [self.pose_history[provider][i].position.x for i in xrange(start_idx, end_idx + 1)]
        ys = [self.pose_history[provider][i].position.y for i in xrange(start_idx, end_idx + 1)]
        plt.plot(
            xs, ys,
            color=self.get_plot_color(provider, enabled),
            linewidth=2
        )

    def plotting(self):
        '''keeps plotting a planar projection of all gathered poses'''
        plt.axes().set_aspect('equal', 'datalim')
        while True:
            # clear
            plt.cla()

            legend_colors = ['gray']
            legend_labels = ['disabled']
            for provider in self.pose_history.keys():
                # there is no point in drawing less then two points
                if len(self.pose_history[provider]) < 2:
                    continue

                # segment path based on being enabled
                start_idx = 0
                prev_enabled = None
                for pose_idx, enabled in enumerate(self.enabled_history[provider]):
                    if (
                        (prev_enabled is not None and prev_enabled != enabled) or
                        pose_idx == len(self.pose_history[provider]) - 1
                    ):
                        self.__plot_positions__(provider, start_idx, pose_idx, prev_enabled)
                        start_idx = pose_idx
                    prev_enabled = enabled

                legend_colors.append(self.get_plot_color(provider, True))
                legend_labels.append(self.get_label(provider))

            # custom legend
            legend_lines = [Line2D([0], [0], color=c, lw=3) for c in legend_colors]
            plt.legend(legend_lines, legend_labels)
            # draw and sleep
            plt.pause(1)  # TODO: catch closed window (TclError)


# there shall be only one
__plotter__ = Plotter()
__started_plotting__ = False
__plot_start_mutex__ = threading.Lock()


def start():
    '''launch plotting thread, but no more than once'''
    global __started_plotting__
    with __plot_start_mutex__:
        if __started_plotting__:
            return
        __started_plotting__ = True
    # detach plotting from main thread
    plot_thread = threading.Thread(target=__plotter__.plotting)
    plot_thread.daemon = True
    plot_thread.start()


def add_pose(provider, delta_transform):
    ''':param provider: subscriber, controller or anything hashable
       :param geometry_msgs.msg.Transform delta_transform: update'''
    __plotter__.add_pose(provider, delta_transform)
