#!/usr/bin/env python3
import sys
import threading
import time
from collections import deque

import matplotlib.pyplot as plt
import numpy as np
import rospy
from matplotlib.animation import FuncAnimation
from sensor_msgs.msg import Imu

TOPICS = {
    'imu': '/livox/imu',
    'clearance': '/realtime_plot/dist_foot2gnd',
    'contact': '/realtime_plot/contact',
    'deviation': '/realtime_plot/foot_d',
    'area': '/realtime_plot/area',
    'slope': '/realtime_plot/tmp_distK'
}

NORMAL_LAYOUT = ('imu', 'clearance', 'deviation', 'contact')
DEBUG_LAYOUT = ('imu', 'clearance', 'contact', 'area', 'deviation', 'slope')

PLOT_CONFIG = {
    'imu': {
        'title': 'Inertial Data',
        'ylabel': 'Acc (g)',
        'ylim': (-1, 1),
        'margin': 0.1,
        'maxlen': 10000,
        'series': {
            'x': ('X', {'color': '#D95319'}),
            'y': ('Y', {'color': '#F0AD25'}),
            'z': ('Z', {'color': '#0072BC'})
        }
    },
    'clearance': {
        'title': 'Foot Clearance',
        'ylabel': 'Dist (m)',
        'ylim': (-1, 1),
        'margin': 0.05,
        'maxlen': 11000,
        'series': {
            'left': ('footL', {'color': '#0072BC'}),
            'right': ('footR', {'color': '#D95319'})
        }
    },
    'contact': {
        'title': 'Contact Events',
        'ylabel': 'Value',
        'ylim': (-1, 1),
        'margin': 0.1,
        'maxlen': 11000,
        'series': {
            'left': ('contactL', {'color': '#0072BC'}),
            'right': ('contactR', {'color': '#D95319'})
        }
    },
    'area': {
        'title': 'Foot Lift Area',
        'ylabel': 'Area (m²)',
        'ylim': (-1, 1),
        'margin': 0.05,
        'maxlen': 11000,
        'series': {
            'left': ('Left Foot Area', {'color': '#0072BC'}),
            'right': ('Right Foot Area', {'color': '#D95319'})
        }
    },
    'deviation': {
        'title': 'Foot Deviation',
        'ylabel': 'Deviation',
        'margin': 0.05,
        'maxlen': 11000,
        'series': {
            'left': ('Left Foot', {'color': '#0072BC'}),
            'right': ('Right Foot', {'color': '#D95319'})
        }
    },
    'slope': {
        'title': 'Foot Slope',
        'ylabel': 'Slope',
        'margin': 0.05,
        'maxlen': 11000,
        'series': {
            'left': ('Left Slope', {
                'color': '#0072BC', 'marker': 'o',
                'linestyle': 'None', 'markersize': 4
            }),
            'right': ('Right Slope', {
                'color': '#D95319', 'marker': 'o',
                'linestyle': 'None', 'markersize': 4
            })
        }
    }
}


class RealTimePlotter:
    def __init__(self):
        self.lock = threading.Lock()
        self.debug_output_en = rospy.get_param('/common/debug_output_en', False)
        self.layout = DEBUG_LAYOUT if self.debug_output_en else NORMAL_LAYOUT

        self.data = self._create_data_buffers()
        self.area_events = deque(maxlen=100)
        self.time_reference = None
        self.last_update_time = None
        self.time_window = 10
        self.gray_rects = []
        self.axes = {}
        self.lines = {}
        self.artists = ()
        self.animation = None

        self._subscribe()

    @staticmethod
    def _create_data_buffers():
        data = {}
        for panel, config in PLOT_CONFIG.items():
            data[panel] = {}
            for series in config['series']:
                data[panel][series] = {
                    'time': deque(maxlen=config['maxlen']),
                    'value': deque(maxlen=config['maxlen'])
                }
        return data

    def _subscribe(self):
        callbacks = {
            'imu': self.imu_callback,
            'clearance': self.clearance_callback,
            'contact': self.contact_callback,
            'deviation': self.deviation_callback,
            'area': self.area_callback,
            'slope': self.slope_callback
        }
        self.subscribers = [
            rospy.Subscriber(TOPICS[panel], Imu, callbacks[panel])
            for panel in self.layout
        ]

    def get_timestamp(self, msg):
        stamp = msg.header.stamp.to_sec()
        with self.lock:
            if self.time_reference is None:
                self.time_reference = stamp
                rospy.loginfo(
                    f"Time reference set to: {self.time_reference}"
                )
            reference = self.time_reference
        return stamp - reference

    def _append(self, panel, timestamp, values):
        with self.lock:
            for series, value in values.items():
                self.data[panel][series]['time'].append(timestamp)
                self.data[panel][series]['value'].append(value)

    def imu_callback(self, msg):
        try:
            timestamp = self.get_timestamp(msg)
            self._append('imu', timestamp, {
                'x': msg.linear_acceleration.x,
                'y': msg.linear_acceleration.y,
                'z': msg.linear_acceleration.z
            })
            self.last_update_time = time.time()
        except Exception as error:
            rospy.logerr(f"IMU callback error: {error}")

    def clearance_callback(self, msg):
        try:
            self._append('clearance', self.get_timestamp(msg), {
                'left': msg.linear_acceleration.x,
                'right': msg.linear_acceleration.y
            })
        except Exception as error:
            rospy.logerr(f"Foot callback error: {error}")

    def contact_callback(self, msg):
        try:
            self._append('contact', self.get_timestamp(msg), {
                'left': msg.linear_acceleration.x,
                'right': msg.linear_acceleration.y
            })
        except Exception as error:
            rospy.logerr(f"Contact callback error: {error}")

    def _append_leg_data(self, panel, msg):
        leg = int(msg.linear_acceleration.y)
        if leg not in (0, 1):
            return
        series = 'left' if leg == 0 else 'right'
        self._append(panel, self.get_timestamp(msg), {
            series: msg.linear_acceleration.x
        })

    def deviation_callback(self, msg):
        try:
            self._append_leg_data('deviation', msg)
        except Exception as error:
            rospy.logerr(f"Foot deviation callback error: {error}")

    def slope_callback(self, msg):
        try:
            self._append_leg_data('slope', msg)
        except Exception as error:
            rospy.logerr(f"Slope callback error: {error}")

    def area_callback(self, msg):
        try:
            timestamp = self.get_timestamp(msg)
            self._append('area', timestamp, {
                'left': msg.linear_acceleration.x,
                'right': msg.linear_acceleration.y
            })
            with self.lock:
                self.area_events.append(
                    (timestamp, msg.linear_acceleration.z)
                )
        except Exception as error:
            rospy.logerr(f"Area callback error: {error}")

    def wait_for_first_message(self):
        rospy.loginfo(
            "Waiting for first message to set time reference..."
        )
        while self.time_reference is None and not rospy.is_shutdown():
            rospy.sleep(0.1)
        if rospy.is_shutdown():
            sys.exit(0)
        rospy.loginfo(
            "Time reference established. Creating plot..."
        )

    def create_plot(self):
        rows = 3 if self.debug_output_en else 2
        figsize_ = (9, 6) if self.debug_output_en else (8, 4.7)
        self.fig, axes = plt.subplots(
            rows, 2, figsize=figsize_, sharex=True
        )
        self.axes = dict(zip(self.layout, axes.flat))

        bottom_panels = self.layout[-2:]
        for panel, axis in self.axes.items():
            config = PLOT_CONFIG[panel]
            axis.set_title(config['title'], fontsize='10')
            axis.set_ylabel(config['ylabel'], fontsize='10')
            if panel in bottom_panels:
                axis.set_xlabel('Time (s)', fontsize='10')
            if 'ylim' in config:
                axis.set_ylim(*config['ylim'])

            self.lines[panel] = {}
            for series, (label, style) in config['series'].items():
                line, = axis.plot([], [], label=label, **style)
                self.lines[panel][series] = line

            axis.legend(loc='upper left')
            axis.grid(True, linestyle='--', alpha=0.5)

        if self.debug_output_en:
            self.fig.subplots_adjust(
                top=0.95, bottom=0.08, left=0.1, right=0.95,
                hspace=0.2, wspace=0.25
            )
        else:
            self.fig.subplots_adjust(
                top=0.95, bottom=0.1, left=0.1, right=0.95,
                hspace=0.25, wspace=0.25
            )

        self.artists = tuple(
            line
            for panel in self.layout
            for line in self.lines[panel].values()
        )

    def _snapshot(self):
        with self.lock:
            data = {
                panel: {
                    series: {
                        'time': list(buffer['time']),
                        'value': list(buffer['value'])
                    }
                    for series, buffer in self.data[panel].items()
                }
                for panel in self.layout
            }
            events = list(self.area_events) if self.debug_output_en else []
        return data, events

    @staticmethod
    def _current_time(data):
        latest_times = []
        for panel_name, panel in data.items():
            for buffer in panel.values():
                if not buffer['time']:
                    continue
                if panel_name in ('deviation', 'slope'):
                    latest_times.append(max(buffer['time']))
                else:
                    latest_times.append(buffer['time'][-1])
        return max(latest_times) if latest_times else 0

    def _window_indices(self, times, current_time):
        return [
            index for index, timestamp in enumerate(times)
            if current_time - self.time_window <= timestamp <= current_time
        ]

    def _update_panel(self, panel, data, current_time):
        selected = {
            series: self._window_indices(
                buffer['time'], current_time
            )
            for series, buffer in data.items()
        }
        if not any(selected.values()):
            return

        center = current_time - self.time_window / 2
        all_values = []
        for series, indices in selected.items():
            line = self.lines[panel][series]
            if not indices:
                line.set_data([], [])
                continue

            buffer = data[series]
            times = [buffer['time'][index] - center for index in indices]
            values = [buffer['value'][index] for index in indices]
            line.set_data(times, values)
            all_values.extend(values)

        if all_values:
            minimum = min(all_values)
            maximum = max(all_values)
            margin = max(
                PLOT_CONFIG[panel]['margin'],
                (maximum - minimum) * 0.2
            )
            self.axes[panel].set_ylim(
                minimum - margin, maximum + margin
            )

    def _update_gray_regions(self, events, current_time):
        for rectangle in self.gray_rects:
            try:
                rectangle.remove()
            except Exception:
                pass
        self.gray_rects.clear()

        window_start = current_time - self.time_window
        center = current_time - self.time_window / 2
        for event_time, offset in events:
            left_time = event_time - offset
            right_time = event_time
            if not (
                window_start <= left_time <= current_time
                or window_start <= right_time <= current_time
            ):
                continue

            for axis in self.axes.values():
                rectangle = axis.axvspan(
                    left_time - center,
                    right_time - center,
                    color='gray',
                    alpha=0.25,
                    zorder=0
                )
                self.gray_rects.append(rectangle)

    def _update_time_axis(self, current_time):
        half_window = self.time_window / 2
        self.axes[self.layout[0]].set_xlim(-half_window, half_window)

        ticks = np.linspace(-half_window, half_window, 5)
        labels = [
            f"{current_time - half_window + tick:.2f}"
            for tick in ticks
        ]
        for panel in self.layout[-2:]:
            self.axes[panel].set_xticks(ticks)
            self.axes[panel].set_xticklabels(labels)

    def update(self, frame):
        if (
            self.last_update_time is not None
            and time.time() - self.last_update_time > 1.0
        ):
            return self.artists

        data, events = self._snapshot()
        current_time = self._current_time(data)
        if current_time == 0:
            return self.artists

        for panel in self.layout:
            self._update_panel(panel, data[panel], current_time)
        if self.debug_output_en:
            self._update_gray_regions(events, current_time)

        self._update_time_axis(current_time)
        self.fig.canvas.draw_idle()
        return self.artists

    def run(self):
        mode = 'DEBUG' if self.debug_output_en else 'DEFAULT'
        rospy.loginfo(f"Real-time plot mode: [{mode}]")
        self.wait_for_first_message()
        self.create_plot()
        self.animation = FuncAnimation(
            self.fig,
            self.update,
            interval=100,
            blit=False,
            save_count=1000
        )
        self.fig.canvas.manager.window.setWindowTitle(
            "Data Visualization"
        )
        plt.show()
        rospy.spin()


def main():
    rospy.init_node('plotter')
    RealTimePlotter().run()


if __name__ == '__main__':
    main()
