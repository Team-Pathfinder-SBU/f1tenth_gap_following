import time
from collections import deque

import numpy as np
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanPlotter:
    def __init__(self, topic="/scan"):
        rclpy.init()
        self.node = rclpy.create_node("plot_ranges_plotter")
        self.latest = None
        self.sub = self.node.create_subscription(LaserScan, topic, self._callback, 10)

        plt.ion()
        self.fig, self.ax = plt.subplots()
        (self.line,) = self.ax.plot([], [], lw=1)
        self.ax.set_xlabel("beam index")
        self.ax.set_ylabel("range (m)")
        self.ax.set_title("Live LiDAR ranges")
        self.ax.grid(True)

    def _callback(self, msg: LaserScan):
        # store as numpy array for plotting in the main loop
        self.latest = np.array(msg.ranges)

    def run(self):
        try:
            while rclpy.ok():
                # process incoming messages
                rclpy.spin_once(self.node, timeout_sec=0.01)

                if self.latest is not None:
                    y = self.latest
                    x = np.arange(len(y))

                    # update plot data
                    self.line.set_data(x, y)
                    self.ax.set_xlim(0, len(y))

                    # adjust y-limits with a small margin
                    ymax = (
                        np.nanmax(y[np.isfinite(y)]) if np.any(np.isfinite(y)) else 1.0
                    )
                    self.ax.set_ylim(0, max(1.0, ymax + 0.2))

                    self.fig.canvas.draw()
                    self.fig.canvas.flush_events()

                # small sleep to keep UI responsive
                time.sleep(0.02)
        except KeyboardInterrupt:
            pass
        finally:
            try:
                self.node.destroy_node()
            except Exception:
                pass
            rclpy.shutdown()


if __name__ == "__main__":
    ScanPlotter().run()
