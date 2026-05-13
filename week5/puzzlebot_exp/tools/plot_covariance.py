#!/usr/bin/env python3
"""Subscribe to /robot1/odom, record covariance and plot results on exit.

Usage:
  # source ROS 2 and workspace
  . /opt/ros/<distro>/setup.bash
  . install/setup.bash
  python3 tools/plot_covariance.py

Requires: numpy, matplotlib
"""
import math
import signal
import sys
from collections import deque

import matplotlib.pyplot as plt
import numpy as np

import rclpy
from nav_msgs.msg import Odometry


class CovariancePlotter:
    def __init__(self, topic='/robot1/odom', max_history=1000, skip=10):
        rclpy.init()
        self.node = rclpy.create_node('covariance_plotter')
        self.sub = self.node.create_subscription(Odometry, topic, self.cb, 10)

        self.times = []
        self.xs = []
        self.ys = []
        self.thetas = []
        self.var_x = []
        self.var_y = []
        self.var_theta = []
        self.ellipses = []  # store (x,y,2x_axis,2y_axis,angle)

        self.max_history = max_history
        self.skip = skip
        self.counter = 0

        # handle ctrl-c
        signal.signal(signal.SIGINT, self._shutdown)

    def cb(self, msg: Odometry):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # extract 6x6 pose covariance (row-major) and map to 3x3 [x,y,theta]
        cov = list(msg.pose.covariance)
        if len(cov) < 36:
            return

        P00 = cov[0]
        P01 = cov[1]
        P05 = cov[5]

        P10 = cov[6]
        P11 = cov[7]
        P15 = cov[11]

        P50 = cov[30]
        P51 = cov[31]
        P55 = cov[35]

        # 2D position covariance
        pos_cov = np.array([[P00, P01], [P10, P11]])

        # eigen decomposition for ellipse
        try:
            vals, vecs = np.linalg.eig(pos_cov)
            # ensure positive
            vals = np.real(vals)
            vecs = np.real(vecs)
            order = np.argsort(vals)[::-1]
            vals = vals[order]
            vecs = vecs[:, order]
            major = math.sqrt(max(vals[0], 0.0))
            minor = math.sqrt(max(vals[1], 0.0))
            angle = math.atan2(vecs[1, 0], vecs[0, 0])
        except Exception:
            major = 0.0
            minor = 0.0
            angle = 0.0

        # store values
        self.times.append(stamp)
        self.xs.append(x)
        self.ys.append(y)
        # theta from quaternion
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        theta = math.atan2(2.0 * (qw * qz), 1.0 - 2.0 * (qz * qz)) if True else 0.0
        self.thetas.append(theta)

        self.var_x.append(P00)
        self.var_y.append(P11)
        self.var_theta.append(P55)

        self.counter += 1
        if (self.counter % self.skip) == 0:
            # store ellipse with 2-sigma axes for visibility
            self.ellipses.append((x, y, 2.0 * major, 2.0 * minor, angle))

        # keep history bounded
        if len(self.times) > self.max_history:
            self.times.pop(0)
            self.xs.pop(0)
            self.ys.pop(0)
            self.var_x.pop(0)
            self.var_y.pop(0)
            self.var_theta.pop(0)

    def run(self):
        try:
            self.node.get_logger().info('Covariance plotter started, listening...')
            rclpy.spin(self.node)
        except KeyboardInterrupt:
            pass
        finally:
            self._do_plots()
            self.node.destroy_node()
            rclpy.shutdown()

    def _shutdown(self, signum, frame):
        # stop spinning and plot
        try:
            rclpy.shutdown()
        except Exception:
            pass

    def _do_plots(self):
        if not self.times:
            print('No data received.')
            return

        t0 = self.times[0]
        ts = [t - t0 for t in self.times]

        # Plot variances
        fig, axs = plt.subplots(2, 1, figsize=(8, 10))
        axs[0].plot(ts, self.var_x, label='var(x)')
        axs[0].plot(ts, self.var_y, label='var(y)')
        axs[0].plot(ts, self.var_theta, label='var(theta)')
        axs[0].set_xlabel('time (s)')
        axs[0].set_ylabel('variance')
        axs[0].legend()
        axs[0].grid(True)

        # Plot trajectory and ellipses
        axs[1].plot(self.xs, self.ys, '-k', alpha=0.6, label='trajectory')
        for (x, y, a, b, ang) in self.ellipses:
            ell = plt.matplotlib.patches.Ellipse((x, y), width=a, height=b,
                                                 angle=math.degrees(ang),
                                                 edgecolor='purple', facecolor='none', alpha=0.6)
            axs[1].add_patch(ell)

        axs[1].set_aspect('equal', adjustable='datalim')
        axs[1].set_xlabel('x (m)')
        axs[1].set_ylabel('y (m)')
        axs[1].legend()
        axs[1].grid(True)

        fig.tight_layout()
        out = 'covariance_results.png'
        fig.savefig(out, dpi=150)
        print(f'Plots saved to {out}')


def main():
    plotter = CovariancePlotter()
    plotter.run()


if __name__ == '__main__':
    main()
