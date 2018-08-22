#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from math import pi


class Visualizer:
    x = 0
    y = 0

    def __init__(self):
        self.listener()
        self.fig = plt.figure()

        self.ax1 = self.fig.add_subplot(1, 1, 1)
        self.dot, = self.ax1.plot([self.x], [self.y], '.')
        self.ani = animation.FuncAnimation(self.fig, self.animate, interval=10)
        plt.xlim(-pi, pi)
        plt.ylim(-pi, pi)
        plt.show()

    def animate(self, i):
        self.dot.set_xdata([-self.x])
        self.dot.set_ydata([self.y])

    def callback_x(self, x):
        self.x = x.data

    def callback_y(self, y):
        self.y = y.data

    def listener(self):
        rospy.init_node('laser_visualizer')
        x_topic = rospy.get_param('~x_topic', '/pan_controller/command')
        y_topic = rospy.get_param('~y_topic', '/tilt_controller/command')
        rospy.Subscriber(x_topic, Float64, self.callback_x)
        rospy.Subscriber(y_topic, Float64, self.callback_y)


if __name__ == '__main__':
    Visualizer()
