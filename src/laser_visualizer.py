#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
import matplotlib
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
        self.ani = animation.FuncAnimation(self.fig, self.animate, interval=10)
        self.dot, = self.ax1.plot([self.x.data], [self.y.data], '.')
        plt.xlim(-pi, pi)
        plt.ylim(-pi, pi)
        plt.show()

    def animate(self, i):
        self.dot.set_xdata([-self.x.data])
        self.dot.set_ydata([self.y.data])

    def callback_x(self, x):
        self.x = x

    def callback_y(self, y):
        self.y = y

    def listener(self):
        rospy.init_node('laser_visualizer')
        x_topic = rospy.get_param('~x_topic', '/pan_controller/command')
        y_topic = rospy.get_param('~y_topic', '/tilt_controller/command')
        rospy.Subscriber(x_topic, Float64, self.callback_x)
        rospy.Subscriber(y_topic, Float64, self.callback_y)

if __name__ == '__main__':
    Visualizer()
