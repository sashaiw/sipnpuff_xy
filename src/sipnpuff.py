#!/usr/bin/env python

import rospy
import time
from math import pi
from std_msgs.msg import Float64


class SipNPuffUI:
    def __init__(self, zero=377, deadzone=3, timeout=.5, rate=1):
        self.zero = zero
        self.deadzone = deadzone
        self.timeout = timeout
        self.rate=rate
        self.mode = 'rest'
        self.servo_x = 0
        self.servo_y = 0
        self.start_time = time.time()
        self.end_time = time.time()

    @staticmethod
    def clamp(minimum, x, maximum):
        return max(minimum, min(x, maximum))

    def process(self, input):
        if self.zero - self.deadzone < input < self.zero + self.deadzone:
            output = 0
        else:
            output = input - self.zero
        return output

    def spin(self, sensor_val):
        if self.mode == 'rest' and self.process(sensor_val) == 0:
            self.start_time = time.time()

        # detect sel
        if self.mode == 'rest' and self.process(sensor_val) > 1000 - self.zero:
            self.end_time = time.time()
            if self.end_time - self.start_time > self.timeout:
                print('select')
                self.mode = 'sel'

        if self.mode == 'sel' and self.process(sensor_val) > 1000 - self.zero:
            self.start_time = time.time()
        elif self.mode == 'sel':
            self.end_time = time.time()
            if self.end_time - self.start_time > self.timeout:
                self.mode = 'rest'
                print('rest mode')

        # detect sip
        if self.mode == 'rest' and self.process(sensor_val) < -10:
            self.end_time = time.time()
            if self.end_time - self.start_time > self.timeout:
                print('x adj mode')
                self.mode = 'sip'

        # activate sip mode
        if self.mode == 'sip' and self.process(sensor_val) != 0:
            self.start_time = time.time()
            self.servo_x += -self.process(sensor_val) / 100000 * self.rate
            self.servo_x = self.clamp(-pi, self.servo_x, pi)
        elif self.mode == 'sip':
            self.end_time = time.time()
            if self.end_time - self.start_time > self.timeout:
                self.mode = 'rest'
                print('rest mode')

        # detect puff
        if self.mode == 'rest' and 10 < self.process(sensor_val) < 1000:
            self.end_time = time.time()
            if self.end_time - self.start_time > self.timeout:
                print('y adj mode')
                self.mode = 'puff'

        # activate puff mode
        if self.mode == 'puff' and self.process(sensor_val) != 0:
            self.start_time = time.time()
            self.servo_y += self.process(sensor_val) / 100000 * self.rate
            self.servo_y = self.clamp(-1, self.servo_y, 1)
        elif self.mode == 'puff':
            self.end_time = time.time()
            if self.end_time - self.start_time > self.timeout:
                self.mode = 'rest'
                print('rest mode')

        # print(self.process(sensor_val))
        return self.servo_x, self.servo_y


class SipNPuffNode:
    sensor_val = 0

    def __init__(self):
        sipnpuffui = SipNPuffUI()
        rospy.init_node('sipnpuff_ui')
        rate = float(rospy.get_param('~rate', '100'))
        servo_x_topic = rospy.get_param('~x_topic', '/pan_controller/command')
        servo_y_topic = rospy.get_param('~y_topic', '/tilt_controller/command')

        servo_x_pub = rospy.Publisher(servo_x_topic, Float64, queue_size=10)
        servo_y_pub = rospy.Publisher(servo_y_topic, Float64, queue_size=10)

        servo_x_msg = Float64()
        servo_y_msg = Float64()

        sensor_topic = rospy.get_param('~sensor_topic', 'sipnpuff_sensor')
        rospy.Subscriber(sensor_topic, Float64, self.callback)

        while not rospy.is_shutdown():
            servo_x_msg.data, servo_y_msg.data = sipnpuffui.spin(self.sensor_val)
            servo_x_pub.publish(servo_x_msg)
            servo_y_pub.publish(servo_y_msg)
            if rate:
                rospy.sleep(1/rate)
            else:
                rospy.sleep(1.0)

    def callback(self, data):
        self.sensor_val = data.data


if __name__ == '__main__':
    try:
        sipnpuff = SipNPuffNode()
    except rospy.ROSInterruptException:
        pass
