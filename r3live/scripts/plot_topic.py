#!/usr/bin/env python
# coding=utf-8

from cmath import sqrt
import numpy as np
import matplotlib.pyplot as plt
import argparse
import os
from datetime import datetime

import rospy
import rosbag
from genpy.rostime import Time
from sensor_msgs.msg import Imu


class plotTopic:
    def __init__(self, topic_name):
        rospy.init_node('reader', anonymous=True)
        self.name = topic_name
        self.norms = []
        rospy.Subscriber(self.name, Imu, self.print_grav)

    def print_grav(self, msg):
        lin_x = msg.linear_acceleration.x
        lin_y = msg.linear_acceleration.y
        lin_z = msg.linear_acceleration.z
        
        norm = sqrt(lin_x*lin_x + lin_y*lin_y + lin_z*lin_z)
        self.norms.append(norm.real)
        avg = np.average(self.norms)
        print("acc norm = {}".format(avg))



if __name__ == '__main__':
    # parser = argparse.ArgumentParser(description='Read imu topic and plot')
    # parser.add_argument('-t', '--topic', type=str, help='Imu rostopic name to read', required=True)
    # parser.add_argument('-r', '--result', type=str, help='Result Bag file and plot save path', default='./result')
    # args = parser.parse_args()

    # topic = plotTopic(args.topic)
    # topic.logging(args.result)

    topic = plotTopic("/vectornav/IMU")
    rospy.spin()