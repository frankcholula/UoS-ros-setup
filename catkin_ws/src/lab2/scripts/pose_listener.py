#!/usr/bin/env python
import rospy
from turtlesim.msg import Pose


def callback(data):
    print(data)

if __name__ == '__main__':
    rospy.init_node('pose_listener_python')

    rospy.Subscriber('/turtle1/pose', Pose, callback, queue_size=1)

    rospy.spin()
