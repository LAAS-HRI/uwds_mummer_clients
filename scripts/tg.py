#!/usr/bin/python

import rospy
from naoqi import ALProxy

if __name__ == "__main__":
    rospy.init_node("tg")
    while not rospy.is_shutdown():
        a = ALProxy("ALFaceDetection", "mummer6-eth0.laas.fr", 9559)
        a.pause(1)
        rospy.sleep(5.0)
