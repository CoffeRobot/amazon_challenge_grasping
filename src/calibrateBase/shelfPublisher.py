#! /usr/bin/python

from baseScan import *


rospy.init_node('shelf_publisher', anonymous=True)

bs = baseScan()
bs.publish2TF()