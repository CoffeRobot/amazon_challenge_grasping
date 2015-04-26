#! /usr/bin/python

from baseScan import *


rospy.init_node('shelf_publisher', anonymous=True)
rospy.sleep(15.0)
bs = baseScan()
bs.publish2TF()