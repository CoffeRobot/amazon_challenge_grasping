#! /usr/bin/python

from baseScan import *
import os


rospy.init_node('shelf_publisher', anonymous=True)
# rospy.sleep(15.0)
os.nice(1000)
bs = baseScan()
bs.publish2TF()
