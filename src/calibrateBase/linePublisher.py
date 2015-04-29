#! /usr/bin/python

import rospy
from lineScan import *

rospy.init_node('line_scan', anonymous=True)

ls = lineScan()
ls.cloud2Input()
