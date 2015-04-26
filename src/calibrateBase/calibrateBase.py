#! /usr/bin/python

import rospy
import math


from baseMove import *
from baseScan import *
import matplotlib.pyplot as plt


rospy.init_node('calibrate_base_node', anonymous=True)

position = [-0.7579, 0]
angle = 0

bm = baseMove(verbose=True)

bm.setPosTolerance(0.02)
bm.setAngTolerance(0.006)
bm.setLinearGain(1)
bm.setAngularGain(1)




bm.goAngle(angle)
bm.goPosition(position)
bm.goAngle(angle)