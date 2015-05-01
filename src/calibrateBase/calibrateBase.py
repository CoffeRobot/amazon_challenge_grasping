#! /usr/bin/python

import rospy
import math


from baseMove import *
from baseScan import *
import matplotlib.pyplot as plt


rospy.init_node('calibrate_base_node', anonymous=True)

position = [-0.82, 0]
angle = 0

bm = baseMove(verbose=True)

bm.setPosTolerance(0.01)
bm.setAngTolerance(0.006)
bm.setLinearGain(0.4)
bm.setAngularGain(1)




bm.goAngle(angle)
bm.goPosition(position)
bm.goAngle(angle)