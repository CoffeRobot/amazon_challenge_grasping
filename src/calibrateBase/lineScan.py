#! /usr/bin/python

import rospy
import tf
from numpy import linalg as LA
from math import *
from sensor_msgs.msg import LaserScan, JointState
from laser_geometry import LaserProjection
import sensor_msgs.point_cloud2 as pc2
import math
from termcolor import colored
from lineScan import *
import numpy as np
from line_extraction import *

# assuming there is already a ros node, do not init one here

# P control is sufficient for this function


class lineScan:
	def __init__(self, verbose=False):
		self.rangeData = LaserScan()
		self.scan_sub = rospy.Subscriber("/base_scan", LaserScan, self.callback)
		self.listener = tf.TransformListener()
		self.laser_projector = LaserProjection()
		self.pc = []
		self.leg1 = []
		self.leg2 = []
		self.br = tf.TransformBroadcaster()
		self.rate = rospy.Rate(100.0)
		self.calibrated = False
		self.reCalibration = False
		self.priorOri = []
		self.priorLeft = []
		self.priorRight = []
		self.odomL = []
		self.odomR = []
		self.priorAvailable = False
		self.newObsWeight = 0.1
		self.offsetXY = [-0.044, 0]


	def callback(self,data):
		self.rangeData = data

	def refreshRangeData(self):
		self.rangeData = LaserScan() # flush
		while len(self.rangeData.ranges) == 0:
			rospy.sleep(0.04)

	def getStatus(self):
		return self.calibrated

	def getCloud(self):

		self.refreshRangeData()
		cloud2 = self.laser_projector.projectLaser(self.rangeData)

		xyz = pc2.read_points(cloud2, skip_nans=True, field_names=("x", "y", "z"))
		self.pc = []
		while True:
			try:
				self.pc.append(xyz.next())
			except:
				break
		return self.pc

	def segmentLength(self, m, i1, i2):
		v1 = m[i1]
		v2 = m[i2]
		return np.linalg.norm(v1-v2)


	def cloud2Input(self):


		while not rospy.is_shutdown():
			plt.clf()
			pc = self.getCloud()
			m = np.asarray(pc)

			segs = []
			r = split_and_merge(m, 0.1)
			for i in range(1, len(r)):
				if r[i] - r[i-1] < 20:
					continue
				l = self.segmentLength(m, r[i], r[i-1])
				if l > 0.8 and l < 0.9:
					print 'found'
					print l
					segs.append([r[i], r[i-1]])

			# Plot found lines
			plt.plot(m[:, 0], m[:, 1], '.r')
			plt.plot(m[r, 0], m[r, 1], '-')
			plt.plot(m[r, 0], m[r, 1], '+')
			for s in segs:
				plt.plot(m[s, 0], m[s, 1], 'g-+', linewidth=5)
			plt.axis('equal')
			plt.draw()
			plt.title("Laser scan")
			plt.show(block=False)

