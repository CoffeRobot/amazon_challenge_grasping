#! /usr/bin/python

import rospy
import moveit_commander
from geometry_msgs.msg import Twist
import tf
from numpy import linalg as LA
from math import *
from collections import namedtuple


# assuming there is already a ros node, do not init one here

# P control is sufficient for this function

twistBound = namedtuple('twistBound', ['lower', 'upper'])

class baseMove:
	def __init__(self, verbose=False):
		self.base_pub = rospy.Publisher('/base_controller/command', Twist)
		self.listener = tf.TransformListener()
		self.verbose = verbose
		self.posTolerance = 4
		self.angTolerance = 1
		self.linearGain = 10
		self.angularGain = 10
		self.comm = rospy.Rate(100)
		self.linearTwistBound = twistBound(0.06, 0.14)
		self.angularTwistBound = twistBound(0.06, 0.2)
		self.refFrame = '/shelf_frame'

	def setPosTolerance(self, t):
		self.posTolerance = t

	def setAngTolerance(self, t):
		self.angTolerance = t

	def setLinearGain(self, g):
		self.linearGain = g

	def setAngularGain(self, g):
		self.angularGain = g

	def goPosition(self, position, wait=True):
		s = Twist()
		while True:
			try:
				(trans,rot) = self.listener.lookupTransform(self.refFrame, "/base_link", rospy.Time(0))
				theta = tf.transformations.euler_from_quaternion(rot)[2]
				x_diff = (position[0] - trans[0])
				y_diff = (position[1] - trans[1])
				alpha = atan2(y_diff, x_diff)
				r = alpha - theta
				if self.verbose:
					print 'X: %4f, Y: %4f' % (trans[0], trans[1])
				l = LA.norm([x_diff, y_diff])
				s.linear.x = l * cos(r) * self.linearGain
				s.linear.y = l * sin(r) * self.linearGain
				tmp = LA.norm([s.linear.x, s.linear.y])

				if tmp <= self.linearTwistBound.lower:
					s.linear.x = s.linear.x * (self.linearTwistBound.lower / tmp)
					s.linear.y = s.linear.y * (self.linearTwistBound.lower / tmp)

				if tmp > self.linearTwistBound.upper:
					s.linear.x = s.linear.x * (self.linearTwistBound.upper / tmp)
					s.linear.y = s.linear.y * (self.linearTwistBound.upper / tmp)

				self.base_pub.publish(s)
				if LA.norm([x_diff, y_diff]) < self.posTolerance:
					if self.verbose:
						print 'position arrived'
					return True

				if not wait:
					return False

			except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				if wait:
					self.comm.sleep()
				else:
					return False

	def goAngle(self, angle, wait=True):
		s = Twist()
		while True:
			try:
				(trans,rot) = self.listener.lookupTransform(self.refFrame, "/base_link", rospy.Time(0))
				theta = tf.transformations.euler_from_quaternion(rot)[2]


				if self.verbose:
					print 'theta: %4f, angle: %4f' % (theta, angle)
				z_diff = (angle - theta)
				s.angular.z = z_diff * self.angularGain

				print s.angular.z

				if abs(s.angular.z) >= self.angularTwistBound.upper:
					s.angular.z = self.angularTwistBound.upper * (s.angular.z)/abs(s.angular.z)
				elif abs(s.angular.z) < self.angularTwistBound.lower:
					s.angular.z = self.angularTwistBound.lower * (s.angular.z)/abs(s.angular.z)

				print s.angular.z

				self.base_pub.publish(s)
				if abs(z_diff) < self.angTolerance:
					if self.verbose:
						print 'angle arrived'
					return True

				if not wait:
					return False

			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				if wait:
					self.comm.sleep()

				else:
					return False

	'''
	The go() method is not stable in practice use to constant rotation of wheels, DO NOT use it.
	'''
	def go(self, position, angle, wait=True):
		s = Twist()
		while True:
			try:
				(trans,rot) = self.listener.lookupTransform(self.refFrame, "/base_link", rospy.Time(0))
				theta = tf.transformations.euler_from_quaternion(rot)[2]
				x_diff = (position[0] - trans[0])
				y_diff = (position[1] - trans[1])
				alpha = atan2(y_diff, x_diff)
				r = alpha - theta
				if self.verbose:
					print 'X: %4f, Y: %4f, angle: %4f' % (trans[0], trans[1], theta)
				l = LA.norm([x_diff, y_diff])
				s.linear.x = l * cos(r) * self.linearGain
				s.linear.y = l * sin(r) * self.linearGain
				tmp = LA.norm([s.linear.x, s.linear.y])

				if tmp < self.linearTwistBound.lower:
					s.linear.x = s.linear.x * (self.linearTwistBound.lower / tmp)
					s.linear.y = s.linear.y * (self.linearTwistBound.lower / tmp)

				if tmp > self.linearTwistBound.upper:
					s.linear.x = s.linear.x * (self.linearTwistBound.upper / tmp)
					s.linear.y = s.linear.y * (self.linearTwistBound.upper / tmp)

				z_diff = (angle - theta)
				s.angular.z = z_diff * self.angularGain

				if abs(s.angular.z) > self.angularTwistBound.upper:
					s.angular.z = self.angularTwistBound.upper * (s.angular.z)/abs(s.angular.z)
				elif abs(s.angular.z) < self.angularTwistBound.lower:
					s.angular.z = self.angularTwistBound.lower * (s.angular.z)/abs(s.angular.z)


				self.base_pub.publish(s)
				if LA.norm([x_diff, y_diff]) < self.posTolerance and abs(z_diff) < self.angTolerance:
					if self.verbose:
						print 'position and angle arrived'
					return True
				if not wait:
					return False

			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				if wait:
					self.comm.sleep()

				else:
					return False