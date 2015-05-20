#! /usr/bin/python

import rospy
import moveit_commander
from geometry_msgs.msg import Twist
import tf
from numpy import linalg as LA
from math import *
from collections import namedtuple
from geometry_msgs.msg import PoseStamped
import time

# assuming there is already a ros node, do not init one here

# P control is sufficient for this function

twistBound = namedtuple('twistBound', ['lower', 'upper'])

class baseMove:
    def __init__(self, verbose=False):
        self.base_pub = rospy.Publisher('/base_controller/command', Twist)
        # self.listener = tf.TransformListener()
        self.verbose = verbose
        self.posTolerance = 4
        self.angTolerance = 1
        self.linearGain = 10
        self.angularGain = 10
        self.comm = rospy.Rate(20)
        self.linearTwistBound = twistBound(0.04, 0.1)
        self.angularTwistBound = twistBound(0.06, 0.3)
        self.refFrame = '/shelf_frame'
        rospy.Subscriber("/pubShelfSep", PoseStamped, self.updateShelfPose)
        self.trans = (0,0,0)
        self.pose = (0,0,0,0)
        self.move = False
        self.walltime = rospy.Time.now()

        self.delay = 0.0

        self.source = 1 # 0 for tf tree and 1 for pubShelfSep

    def setPosTolerance(self, t):
        self.posTolerance = t

    def setAngTolerance(self, t):
        self.angTolerance = t

    def setLinearGain(self, g):
        self.linearGain = g

    def setAngularGain(self, g):
        self.angularGain = g



    def updateShelfPose(self, msg):
        self.trans = (-msg.pose.position.x - 0.275, -msg.pose.position.y, -msg.pose.position.z - 0.252) # from base_laser_link to base_link
        self.pose = (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, -msg.pose.orientation.w)
        self.walltime = msg.header.stamp
        self.move = True


    def update_delay(self):
        measurements = 0
        self.delay = 0.0
        r = rospy.Rate(20.0)
        while not rospy.is_shutdown() and measurements<10:
            self.delay += (rospy.Time.now() - self.walltime).to_sec()

            measurements += 1
            r.sleep()

        self.delay /= 10

        rospy.loginfo('[base_move]: new delay ' + str(self.delay) + ' sec')

    def goPosition(self, position, wait=True):
        if wait:
            self.update_delay()
        s = Twist()
        while not rospy.is_shutdown():
            if wait:
                self.comm.sleep()

            if self.source == 0:
                try:
                    (trans,rot) = self.listener.lookupTransform(self.refFrame, "/base_link", rospy.Time(0))

                except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    if not wait:
                        return False

                    else:
                        continue

            elif self.source == 1:

                if (rospy.Time.now() - self.walltime).to_sec() > 0.4 + self.delay:
                    rospy.logerr('[baseMove]: old shelf pose message, time diff ' + str((rospy.Time.now()-self.walltime).to_sec()))
                    self.move = False
                # self.move=True

                if self.move:
                    trans = self.trans
                    rot = self.pose
                else:
                    if self.verbose:
                        rospy.logwarn('[baseMove]: pose msg not ready')
                    continue

            else:
                if self.verbose:
                    rospy.logwarn('[baseMove]: pose source not known')
                pass


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




    def goAngle(self, angle, wait=True):
        if wait:
            self.update_delay()
        s = Twist()
        while not rospy.is_shutdown():
            if wait:
                self.comm.sleep()

            if self.source == 0:
                try:
                    (trans,rot) = self.listener.lookupTransform(self.refFrame, "/base_link", rospy.Time(0))

                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    if not wait:
                        return False
                    else:
                        continue

            elif self.source == 1:

                if (rospy.Time.now() - self.walltime).to_sec() > 0.4 + self.delay:
                    rospy.logerr('[baseMove]: old shelf pose message, time diff ' + str((rospy.Time.now()-self.walltime).to_sec()))
                    self.move = False
                # self.move=True

                if self.move:
                    trans = self.trans
                    rot = self.pose
                else:
                    if self.verbose:
                        rospy.logwarn('[baseMove]: pose msg not ready')
                    continue

            else:
                if self.verbose:
                    rospy.logwarn('[baseMove]: pose source not known')

            theta = tf.transformations.euler_from_quaternion(rot)[2]


            if self.verbose:
                print 'theta: %4f, angle: %4f' % (theta, angle)
            z_diff = (angle - theta)
            s.angular.z = z_diff * self.angularGain

            # print s.angular.z

            if abs(s.angular.z) >= self.angularTwistBound.upper:
                s.angular.z = self.angularTwistBound.upper * (s.angular.z)/abs(s.angular.z)
            elif abs(s.angular.z) < self.angularTwistBound.lower:
                s.angular.z = self.angularTwistBound.lower * (s.angular.z)/abs(s.angular.z)

            # print s.angular.z

            self.base_pub.publish(s)
            if abs(z_diff) < self.angTolerance:
                if self.verbose:
                    print 'angle arrived'
                return True

            if not wait:
                return False



    '''
    The go() method is not stable in practice use to constant rotation of wheels, DO NOT use it.
    '''
    def go(self, position, angle, wait=True):

        rospy.logerr('no longer supported!!!!!!!!!!!!')
        rospy.shutdown()
        
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