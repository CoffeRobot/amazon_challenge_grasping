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
from geometry_msgs.msg import PoseStamped

# assuming there is already a ros node, do not init one here

# P control is sufficient for this function


class baseScan:
    def __init__(self, verbose=False):
        self.rangeData = LaserScan()
        self.scan_sub = rospy.Subscriber("/base_scan", LaserScan, self.callback)
        self.listener = tf.TransformListener()
        self.laser_projector = LaserProjection()
        self.pc = []
        self.leg1 = []
        self.leg2 = []
        self.br = tf.TransformBroadcaster()
        self.rate = rospy.Rate(60.0)
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
        self.binOffset = 0.02
        self.pubShelfSep = rospy.Publisher('pubShelfSep', PoseStamped)



    def callback(self,data):
        self.rangeData = data

    def refreshRangeData(self):
        self.rangeData = LaserScan() # flush
        while len(self.rangeData.ranges) == 0 and not rospy.is_shutdown():
            rospy.sleep(0.04)

    def getStatus(self):
        return self.calibrated

    def getCloud(self):

        self.refreshRangeData()
        cloud2 = self.laser_projector.projectLaser(self.rangeData)

        xyz = pc2.read_points(cloud2, skip_nans=True, field_names=("x", "y", "z"))
        self.pc = []
        while not rospy.is_shutdown():
            try:
                self.pc.append(xyz.next())
            except:
                break
        return self.pc

    def findLegsOnce(self):
        pc = self.getCloud()
        x = []
        y= []
        for i in range(len(pc)):
            x.append(pc[i][0])
            y.append(pc[i][1])
        radius = []

        if self.calibrated or self.reCalibration: # use prior to find legs
            for i in range(len(pc)):
                radius.append(math.sqrt((x[i]-self.priorLeft[0])**2 + (y[i] - self.priorLeft[1])**2))
            n1 = radius.index(min(radius))

            radius = []
            for i in range(len(pc)):
                radius.append(math.sqrt((x[i]-self.priorRight[0])**2 + (y[i] - self.priorRight[1])**2))
            n2 = radius.index(min(radius))


            leg1 = [x[n1], y[n1]]
            leg2 = [x[n2], y[n2]]
        else:
            # Assuming there is nothing between the robot and the shelf
            for i in range(len(pc)):
                radius.append(math.sqrt(x[i]**2 + y[i]**2))
            n = radius.index(min(radius))
            
            x2 = [x[i] for i in range(len(x)) if math.sqrt( (x[i]-x[n])**2 + (y[i]-y[n])**2 ) > 0.4]
            y2 = [y[i] for i in range(len(y)) if math.sqrt( (x[i]-x[n])**2 + (y[i]-y[n])**2 ) > 0.4]
            radius2 = []
            
            if self.calibrated:
                for i in range(len(x2)):
                    radius2.append(math.sqrt((x2[i] - self.priorOri[0])**2 + (y2[i] - self.priorOri[1])**2))
            else:
                for i in range(len(x2)):
                    radius2.append(math.sqrt(x2[i]**2 + y2[i]**2))
            n2 = radius2.index(min(radius2))

            leg1 = [x[n], y[n]]
            leg2 = [x2[n2], y2[n2]]
        leg1 = [leg1[0] + self.offsetXY[0], leg1[1] + self.offsetXY[1]]
        leg2 = [leg2[0] + self.offsetXY[0], leg2[1] + self.offsetXY[1]]
        return [leg1, leg2] # left, right

    def findLegs(self):
        '''
        accumulate new observations
        '''
        L1x = 0
        L1y = 0
        L2x = 0
        L2y = 0

        legs = self.findLegsOnce()

        if legs[0][1] < legs[1][1]:
            legs[0], legs[1] = legs[1], legs[0]

        if self.calibrated:
            self.leg1[0] = self.leg1[0] * (1-self.newObsWeight) + legs[0][0] * self.newObsWeight
            self.leg1[1] = self.leg1[1] * (1-self.newObsWeight) + legs[0][1] * self.newObsWeight
            self.leg2[0] = self.leg2[0] * (1-self.newObsWeight) + legs[1][0] * self.newObsWeight
            self.leg2[1] = self.leg2[1] * (1-self.newObsWeight) + legs[1][1] * self.newObsWeight
        else:
            self.leg1 = legs[0]
            self.leg2 = legs[1]

        return [self.leg1, self.leg2] # left, right




    def getShelfFrame(self):
        # with respect to the frame of /base_scan
        legs = self.findLegs()
        ori_x = (legs[0][0] + legs[1][0]) / 2.
        ori_y = (legs[0][1] + legs[1][1]) / 2.

        left_leg = legs[0]
        if legs[0][1] < legs[1][1]:
            left_leg = legs[1]

        rotAngle = atan2(ori_x - left_leg[0], left_leg[1] - ori_y)

        return [ori_x, ori_y], rotAngle

    def tf2PoseStamped(self, xy, ori):
        shelfPoseMsg = PoseStamped()
        shelfPoseMsg.pose.position.x = xy[0]
        shelfPoseMsg.pose.position.y = xy[1]
        shelfPoseMsg.pose.position.z = 0.0
        shelfPoseMsg.pose.orientation.x = ori[0]
        shelfPoseMsg.pose.orientation.y = ori[1]
        shelfPoseMsg.pose.orientation.z = ori[2]
        shelfPoseMsg.pose.orientation.w = ori[3]
        shelfPoseMsg.header.frame_id = 'base_laser_link'
        shelfPoseMsg.header.stamp = rospy.Time.now()
        return shelfPoseMsg

    def publish2TF(self):
        answer = 'n'
        ask = True
        while not rospy.is_shutdown():
            # check if human calibration is done
            shelfOri, shelfRot = self.getShelfFrame()
            legs = self.findLegs()
            

            if not self.calibrated:
                try:
                    self.br.sendTransform((shelfOri[0], shelfOri[1], 0),
                                     tf.transformations.quaternion_from_euler(0, 0, shelfRot),
                                     rospy.Time.now(),
                                     "/shelf_frame",     # child
                                     "/base_laser_link"      # parent
                                     )

                    self.br.sendTransform((legs[0][0], legs[0][1], 0), \
                                     tf.transformations.quaternion_from_euler(0, 0, shelfRot), \
                                     rospy.Time.now(),\
                                     "/left_leg",   \
                                     "/base_laser_link")

                    self.br.sendTransform((legs[1][0], legs[1][1], 0), \
                                     tf.transformations.quaternion_from_euler(0, 0, shelfRot), \
                                     rospy.Time.now(),\
                                     "/right_leg",   \
                                     "/base_laser_link")

                    self.pubShelfSep.publish(self.tf2PoseStamped(shelfOri, tf.transformations.quaternion_from_euler(0, 0, shelfRot)))
                except:
                    continue
                
            
            if self.priorAvailable:
                try:
                    newOri, newRot = self.listener.lookupTransform("/odom_combined", "/shelf_frame", rospy.Time(0))
                    newL, newRL = self.listener.lookupTransform("/base_laser_link", "/odomL", rospy.Time(0))
                    newR, newRR = self.listener.lookupTransform("/base_laser_link", "/odomR", rospy.Time(0))
                    self.priorLeft = newL
                    self.priorRight = newR
                except:
                    continue
            else:
                try:
                    newOri, newRot = self.listener.lookupTransform("/odom_combined", "/shelf_frame", rospy.Time(0))
                    newL, newRL = self.listener.lookupTransform("/base_laser_link", "/left_leg", rospy.Time(0))
                    newR, newRR = self.listener.lookupTransform("/base_laser_link", "/right_leg", rospy.Time(0))
                except Exception, e:
                    print e
                    continue
                    
            if self.reCalibration and math.sqrt((newOri[0]-self.priorOri[0]) **2 + (newOri[1]-self.priorOri[1]) **2) < 0.1:
                rospy.loginfo('reCalibrated!')
                while not rospy.is_shutdown(): # make sure the odomL and odomR are updated
                    try:
                        newOri, newRot = self.listener.lookupTransform("/odom_combined", "/shelf_frame", rospy.Time(0))
                        newL, newRL = self.listener.lookupTransform("/odom_combined", "/left_leg", rospy.Time(0))
                        newR, newRR = self.listener.lookupTransform("/odom_combined", "/right_leg", rospy.Time(0))
                        self.odomL = newL
                        self.odomR = newR
                        self.priorOri = newOri
                        self.priorRot = newRot
                        self.calibrated = True
                        self.reCalibration = False
                        break
                    except:
                        continue

            if not self.calibrated and ask:
                answer = raw_input("Is the current shelf pose estimation good? (y/n)")

                if answer == 'y' or answer == 'yes':
                    self.calibrated = True
                    ask = False
                    self.priorAvailable = True
                    print colored('human calibration of shelf pose is done', 'yellow', 'on_white')
                    print colored('prior position of the shelf is: X = %4f, Y = %4f' % (shelfOri[0], shelfOri[1]), 'yellow', 'on_white')
                    self.priorOri = newOri
                    self.priorRot = newRot
                    self.priorLeft = legs[0]
                    self.priorRight = legs[1]
                    while not rospy.is_shutdown():
                        try:
                            self.odomL, odomRL = self.listener.lookupTransform("/odom_combined", "/left_leg", rospy.Time(0))
                            self.odomR, odomRR = self.listener.lookupTransform("/odom_combined", "/right_leg", rospy.Time(0))
                            break
                        except:
                            rospy.sleep(0.4)

            

            # check in the odometry frame
            if self.calibrated:
                if math.sqrt((newOri[0]-self.priorOri[0]) **2 + (newOri[1]-self.priorOri[1]) **2) > 0.16:
                    rospy.logwarn('something is wrong with shelf pose estimation!!!!!!!!!! RECALIBRATING')
                    self.calibrated = False
                    self.reCalibration = True
                else:
                    self.br.sendTransform((shelfOri[0], shelfOri[1], 0),
                                     tf.transformations.quaternion_from_euler(0, 0, shelfRot),
                                     rospy.Time.now(),
                                     "/shelf_frame",     # child
                                     "/base_laser_link"      # parent
                                     )

                    self.br.sendTransform((legs[0][0], legs[0][1], 0), \
                                     tf.transformations.quaternion_from_euler(0, 0, shelfRot), \
                                     rospy.Time.now(),\
                                     "/left_leg",   \
                                     "/base_laser_link")

                    self.br.sendTransform((legs[1][0], legs[1][1], 0), \
                                     tf.transformations.quaternion_from_euler(0, 0, shelfRot), \
                                     rospy.Time.now(),\
                                     "/right_leg",   \
                                     "/base_laser_link")

                    self.br.sendTransform((shelfOri[0], shelfOri[1] + 0.1515 + self.binOffset, 1.21),
                                     tf.transformations.quaternion_from_euler(0, 0, shelfRot),
                                     rospy.Time.now(),
                                     "/shelf_bin_A",     # child
                                     "/base_laser_link"      # parent
                                     )

                    self.br.sendTransform((shelfOri[0], shelfOri[1] - 0.1515 + self.binOffset, 1.21),
                                     tf.transformations.quaternion_from_euler(0, 0, shelfRot),
                                     rospy.Time.now(),
                                     "/shelf_bin_B",     # child
                                     "/base_laser_link"      # parent
                                     )

                    self.br.sendTransform((shelfOri[0], shelfOri[1] - 0.4303 + self.binOffset, 1.21),
                                     tf.transformations.quaternion_from_euler(0, 0, shelfRot),
                                     rospy.Time.now(),
                                     "/shelf_bin_C",     # child
                                     "/base_laser_link"      # parent
                                     )

                    self.br.sendTransform((shelfOri[0], shelfOri[1] + 0.1515 + self.binOffset, 1),
                                     tf.transformations.quaternion_from_euler(0, 0, shelfRot),
                                     rospy.Time.now(),
                                     "/shelf_bin_D",     # child
                                     "/base_laser_link"      # parent
                                     )

                    self.br.sendTransform((shelfOri[0], shelfOri[1] - 0.1515 + self.binOffset, 1),
                                     tf.transformations.quaternion_from_euler(0, 0, shelfRot),
                                     rospy.Time.now(),
                                     "/shelf_bin_E",     # child
                                     "/base_laser_link"      # parent
                                     )

                    self.br.sendTransform((shelfOri[0], shelfOri[1] - 0.4303 + self.binOffset, 1),
                                     tf.transformations.quaternion_from_euler(0, 0, shelfRot),
                                     rospy.Time.now(),
                                     "/shelf_bin_F",     # child
                                     "/base_laser_link"      # parent
                                     )

                    self.br.sendTransform((shelfOri[0], shelfOri[1] + 0.1515 + self.binOffset, 0.78),
                                     tf.transformations.quaternion_from_euler(0, 0, shelfRot),
                                     rospy.Time.now(),
                                     "/shelf_bin_G",     # child
                                     "/base_laser_link"      # parent
                                     )

                    self.br.sendTransform((shelfOri[0], shelfOri[1] - 0.1515 + self.binOffset, 0.78),
                                     tf.transformations.quaternion_from_euler(0, 0, shelfRot),
                                     rospy.Time.now(),
                                     "/shelf_bin_H",     # child
                                     "/base_laser_link"      # parent
                                     )

                    self.br.sendTransform((shelfOri[0], shelfOri[1] - 0.4303 + self.binOffset, 0.78),
                                     tf.transformations.quaternion_from_euler(0, 0, shelfRot),
                                     rospy.Time.now(),
                                     "/shelf_bin_I",     # child
                                     "/base_laser_link"      # parent
                                     )

                    self.br.sendTransform((shelfOri[0], shelfOri[1] + 0.1515 + self.binOffset, 0.51),
                                     tf.transformations.quaternion_from_euler(0, 0, shelfRot),
                                     rospy.Time.now(),
                                     "/shelf_bin_J",     # child
                                     "/base_laser_link"      # parent
                                     )

                    self.br.sendTransform((shelfOri[0], shelfOri[1] - 0.1515 + self.binOffset, 0.51),
                                     tf.transformations.quaternion_from_euler(0, 0, shelfRot),
                                     rospy.Time.now(),
                                     "/shelf_bin_K",     # child
                                     "/base_laser_link"      # parent
                                     )

                    self.br.sendTransform((shelfOri[0], shelfOri[1] - 0.4303 + self.binOffset, 0.51),
                                     tf.transformations.quaternion_from_euler(0, 0, shelfRot),
                                     rospy.Time.now(),
                                     "/shelf_bin_L",     # child
                                     "/base_laser_link"      # parent
                                     )

                    self.pubShelfSep.publish(self.tf2PoseStamped(shelfOri, tf.transformations.quaternion_from_euler(0, 0, shelfRot)))




            if self.priorAvailable:
                    self.br.sendTransform(self.odomL, \
                                     tf.transformations.quaternion_from_euler(0, 0, shelfRot), \
                                     rospy.Time.now(),\
                                     "/odomL",   \
                                     "/odom_combined")

                    self.br.sendTransform(self.odomR, \
                                     tf.transformations.quaternion_from_euler(0, 0, shelfRot), \
                                     rospy.Time.now(),\
                                     "/odomR",   \
                                     "/odom_combined")          
            



            self.rate.sleep()
