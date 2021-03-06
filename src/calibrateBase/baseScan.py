#! /usr/bin/python

import rospy
import tf
from numpy import linalg as LA
from math import *
from sensor_msgs.msg import LaserScan, JointState
from laser_geometry import LaserProjection
import sensor_msgs.point_cloud2 as pc2
import math
from geometry_msgs.msg import PoseStamped
from select import select
import sys
import pickle
from collections import namedtuple
import rospkg
from grasping.myTypes import *
import random
from std_srvs.srv import Empty, EmptyResponse

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
        self.rate = rospy.Rate(20.0)
        self.calibrated = False
        self.reCalibration = False
        self.priorOri_in_base_laser_link = [] # in base_laser_link frame
        self.priorLeft_in_base_laser_link = [] # in base_laser_link frame
        self.priorRight_in_base_laser_link = [] # in base_laser_link frame
        self.odomL = [] # in odom_combined frame
        self.odomR = [] # in odom_combined frame
        self.priorAvailable = False
        self.newObsWeight = 0.2
        self.offsetXY = [-0.044, 0]
        self.binOffset = 0.02
        self.pubShelfSep = rospy.Publisher('pubShelfSep', PoseStamped)
        self.reCalibrationCount = 4
        self.tolerance = 0.1
        self.updateRounds = 100
        self.asyncRate = 20
        self.limitInitX = True
        self.xLimit = 0.1
        self.emergencyThreshold = 30*20
        self.emergencyTimeWindow = 60

        while not rospy.is_shutdown():
            try:
                self.offsetXY = rospy.get_param('/base_scan/offset')
                break
            except:
                rospy.loginfo('[shelf publisher]: waiting for base scan offset param')
                rospy.sleep(random.uniform(0,1))
                continue

        self.start_srv = rospy.Service(rospy.get_name() + '/start', Empty, self.start_srv_callback)
        self._start = False

        # backup human supervised info for emergency
        rp = rospkg.RosPack()
        try:
            dict_fp = rp.get_path('amazon_challenge_grasping')

        except:
            rospy.logerr('[shelf_publisher]: emergency file not found!')
            sys.exit(1)
        self.fileName = dict_fp + '/config/shelf_emergency.dat'
        self.emergency = {}


    def start_srv_callback(self, req):
        rospy.loginfo('[shelf_publisher]: starting shelf publisher!')
        if self._start:
            rospy.logwarn('[shelf_publisher]: already started!!!!')
        self._start = True
        return EmptyResponse()

    def raw_input_with_timeout(prompt, timeout=1.0):
        print prompt,    
        timer = threading.Timer(timeout, thread.interrupt_main)
        astring = None
        try:
            timer.start()
            astring = raw_input(prompt)
        except KeyboardInterrupt:
            pass
        timer.cancel()
        return astring

    def callback(self,data):
        self.rangeData = data

    def refreshRangeData(self):
        self.rangeData = LaserScan() # flush
        while len(self.rangeData.ranges) == 0 and not rospy.is_shutdown():
            rospy.sleep(0.04)

    def getStatus(self):
        return self.calibrated

    def getCloud(self):

        # self.refreshRangeData()
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
            for i in range(len(x)):
                radius.append(math.sqrt((x[i]-self.priorLeft_in_base_laser_link[0])**2 + (y[i] - self.priorLeft_in_base_laser_link[1])**2))
            n1 = radius.index(min(radius))

            radius = []
            for i in range(len(x)):
                radius.append(math.sqrt((x[i]-self.priorRight_in_base_laser_link[0])**2 + (y[i] - self.priorRight_in_base_laser_link[1])**2))
            n2 = radius.index(min(radius))


            leg1 = [x[n1], y[n1]]
            leg2 = [x[n2], y[n2]]
        else:
            # Assuming there is nothing between the robot and the shelf
            if self.limitInitX:
                y = [y[i] for i in range(len(y)) if x[i] >= self.xLimit]
                x = [x[i] for i in range(len(x)) if x[i] >= self.xLimit]

            for i in range(len(x)):
                radius.append(math.sqrt(x[i]**2 + y[i]**2))
            n = radius.index(min(radius))
            
            x2 = [x[i] for i in range(len(x)) if math.sqrt( (x[i]-x[n])**2 + (y[i]-y[n])**2 ) > 0.7]
            y2 = [y[i] for i in range(len(y)) if math.sqrt( (x[i]-x[n])**2 + (y[i]-y[n])**2 ) > 0.7]
            radius2 = []
            

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

        return [ori_x, ori_y], rotAngle, legs

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


    def saveEmergency(self):
        with open(self.fileName, 'wb') as handle:
            pickle.dump(self.emergency, handle)

    def loadEmergency(self):
       
        with open(self.fileName, 'rb') as handle:
            self.emergency = pickle.load(handle)


    def publish2TF(self):
        answer = 'n'
        ask = True
        u = 0
        shelfN = 0
        recalibrateCount = 0
        emergencyCount = 0
        t_init = rospy.Time.now()

        while not rospy.is_shutdown():
            self.rate.sleep()
            # check if human calibration is done
            shelfOri, shelfRot, legs = self.getShelfFrame()


            if self.reCalibration:
                u = 0

            if not self.calibrated:

                self.br.sendTransform((shelfOri[0], shelfOri[1], 0),
                                      tf.transformations.quaternion_from_euler(0, 0, shelfRot),
                                      rospy.Time.now(),
                                      "/shelf_frame",     # child
                                      "/base_laser_link"      # parent
                                      )

                self.br.sendTransform((legs[0][0], legs[0][1], 0), \
                                      tf.transformations.quaternion_from_euler(0, 0, shelfRot), \
                                      rospy.Time.now(), \
                                      "/left_leg", \
                                      "/base_laser_link")

                self.br.sendTransform((legs[1][0], legs[1][1], 0), \
                                      tf.transformations.quaternion_from_euler(0, 0, shelfRot), \
                                      rospy.Time.now(), \
                                      "/right_leg", \
                                      "/base_laser_link")


            
                
            
            if self.priorAvailable:
                while not rospy.is_shutdown():
                    try:
                        shelf_in_odom, shelf_rot_in_odom = self.listener.lookupTransform("/odom_combined", "/shelf_frame", rospy.Time(0))
                        self.priorLeft_in_base_laser_link, dummy = self.listener.lookupTransform("/base_laser_link", "/odomL", rospy.Time(0))
                        self.priorRight_in_base_laser_link, dummy = self.listener.lookupTransform("/base_laser_link", "/odomR", rospy.Time(0))
                        break
                    except:
                        continue
            else:
                try:
                    shelf_in_odom, shelf_rot_in_odom = self.listener.lookupTransform("/odom_combined", "/shelf_frame", rospy.Time(0))
                except Exception, e:
                    # print e
                    continue
                    
            if self.reCalibration and math.sqrt((shelf_in_odom[0]-self.priorOri_in_odom[0]) **2 + (shelf_in_odom[1]-self.priorOri_in_odom[1]) **2) <= self.tolerance:
                # rospy.sleep(2)
                recalibrateCount += 1
                if recalibrateCount == self.reCalibrationCount: # take recalibration only if it's stable
                    rospy.loginfo('reCalibrated!')
                    recalibrateCount = 0
                    u = 0
                    while not rospy.is_shutdown(): # make sure the odomL and odomR are updated
                        try:
                            ######## self.priorOri_in_odom, self.priorRot_in_odom = self.listener.lookupTransform("/odom_combined", "/shelf_frame", rospy.Time(0))
                            ######## self.odomL, self.odomL_rot = self.listener.lookupTransform("/odom_combined", "/left_leg", rospy.Time(0))
                            ######## self.odomR, self.odomR_rot = self.listener.lookupTransform("/odom_combined", "/right_leg", rospy.Time(0))
                            self.calibrated = True
                            self.reCalibration = False
                            rospy.loginfo("Prior origin in odom_combined: X = %4f, Y = %4f" % (self.priorOri_in_odom[0], self.priorOri_in_odom[1]))
                            break
                        except:
                            continue

            if not self.calibrated and ask:
                emergencyCount += 1
                if emergencyCount == self.emergencyThreshold:
                    self.loadEmergency()
                    self.odomL = self.emergency.odomL
                    self.odomL_rot = self.emergency.odomL_rot
                    self.odomR = self.emergency.odomR
                    self.odomR_rot = self.emergency.odomR_rot
                    emergency_timestamp = self.emergency.timestamp
                    if (rospy.Time.now() - emergency_timestamp).to_sec() > self.emergencyTimeWindow:
                        print ''
                        rospy.logwarn('***********************************************************************')
                        rospy.logwarn('[shelf_publisher] NOT ABLE TO RECOVER FROM CRASHING, GAME OVER')
                        rospy.logwarn('***********************************************************************')
                        continue

                    ask = False
                    self.calibrated = True
                    self.priorAvailable = True
                    self.priorOri_in_odom = shelf_in_odom
                    self.priorRot_in_odom = shelf_rot_in_odom
                    self.priorLeft_in_base_laser_link = legs[0]
                    self.priorRight_in_base_laser_link = legs[1]

                    print ''
                    rospy.logwarn('***********************************************************************')
                    rospy.logwarn('[shelf_publisher] EMERGENCY RECOVERY CALLED!!!!!!!!!!!!!!!!!!!!!!')
                    rospy.logwarn('***********************************************************************')
                else:
                    sys.stdout.write("\r [ROS time: %s] Is the current shelf pose estimation good? (y/n)" % rospy.get_time() )
                    sys.stdout.flush()


                    if self._start:
                        answer = 'y'

                    if answer == 'y' or answer == 'yes':
                        self.calibrated = True
                        ask = False
                        self.priorAvailable = True
                        self.priorOri_in_odom = shelf_in_odom
                        self.priorRot_in_odom = shelf_rot_in_odom
                        self.priorLeft_in_base_laser_link = legs[0]
                        self.priorRight_in_base_laser_link = legs[1]
                        print ""
                        rospy.loginfo("Human calibration finished")
                        rospy.loginfo("Prior origin in /odom_combined: X = %4f, Y = %4f" % (self.priorOri_in_odom[0], self.priorOri_in_odom[1]))
                        while not rospy.is_shutdown():
                            try:
                                self.odomL, self.odomL_rot = self.listener.lookupTransform("/odom_combined", "/left_leg", rospy.Time(0))
                                self.odomR, self.odomR_rot = self.listener.lookupTransform("/odom_combined", "/right_leg", rospy.Time(0))
                                break
                            except:
                                pass
                        self.emergency = shelfInfo(self.odomL, self.odomL_rot, self.odomR, self.odomR_rot, rospy.Time.now())
                        self.saveEmergency()
                    else:
                        continue

            
            # check in the odometry frame

            if self.priorAvailable:
                # print 'pub odom'
                self.br.sendTransform(self.odomL, \
                                 self.odomL_rot, \
                                 rospy.Time.now(),\
                                 "/odomL",   \
                                 "/odom_combined")

                self.br.sendTransform(self.odomR, \
                                 self.odomR_rot, \
                                 rospy.Time.now(),\
                                 "/odomR",   \
                                 "/odom_combined")
            if self.calibrated:
                u += 1
                shelfN += 1
                if math.sqrt((shelf_in_odom[0] - self.priorOri_in_odom[0]) **2 + (shelf_in_odom[1] - self.priorOri_in_odom[1]) **2) > self.tolerance:
                    rospy.logwarn('something is wrong with shelf pose estimation!!!!!!!!!! RECALIBRATING')
                    recalibrateCount = 0
                    u = 0
                    self.calibrated = False
                    self.reCalibration = True
                    continue
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

                    self.pubShelfSep.publish(self.tf2PoseStamped(shelfOri, tf.transformations.quaternion_from_euler(0, 0, shelfRot)))

                if u == self.updateRounds:
                    while not rospy.is_shutdown(): # make sure the odomL and odomR are updated
                        try:
                            self.priorOri_in_odom, self.priorRot_in_odom = self.listener.lookupTransform("/odom_combined", "/shelf_frame", rospy.Time(0))
                            self.odomL, self.odomL_rot = self.listener.lookupTransform("/odom_combined", "/left_leg", rospy.Time(0))
                            self.odomR, self.odomR_rot = self.listener.lookupTransform("/odom_combined", "/right_leg", rospy.Time(0))
                            rospy.loginfo("Prior origin in /odom_combined: X = %4f, Y = %4f" % (self.priorOri_in_odom[0], self.priorOri_in_odom[1]))
                            u = 0
                            break
                        except:
                            continue

                if (rospy.Time.now()-t_init).to_sec()>1.0:
                    t_init = rospy.Time.now()
                    self.emergency = shelfInfo(self.odomL, self.odomL_rot, self.odomR, self.odomR_rot, rospy.Time.now())
                    self.saveEmergency()
                    # print 'pub'
                    shelfN = 0
                    self.br.sendTransform((0, 0.1515 + self.binOffset, 1.21),
                                     tf.transformations.quaternion_from_euler(0, 0, 0),
                                     rospy.Time.now(),
                                     "/shelf_bin_A",     # child
                                     "/shelf_frame"      # parent
                                     )

                    self.br.sendTransform((0, - 0.1515 + self.binOffset, 1.21),
                                     tf.transformations.quaternion_from_euler(0, 0, 0),
                                     rospy.Time.now(),
                                     "/shelf_bin_B",     # child
                                     "/shelf_frame"      # parent
                                     )

                    self.br.sendTransform((0, - 0.4303 + self.binOffset, 1.21),
                                     tf.transformations.quaternion_from_euler(0, 0, 0),
                                     rospy.Time.now(),
                                     "/shelf_bin_C",     # child
                                     "/shelf_frame"      # parent
                                     )

                    self.br.sendTransform((0, 0.1515 + self.binOffset, 1),
                                     tf.transformations.quaternion_from_euler(0, 0, 0),
                                     rospy.Time.now(),
                                     "/shelf_bin_D",     # child
                                     "/shelf_frame"      # parent
                                     )

                    self.br.sendTransform((0,  - 0.1515 + self.binOffset, 1),
                                     tf.transformations.quaternion_from_euler(0, 0, 0),
                                     rospy.Time.now(),
                                     "/shelf_bin_E",     # child
                                     "/shelf_frame"      # parent
                                     )

                    self.br.sendTransform((0, - 0.4303 + self.binOffset, 1),
                                     tf.transformations.quaternion_from_euler(0, 0, 0),
                                     rospy.Time.now(),
                                     "/shelf_bin_F",     # child
                                     "/shelf_frame"      # parent
                                     )

                    self.br.sendTransform((0, 0.1515 + self.binOffset, 0.78),
                                     tf.transformations.quaternion_from_euler(0, 0, 0),
                                     rospy.Time.now(),
                                     "/shelf_bin_G",     # child
                                     "/shelf_frame"      # parent
                                     )

                    self.br.sendTransform((0, - 0.1515 + self.binOffset, 0.78),
                                     tf.transformations.quaternion_from_euler(0, 0, 0),
                                     rospy.Time.now(),
                                     "/shelf_bin_H",     # child
                                     "/shelf_frame"      # parent
                                     )

                    self.br.sendTransform((0, - 0.4303 + self.binOffset, 0.78),
                                     tf.transformations.quaternion_from_euler(0, 0, 0),
                                     rospy.Time.now(),
                                     "/shelf_bin_I",     # child
                                     "/shelf_frame"      # parent
                                     )

                    self.br.sendTransform((0, 0.1515 + self.binOffset, 0.51),
                                     tf.transformations.quaternion_from_euler(0, 0, 0),
                                     rospy.Time.now(),
                                     "/shelf_bin_J",     # child
                                     "/shelf_frame"      # parent
                                     )

                    self.br.sendTransform((0, - 0.1515 + self.binOffset, 0.51),
                                     tf.transformations.quaternion_from_euler(0, 0, 0),
                                     rospy.Time.now(),
                                     "/shelf_bin_K",     # child
                                     "/shelf_frame"      # parent
                                     )

                    self.br.sendTransform((0, - 0.4303 + self.binOffset, 0.51),
                                     tf.transformations.quaternion_from_euler(0, 0, 0),
                                     rospy.Time.now(),
                                     "/shelf_bin_L",     # child
                                     "/shelf_frame"      # parent
                                     )







            
                


            # self.rate.sleep()
