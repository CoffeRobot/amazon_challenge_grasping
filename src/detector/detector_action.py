#!/usr/bin/python
import moveit_commander

import rospy

import actionlib

import amazon_challenge_bt_actions.msg

import grasping.grasping_lib as grasping_lib
from std_srvs.srv import Empty, EmptyRequest


from std_msgs.msg import String
import sys

import tf
import threading
from grasping.myTypes import *
from simtrack_nodes.srv import *
from vision.srv import StartAggregator
import random
from grasping.generate_object_dict import *
from amazon_challenge_grasping.srv import BaseMove, BaseMoveResponse, BaseMoveRequest
from amazon_challenge_bt_actions.srv import *
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
import numpy as np
from geometry_msgs.msg import PoseStamped

class superDetector(object):
    # create messages that are used to publish feedback/result
    _feedback = amazon_challenge_bt_actions.msg.BTFeedback()
    _result = amazon_challenge_bt_actions.msg.BTResult()

    def __init__(self, name):
        self._action_name = name
        rospy.init_node(self._action_name)
        self._as = actionlib.SimpleActionServer(self._action_name, amazon_challenge_bt_actions.msg.BTAction,\
            execute_cb=self.receive_update, auto_start = False)

        # self._as.start()
        self._exit = False
        self.pub_rate = rospy.Rate(50)
        rospy.Subscriber("/amazon_next_task", String, self.get_task)
        self._item = ''
        self._bin = ''
        self._binItems = []
        self.trials = 10
        self.obsN = 4
        self.br = tf.TransformBroadcaster()
        self.tp = []
        self.vThresh = 0.1
        self.updating = False
        self.lock = threading.Lock()
        self.preempted = False
        
        self.get_services()
        self._shelf_pose_sub = rospy.Subscriber("/pubShelfSep", PoseStamped, self.get_shelf_pose)
        self._got_shelf_pose = False
        
        self.simTrackUsed = True
        self.found = False

        while not rospy.is_shutdown():
            try:
                self.left_arm_joint_pos_dict = rospy.get_param('/left_arm_joint_pos_dict')
                self.right_arm_joint_pos_dict = rospy.get_param('/right_arm_joint_pos_dict')
                self.torso_joint_pos_dict = rospy.get_param('/torso_joint_pos_dict')
                self._timeout = rospy.get_param(rospy.get_name() + '/timeout')
                self.base_pos_dict = rospy.get_param('/base_pos_dict')
                self.dictObj = objDict()
                break
            except:
                rospy.sleep(random.uniform(0,2))
                continue

        while not rospy.is_shutdown():
            try:
                self.listener = tf.TransformListener()
                self.robot = moveit_commander.RobotCommander()
                self.left_arm = self.robot.get_group('left_arm')
                self.right_arm = self.robot.get_group('right_arm')
                self.torso = self.robot.get_group('torso')
                self._arms = self.robot.get_group('arms')
                break
            except:
                rospy.sleep(random.uniform(0,2))
                pass

        self._as.start()
        rospy.loginfo('SuperDetector ready')

    def flush(self):
        self._item = ""
        self._bin = ""
        self._binItems = []

    def get_shelf_pose(self, msg):
        self._shelf_pose = msg
        self._got_shelf_pose = True

    def get_bm_srv(self):
        while not rospy.is_shutdown():
            try:
                rospy.wait_for_service('/base_move_server/move', 5.0)
                rospy.wait_for_service('/base_move_server/preempt', 5.0)
                break
            except:
                rospy.loginfo('[' + rospy.get_name() + ']: waiting for base move server')
                continue

        self._bm_move_srv = rospy.ServiceProxy('/base_move_server/move', BaseMove)
        self._bm_preempt_srv = rospy.ServiceProxy('/base_move_server/preempt', Empty)

    def get_services(self):
        while not rospy.is_shutdown():

            if self._exit:
                return False

            try:
                # rospy.wait_for_service('/simtrack/switch_camera', 1.0)
                rospy.wait_for_service('/simtrack/switch_objects', 1.0)
                rospy.wait_for_service('/aggregate_cloud', 1.0)
                rospy.wait_for_service('/bin_trigger', 1.0)
                break
            except:
                rospy.loginfo('[detector]: waiting for simtrack and segmentation services')
                pass


        # self.cameraSrv = rospy.ServiceProxy('/simtrack/switch_camera', SwitchCamera)
        self.objSrv = rospy.ServiceProxy('/simtrack/switch_objects', SwitchObjects)
        self.segSrv = rospy.ServiceProxy('/aggregate_cloud', StartAggregator)
        self.binSrv = rospy.ServiceProxy('bin_trigger', BinTrigger)

        return True




    def my_pub(self):

        # publish info to the console for the user
        r = rospy.Rate(2.0)


        while not rospy.is_shutdown():
            r.sleep()
            if self.updating:
                continue
            if not self.found:
                continue

            if len(self._item) == 0 or len(self._bin) == 0:
                continue
            if self.simTrackUsed:
                try:
                    self.br.sendTransform(self.tp[0], self.tp[1], rospy.Time.now(),\
                                             "/" + self._item + "_detector",   \
                                             '/' + 'shelf_' + self._bin)
                except:
                    continue
            else:
                try:
                    self.br.sendTransform(self.tp[0], self.tp[1], rospy.Time.now(),\
                                             "/" + self._item + "_detector_seg",   \
                                             '/' + 'shelf_' + self._bin)
                except:
                    continue


    def get_task(self, msg):
        text = msg.data
        text = text.replace('[','')
        text = text.replace(']','')
        words = text.split(',')
        self._bin = words[0]
        self._item = words[1]

    def updateBinItems(self):
        text = self.binSrv.call()
        text = text.message
        self._binItems = []
        for it in text:
            self._binItems.append(it)
        rospy.loginfo('bin_items updated')



    

    def getSimTrackUpdate(self):

        self.obsAccumulation = []
        enough = False
        good = False
        for i in range(self.trials):
            if self._exit:
                 self._success = False
                 return False
            if self.simTrackUsed:
                try:
                    self.tp = grasping_lib.getGraspFrame(self.listener, '/' + 'shelf_' + self._bin, '/' + self._item, True)
                    self.obsAccumulation.append(self.tp[0])
                    if len(self.obsAccumulation) == self.obsN:
                        enough = True
                except:
                    continue
                if enough:
                    break
            else:
                try:
                    self.tp = grasping_lib.getGraspFrame(self.listener, '/' + 'shelf_' + self._bin, '/' + self._item + '_scan', False)
                    self.obsAccumulation.append(self.tp[0])
                    if len(self.obsAccumulation) == self.obsN:
                        enough = True
                except:
                    continue
                if enough:
                    break


        if enough:
            good = self.validate()

        if good:
            return True
        else:
            return False

        

    def getAllSimtrackItems(self):

        '''
        this returns the xyz of non-target objects in the current bin
        '''


        refs = []
        foundItems = []

        for it in self._binItems:

            if it == self._item:
                continue
            
            for i in range(4):
                try:
                    refTmp = self.listener.lookupTransform('/base_footprint', "/" + it, rospy.Time(0))
                    refs.append(refTmp[0])
                    foundItems.append(it)
                    rospy.loginfo('got %s' % it)
                    break
                except:
                    rospy.sleep(0.4)

        return foundItems, refs




    def move_arm_to_init(self, arm_name):

        if arm_name == 'right_arm':
            try:
                self.right_arm.set_joint_value_target(self.right_arm_joint_pos_dict['start'])
                self.right_arm.go(wait=True)
            except:
                rospy.logerr('can not move right arm to init pose')
                return False
        elif arm_name == 'left_arm':
            try:
                self.left_arm.set_joint_value_target(self.left_arm_joint_pos_dict['start'])
                self.left_arm.go(wait=True)
            except:
                rospy.logerr('can not move left arm to init pose')
                return False

        return True

    def timer_callback(self, event):
        self.preempted = True
        rospy.logerr('[' + rospy.get_name() + ']: TIMED OUT!')

        # pull the base back 60 cm

        self.left_arm.stop()
        self.right_arm.stop()

        r = rospy.Rate(1.0)
        while not self._got_shelf_pose:
            rospy.loginfo('[' + rospy.get_name() + ']: waiting for shelf pose')
            r.sleep()

        base_pos_goal = [-1.42, -self._shelf_pose.pose.position.y, 0.0, 0.0, 0.0, 0.0]

        angle = base_pos_goal[5]
        pos = base_pos_goal[0:2]

        req = BaseMoveRequest()
        req.x = pos[0]
        req.y = pos[1]
        req.theta = angle

        self.get_bm_srv()
        self._bm_preempt_srv.call(EmptyRequest())
        res = self._bm_move_srv.call(req)

        left_arm_joint_pos_goal = self.left_arm_joint_pos_dict['start']
        right_arm_joint_pos_goal = self.right_arm_joint_pos_dict['start']

        joint_pos_goal = left_arm_joint_pos_goal + right_arm_joint_pos_goal

        self._arms.set_joint_value_target(joint_pos_goal)
        self._arms.go()
        self._exit = True

    def execute_exit(self):
        if self._exit:
            self._success = False
            self.timer.shutdown()
            self.lock.release()
            self.set_status("FAILURE")
            return True

        return False

    def setFailureOnExit(self):
        self._failure_on_exit = True
        while not self._exit:
            rospy.sleep(0.4)
        self.set_status('FAILURE')


    def composeSegRefs(self, refs):
        ret = []
        for r in refs:
            for c in r:
                ret.append(c)
        return ret
    def receive_update(self,goal):

        rospy.sleep(1.0)
        self.updateBinItems()

        self._failure_on_exit=False
        self.preempted = False
        self._exit = False
        self.timer = rospy.Timer(rospy.Duration(self._timeout), self.timer_callback, oneshot=True)

        self.lock.acquire()
        rospy.loginfo('Goal Received')
        self.updating = True
        self.found = False


        try:
            objSpec = self.dictObj.getEntry(self._item)
            simtrackEnabled = objSpec.simtrack
        except:
            simtrackEnabled = True

        if not self.get_services():
            if self.execute_exit():
                return

        self.objSrv.call(self._binItems)
        rospy.loginfo('Let\'s give simtrack some time ......')
        rospy.sleep(10.0)

        if simtrackEnabled:
            rospy.loginfo('try to update object pose with IMAX camera')
            self.simTrackUsed = True
            detect = True

            if not self.get_services():
                if self.execute_exit():
                    return


            if self.execute_exit():
                return


            if self.getSimTrackUpdate():
                self.found = True
                rospy.loginfo('object pose UPDATED')
                if not self.preempted:
                    self.set_status('SUCCESS')
                else:
                    self.setFailureOnExit()
                self.updating = False
                self.lock.release()
                self.timer.shutdown()
                return




        rospy.loginfo('try to update object pose with point cloud segmentation')


        detect = True
        self.simTrackUsed = False

        if not self.get_services():
            if self.execute_exit():
                return
        
        foundItems, refs = self.getAllSimtrackItems()

        if len(self._binItems) - len(foundItems) > 2:
            rospy.logerr('Do not have enough info for reasoning in object segmentation')
            if not self.preempted:
                self.set_status('FAILURE')
            else:
                self.setFailureOnExit()

            self.updating = False
            self.timer.shutdown()
            self.lock.release()
            return

        if not self.get_services():
            if self.execute_exit():
                return


        try:
            segResult = self.segSrv.call(0, foundItems, self.composeSegRefs(refs))
        except:
            rospy.logerr('can not finish init of segmentation pose')
            detect = False


        if not segResult:
            rospy.logerr('Segmentation returned FAILURE')
            if not self.preempted:
                self.set_status('FAILURE')
            else:
                self.setFailureOnExit()

            self.segSrv.call(1, [], [])
            self.updating = False
            self.timer.shutdown()
            self.lock.release()
            return


        if detect:
            if self.getSimTrackUpdate():
                self.found = True
                rospy.loginfo('object pose UPDATED')
                if not self.preempted:
                    self.set_status('SUCCESS')
                else:
                    self.setFailureOnExit()
            else:
                if not self.preempted:
                    self.set_status('FAILURE')
                else:
                    self.setFailureOnExit()
            self.updating = False
            if not self.get_services() and not self._failure_on_exit:
                if self.execute_exit():
                    return
            self.segSrv.call(1, [], []) # from this point on, it's gonna be the detector(this) publishing only
            self.timer.shutdown()
            self.lock.release()
            return



        

        rospy.loginfo('object pose CANNOT be UPDATED')
        if not self.preempted:
            self.set_status('FAILURE')
        else:
            self.setFailureOnExit()
        self.updating = False
        self.lock.release()
        self.timer.shutdown()
        return





    def validate(self):
        if len(self.obsAccumulation) < self.obsN:
            return False
        
        Xs = [t[0] for t in self.obsAccumulation]
        Ys = [t[1] for t in self.obsAccumulation]
        Zs = [t[2] for t in self.obsAccumulation]

        xVariance = self.variance(Xs)
        yVariance = self.variance(Ys)
        zVariance = self.variance(Zs)

        if xVariance > self.vThresh or yVariance > self.vThresh or zVariance > self.vThresh:
            return False
        else:
            return True


    def set_status(self, status):
        if status == 'SUCCESS':
            self._feedback.status = 1
            self._result.status = self._feedback.status
            rospy.loginfo('Action %s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)
        elif status == 'FAILURE':
            self._feedback.status = 2
            self._result.status = self._feedback.status
            rospy.loginfo('Action %s: Failed' % self._action_name)
            self._as.set_succeeded(self._result)
        else:
            rospy.logerr('Action %s: has a wrong return status' % self._action_name)

    def variance(self, l):
        lMean = sum(l) / len(l)
        lVariance = 0
        for x in l:
            lVariance += (x - lMean)**2
        return lVariance / len(l)


    def get_row(self):
        while not rospy.is_shutdown():
            try:
                if self._bin=='bin_A' or self._bin=='bin_B' or self._bin=='bin_C':
                    return 'row_1'

                elif self._bin=='bin_D' or self._bin=='bin_E' or self._bin=='bin_F':
                    return 'row_2'

                elif self._bin=='bin_G' or self._bin=='bin_H' or self._bin=='bin_I':
                    return 'row_3'

               
                elif self._bin=='bin_J' or self._bin=='bin_K' or self._bin=='bin_L':
                    return 'row_4'
            except:
                pass



if __name__ == '__main__':
    pubDetector = superDetector('amazon_detector')
    pubDetector.my_pub()
    rospy.spin()
