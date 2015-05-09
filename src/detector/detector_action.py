#!/usr/bin/env python
import moveit_commander

import rospy

import actionlib

import amazon_challenge_bt_actions.msg

import grasping.grasping_lib as grasping_lib


from std_msgs.msg import String
import sys

import tf
import threading
from grasping.myTypes import *
from simtrack_nodes.srv import *
from vision.srv import StartAggregator


class superDetector(object):
    # create messages that are used to publish feedback/result
    _feedback = amazon_challenge_bt_actions.msg.BTFeedback()
    _result = amazon_challenge_bt_actions.msg.BTResult()

    def __init__(self, name):
        self._action_name = name
        rospy.init_node(self._action_name)
        self._as = actionlib.SimpleActionServer(self._action_name, amazon_challenge_bt_actions.msg.BTAction,\
            execute_cb=self.receive_update, auto_start = False)

        self._as.start()
        self.pub_rate = rospy.Rate(50)
        self.listener = tf.TransformListener()
        rospy.Subscriber("/amazon_next_task", String, self.get_task)
        self._item = ''
        self._bin = ''
        self.trials = 10
        self.obsN = 4
        self.br = tf.TransformBroadcaster()
        self.tp = []
        self.vThresh = 0.1
        self.updating = False
        self.lock = threading.Lock()
        rospy.wait_for_service('/simtrack/tracker_switch_camera', 10)
        self.cameraSrv = rospy.ServiceProxy('/simtrack/tracker_switch_camera', SwitchCamera)
        rospy.wait_for_service('/simtrack/tracker_switch_objects', 10)
        self.objSrv = rospy.ServiceProxy('/simtrack/tracker_switch_objects', SwitchObjects)
        rospy.wait_for_service('/aggregate_cloud', 10)
        self.segSrv = rospy.ServiceProxy('/aggregate_cloud', StartAggregator)


        self.left_arm_joint_pos_dict = rospy.get_param('/left_arm_joint_pos_dict')
        self.right_arm_joint_pos_dict = rospy.get_param('/right_arm_joint_pos_dict')
        self.torso_joint_pos_dict = rospy.get_param('/torso_joint_pos_dict')
        while not rospy.is_shutdown():
            try:
                self.left_arm = moveit_commander.MoveGroupCommander('left_arm')
                self.right_arm = moveit_commander.MoveGroupCommander('right_arm')
                self.torso = moveit_commander.MoveGroupCommander('torso')
                break
            except:
                pass

    def flush(self):
        self._item = ""
        self._bin = ""




    def my_pub(self):

        if self.updating:
            return
        # publish info to the console for the user
        rospy.loginfo('Starting Detecting')


        while not rospy.is_shutdown():
            if len(self._item) == 0 or len(self._bin) == 0:
                continue
            try:
                self.br.sendTransform(self.tp[0], self.tp[1], rospy.Time.now(),\
                                         "/" + self._item + "_detector",   \
                                         '/' + 'shelf_' + self._bin)
                self.pub_rate.sleep()
            except:
                continue



    def get_task(self, msg):
        text = msg.data
        text = text.replace('[','')
        text = text.replace(']','')
        words = text.split(',')
        self._bin = words[0]
        self._item = words[1]


    

    def getSimTrackUpdate(self):

        rospy.sleep(2)

        self.obsAccumulation = []
        enough = False
        good = False
        for i in range(self.trials):
            rospy.sleep(0.01)
            try:
                self.tp = grasping_lib.getGraspFrame(self.listener, '/' + 'shelf_' + self._bin, '/' + self._item)
                self.obsAccumulation.append(self.tp[0])
                if len(self.obsAccumulation) == self.obsN:
                    enough = True
            except:
                continue
            if enough:
                break

        if enough:
            # rospy.loginfo('validation')
            good = self.validate()

        if good:
            return True
        else:
            return False

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

    def receive_update(self,goal):

        self.lock.acquire()
        rospy.loginfo('Goal Received')
        self.updating = True

        self.objSrv.call([self._item])
        

        # try with kinect
        rospy.loginfo('try to update object pose with kinect')
        self.cameraSrv.call(0)
        if self.getSimTrackUpdate():
            rospy.loginfo('object pose UPDATED')
            self.set_status('SUCCESS')
            self.updating = False
            self.lock.release()
            return


        rospy.loginfo('try to update object pose with left arm camera')
        self.cameraSrv.call(1)
        detect = True
        try:
            self.torso.set_joint_value_target(self.torso_joint_pos_dict['detector'][self.get_row()])
            self.torso.go()
            self.left_arm.set_joint_value_target(self.left_arm_joint_pos_dict['detector'][self.get_row()])
            self.left_arm.go(wait=True)
        except:
            rospy.logerr('can not move left arm to detecting pose')
            detect = False


        if detect:
            if self.getSimTrackUpdate():
                rospy.loginfo('object pose UPDATED')
                if self.move_arm_to_init('left_arm'):
                    self.set_status('SUCCESS')
                else:
                    self.set_status('FAILURE')
                self.updating = False
                self.lock.release()
                return

        if not self.move_arm_to_init('left_arm'):
            self.set_status("FAILURE")
            return



        rospy.loginfo('try to update object pose with right arm camera')
        self.cameraSrv.call(2)
        detect = True
        try:
            self.torso.set_joint_value_target(self.torso_joint_pos_dict['detector'][self.get_row()])
            self.torso.go()
            self.right_arm.set_joint_value_target(self.right_arm_joint_pos_dict['detector'][self.get_row()])
            self.right_arm.go(wait=True)
        except:
            rospy.logerr('can not move right arm to detecting pose')
            detect = False

        if detect:
            if self.getSimTrackUpdate():
                rospy.loginfo('object pose UPDATED')
                if self.move_arm_to_init('right_arm'):
                    self.set_status('SUCCESS')
                else:
                    self.set_status('FAILURE')
                self.updating = False
                self.lock.release()
                return

        if not self.move_arm_to_init('right_arm'):
            self.set_status("FAILURE")
            return

        rospy.loginfo('try to update object pose with point cloud segmentation')
        self.segSrv.call(1)

        detect = True
        try:
            self.torso.set_joint_value_target(self.torso_joint_pos_dict['detector'][self.get_row()])
            self.torso.go()
        except:
            rospy.logerr('can not move torso to detecting height')
            detect = False

        if detect:
            if self.getSimTrackUpdate():
                rospy.loginfo('object pose UPDATED')
                self.set_status('SUCCESS')
                self.updating = False
                self.lock.release()
                return


        

        rospy.loginfo('object pose CANNOT be UPDATED')
        self.set_status('FAILURE')
        self.updating = False
        self.lock.release()
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
