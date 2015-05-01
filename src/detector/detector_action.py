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

    def receive_update(self,goal):
        self.lock.acquire()
        rospy.loginfo('Goal Received')
        poseAccumulation = []
        self.updating = True


        self.obsAccumulation = []
        enough = False
        good = False
        for i in range(self.trials):
            while not rospy.is_shutdown():
                rospy.sleep(0.01)
                try:
                    rospy.loginfo('try to update object pose')
                    self.tp = grasping_lib.getGraspFrame(self.listener, '/' + 'shelf_' + self._bin, '/' + self._item)
                    # self.tp = self.listener.lookupTransform('/base_link', '/' + self._item, rospy.Time(0))
                    rospy.loginfo('object pose UPDATED')
                    self.obsAccumulation.append(self.tp[0])
                    if len(self.obsAccumulation) == self.obsN:
                        enough = True
                        break
                except:
                    continue
            if enough:
                break

        rospy.loginfo('validation')
        good = self.validate()

        if not enough or not good:
            rospy.loginfo('not enough or not good')
            self.set_status('FAILURE')


        else:
            rospy.loginfo('enough and good')
            self.set_status('SUCCESS')


        self.updating = False
        self.lock.release()



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


if __name__ == '__main__':
    pubDetector = superDetector('amazon_detector')
    pubDetector.my_pub()
    rospy.spin()
