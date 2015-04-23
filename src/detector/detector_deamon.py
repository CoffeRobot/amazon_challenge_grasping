#!/usr/bin/env python
import moveit_commander

import rospy

import actionlib

import amazon_challenge_bt_actions.msg

from std_msgs.msg import String
import sys

import tf


class BTAction(object):
    # create messages that are used to publish feedback/result
    _feedback = amazon_challenge_bt_actions.msg.BTFeedback()
    _result = amazon_challenge_bt_actions.msg.BTResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, amazon_challenge_bt_actions.msg.BTAction,
                                                execute_cb=self.execute_cb, auto_start=False)
        self._as.start()
        self.pub_rate = rospy.Rate(30)
        rospy.Subscriber("/pub_detected", String, self.callback)





    def execute_cb(self, goal):
        # publish info to the console for the user
        rospy.loginfo('Starting Grasping')
        self.flush()

        curr_t = rospy.Time.from_sec(time.time())

        success = False
        for i in range(self.trials):
            if self._as.is_preempt_requested():
                rospy.loginfo('Action Halted')
                self._as.set_preempted()
                success = False
                break
            try:
                tp = self.listener.lookupTransform('/base_link', '/' + self._item, rospy.Time(0))
                rospy.loginfo('got new object pose')
                set_status("SUCCESS")
                success = True
                break
            except:
                continue

        if not success:
            set_status("FAILURE")




        # start executing the action
        while not rospy.is_shutdown():
            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('Action Halted')
                self._as.set_preempted()
                success = False
                break

            rospy.loginfo('Publishing Detector')

            self.br.sendTransform(tp, rospy.Time.now(),\
                                     "/" + self._item + "_detector",   \
                                     "/base_link")




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

    def callback(self):
        set_status("SUCCESS")


if __name__ == '__main__':
    rospy.init_node('detector_node')
    BTAction(rospy.get_name())
    rospy.spin()
