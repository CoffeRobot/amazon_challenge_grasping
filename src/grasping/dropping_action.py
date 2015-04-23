#!/usr/bin/env python

import moveit_commander
import rospy
import actionlib
import amazon_challenge_bt_actions.msg
from std_msgs.msg import String
import sys
import tf
import PyKDL as kdl
import pr2_moveit_utils.pr2_moveit_utils as pr2_moveit_utils
from pr2_controllers_msgs.msg import Pr2GripperCommandActionGoal

class ArmPositionAction(object):

    # create messages that are used to publish feedback/result
    _feedback = amazon_challenge_bt_actions.msg.BTFeedback()
    _result   = amazon_challenge_bt_actions.msg.BTResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, amazon_challenge_bt_actions.msg.BTAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        self.pub_dropped = rospy.Publisher('object_dropped', String, queue_size=10)
        self.pub_rate = rospy.Rate(30)
        self._pr2 = MyPR2()
        self._pr2.load("dropping_dictionary.pr2")
        rospy.Subscriber("/amazon_next_task", String, self.get_task)

    def get_task(self, msg):
        text = msg.data
        text = text.replace('[','')
        text = text.replace(']','')
        words = text.split(',')
        self._bin = words[0]
        self._item = words[1]

    def get_arm_to_move(self):
        if self._bin == 'bin_A' or self._bin == 'bin_D' or self._bin == 'bin_G' or self._bin == 'bin_H' or self._bin == 'bin_J':
            return 'left_arm'
        else:
            return 'right_arm'

    def execute_cb(self, goal):
        # publish info to the console for the user
        rospy.loginfo('[arm_position_server]: started action')
        # start executing the action

        arm_to_move = self.get_arm_to_move()
        if arm_to_move == 'left_arm':
        	pr2.go_left_arm('drop_wp_left_bin_')
        	pr2.go_left_arm('drop_wp_box')
        	pr2.go_left_gripper(10,10)
        	rospy.sleep(10)
    	else:
    		pr2.go_right_arm('drop_wp_left')
        	pr2.go_right_arm('drop_wp_box')
        	pr2.go_right_gripper(10,10)
        	rospy.sleep(10)

        
        self.set_status('SUCCESS')
        return

    
    def set_status(self,status):
        if status == 'SUCCESS':
            self.pub_posed.publish("SUCCESS")
            rospy.sleep(1)
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

    def normalize_angles(self, q):
        '''
        normalize angles to -pi, pi
        '''
        q_normalized = np.mod(q, 2*np.pi)

        for i in xrange(np.size(q)):
            if q_normalized[i] > np.pi:
                q_normalized[i] = -(2*np.pi - q_normalized[i])

        return q_normalized


if __name__ == "__main__":

    rospy.init_node('drop_object')
    ArmPositionAction(rospy.get_name())
    rospy.spin()