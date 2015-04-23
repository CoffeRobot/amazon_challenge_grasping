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
from geometry_msgs.msg import Pose, PoseStamped
from tf_conversions import posemath

class BTAction(object):
    # create messages that are used to publish feedback/result
    _feedback = amazon_challenge_bt_actions.msg.BTFeedback()
    _result = amazon_challenge_bt_actions.msg.BTResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, amazon_challenge_bt_actions.msg.BTAction,
                                                execute_cb=self.execute_cb, auto_start=False)
        self._as.start()
        self.pub_grasped = rospy.Publisher('object_grasped', String)
        self.pub_pose = rospy.Publisher('hand_pose', PoseStamped)
        self.pub_rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            try:
                self.left_arm = moveit_commander.MoveGroupCommander('left_arm')
                self.right_arm = moveit_commander.MoveGroupCommander('right_arm')
                break
            except:
                pass

        self.listener = tf.TransformListener()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.Subscriber("/amazon_next_task", String, self.get_task)
        self._item = ""
        self._bin = ""
        self.l_gripper_pub = rospy.Publisher('/l_gripper_controller/gripper_action/goal', Pr2GripperCommandActionGoal)
        self.r_gripper_pub = rospy.Publisher('/r_gripper_controller/gripper_action/goal', Pr2GripperCommandActionGoal)
        self.pre_distance = -0.1
        self.ft_switch = True
        self.lifting_height = 0.04

    def flush(self):
        self._item = ""
        self._bin = ""

    def transformPoseToRobotFrame(self, planPose, planner_frame):

        pre_pose_stamped = PoseStamped()
        pre_pose_stamped.pose = posemath.toMsg(planPose)
        pre_pose_stamped.header.stamp = rospy.Time()
        pre_pose_stamped.header.frame_id = planner_frame

        while not rospy.is_shutdown():
            try:
                robotPose = self.listener.transformPose('/base_link', pre_pose_stamped)
                break
            except:
                pass

        self.pub_pose.publish(robotPose)
        return robotPose





    def execute_cb(self, goal):
        # publish info to the console for the user
        rospy.loginfo('Starting Grasping')

        # start executing the action
        # check that preempt has not been requested by the client
        if self._as.is_preempt_requested():
            rospy.loginfo('Action Halted')
            self._as.set_preempted()
            return

        rospy.loginfo('Executing Grasping')


        while not rospy.is_shutdown():
            try:
                tp = self.listener.lookupTransform('/base_link', "/" + self._item + "_detector", rospy.Time(0))
                rospy.loginfo('got new object pose')
                break
            except:
                pass


        '''
        PRE-GRASPING
        '''
        planner_frame = '/' + self._item + "_detector"
        arm_now = self.get_arm_to_move()


        if arm_now == 'right_arm':
            self.open_right_gripper()
        else:
            self.open_left_gripper()

        pre_pose = kdl.Frame(kdl.Rotation.RPY(0, 0, 0), kdl.Vector( self.pre_distance, 0, 0))
        pre_pose_robot = self.transformPoseToRobotFrame(pre_pose, planner_frame)




        if arm_now == 'right_arm':
            try:
                pr2_moveit_utils.go_tool_frame(self.right_arm, pre_pose_robot.pose, base_frame_id = pre_pose_robot.header.frame_id, ft=self.ft_switch,
                                               wait=True)
            except:
                self.set_status('FAILURE')
                return
        else:
            try:
                pr2_moveit_utils.go_tool_frame(self.left_arm, pre_pose_robot.pose, base_frame_id = pre_pose_robot.frame_id, ft=self.ft_switch,
                                               wait=True)
            except:
                self.set_status('FAILURE')
                return

        '''
        REACHING
        '''
        reaching_pose = kdl.Frame(kdl.Rotation.RPY(0, 0, 0), kdl.Vector( 0,0,0))

        if arm_now == 'right_arm':
            try:
                pr2_moveit_utils.go_tool_frame(self.right_arm, reaching_pose, base_frame_id = planner_frame, ft=self.ft_switch,
                                               wait=True)
            except:
                self.flush()
                self.set_status('FAILURE')
                return
        else:
            try:
                pr2_moveit_utils.go_tool_frame(self.left_arm, reaching_pose, base_frame_id = planner_frame, ft=self.ft_switch,
                                               wait=True)
            except:
                self.flush()
                self.set_status('FAILURE')
                return

        '''
        GRASPING
        '''
        if arm_now == 'right_arm':
            self.close_right_gripper()
        else:
            self.close_left_gripper()

        '''
        LIFTING
        '''

        lifting_pose = kdl.Frame(tp[1], kdl.Vector( tp[0][0], tp[0][1], tp[0][2]) + self.lifting_height)

        if arm_now == 'right_arm':
            try:
                pr2_moveit_utils.go_tool_frame(self.right_arm, lifting_pose, base_frame_id = 'base_link', ft=self.ft_switch,
                                               wait=True)
            except:
                self.flush()
                self.set_status('FAILURE')
                return
        else:
            try:
                pr2_moveit_utils.go_tool_frame(self.left_arm, lifting_pose, base_frame_id = 'base_link', ft=self.ft_switch,
                                               wait=True)
            except:
                self.flush()
                self.set_status('FAILURE')
                return


        '''
        RETREATING
        '''
        retreating_pose = kdl.Frame(kdl.Rotation.RPY(0, 0, 0), kdl.Vector( -self.pre_distance , 0 ,0))

        if arm_now == 'right_arm':
            try:
                pr2_moveit_utils.go_tool_frame(self.right_arm, retreating_pose, base_frame_id = planner_frame, ft=self.ft_switch,
                                               wait=True)
            except:
                self.flush()
                self.set_status('FAILURE')
                return
        else:
            try:
                pr2_moveit_utils.go_tool_frame(self.left_arm, retreating_pose, base_frame_id = planner_frame, ft=self.ft_switch,
                                               wait=True)
            except:
                self.flush()
                self.set_status('FAILURE')
                return


        #IF THE ACTION HAS SUCCEEDED
        self.flush()



        self.pub_grasped.publish("SUCCESS")
        self.set_status('SUCCESS')
        self.pub_rate.sleep()
        return


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

    def go_left_gripper(self, position, max_effort):
        """Move left gripper to position with max_effort
        """
        ope = Pr2GripperCommandActionGoal()
        ope.goal.command.position = position
        ope.goal.command.max_effort = max_effort
        self.l_gripper_pub.publish(ope)

    def go_right_gripper(self, position, max_effort):
        """Move right gripper to position with max_effort
        """
        ope = Pr2GripperCommandActionGoal()
        ope.goal.command.position = position
        ope.goal.command.max_effort = max_effort
        self.r_gripper_pub.publish(ope)

    def close_left_gripper(self):
        self.go_left_gripper(0, 40)
        rospy.sleep(4)

    def close_right_gripper(self):
        self.go_right_gripper(0, 40)
        rospy.sleep(4)

    def open_left_gripper(self):
        self.go_left_gripper(10, 40)
        rospy.sleep(2)

    def open_right_gripper(self):
        self.go_right_gripper(10, 40)
        rospy.sleep(2)




if __name__ == '__main__':
    rospy.init_node('grasp_object')
    BTAction(rospy.get_name())
    rospy.spin()
