#!/usr/bin/env python


import rospy
import moveit_commander
import tf
from tf_conversions import posemath
import PyKDL as kdl
import sys
from geometry_msgs.msg import Pose
import numpy as np
import tf2_geometry_msgs
import geometry_msgs.msg
import copy
import pr2_moveit_utils.pr2_moveit_utils as pr2_moveit_utils

def tf2msg(tp, curr_pose):
    # print curr_pose.pose.position.x
    # print tp[0][0]
    return kdl.Vector( tp[0][0] - 0.1, tp[0][1], tp[0][2])
    # pose_target = geometry_msgs.msg.Pose()
    # # pose_target = copy.deepcopy(curr_pose.pose)
    # pose_target.position.x = tp[0][0]
    # pose_target.position.y = tp[0][1] 
    # pose_target.position.z = tp[0][2] 
    # return pose_target



if __name__=='__main__':

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('follow_object')
    left_arm = moveit_commander.MoveGroupCommander('left_arm')
    curr_pose = left_arm.get_current_pose()
    listener = tf.TransformListener()

    r = rospy.Rate(0.6)

    rospy.sleep(1.0)



    while not rospy.is_shutdown():

        try:
            T_b_groovy_tf = listener.lookupTransform('/base_link', '/amazon_cheeze', rospy.Time(0))

            print "got new pose"
        except:
            continue
        curr_pose = left_arm.get_current_pose()
        
        target_pose = kdl.Frame(kdl.Rotation.RPY(0,0,0), tf2msg(T_b_groovy_tf,curr_pose))

        try:
            print 'going to tracked position'
            pr2_moveit_utils.go_tool_frame(left_arm, target_pose, base_frame_id ='base_link', ft=False, wait=True)
            print 'position reached'
        except:
            print "goal position not reachable"
            print ""
            print ''
            continue

        # left_arm.set_pose_target(tf2msg(T_b_groovy_tf,curr_pose))
        # left_arm.go()

        # r.sleep()



