#!/usr/bin/python

import moveit_commander
import rospy
import sys
import tf
import pr2_moveit_utils.pr2_moveit_utils as pr2_moveit_utils
import math
import random
import PyKDL as kdl
import numpy as np



'''
This is testing util for top grasping by left_arm, the pose is expressed only in the /bask_link frame
left_arm_group: moveit group for left_arm
torso_group: moveit group for torso
row: which row of the shelf is to be touched
reach_in: distance the gripper wants to reach in
'''
def touchRowBottom(torso_group, left_arm_group, row, ft = False):

    base_frame_id = "/base_link"
    listener = tf.TransformListener()


    while not rospy.is_shutdown():
        try:
            torso_joint_pos_dict = rospy.get_param('/torso_joint_pos_dict')
            tool_size = rospy.get_param('/tool_size', [0.16, 0.02, 0.04])
            touch_shelf_test_dict = rospy.get_param('/touch_shelf_test_dict')
            break
        except:
            print 'here'
            rospy.sleep(random.uniform(0,1))
            continue


    reach_in = touch_shelf_test_dict['reach_in']
    pitch = touch_shelf_test_dict['pitch']
    roll = touch_shelf_test_dict['roll']
    preHeight = touch_shelf_test_dict['preHeight']
    preDistance = touch_shelf_test_dict['preDistance']
    trials = touch_shelf_test_dict['trials']
    bottomHeight = touch_shelf_test_dict['row_height'][row]

    if row == 'row_4':
        bin = "shelf_bin_J"
    elif row == 'row_3':
        bin = "shelf_bin_G"
    elif row == 'row_2':
        bin = 'shelf_bin_D'
    else:
        rospy.logerr('row number not recognized')
        return

        
    while not rospy.is_shutdown():
        try:
            binFrame = listener.lookupTransform('/base_link', "/" + bin, rospy.Time(0))
            break
        except:
            pass


    try:
        torso_group.set_joint_value_target(torso_joint_pos_dict['pregrasp'][row])
        torso_group.go()
    except:
        rospy.logerr('can not move torso to detecting height')
        return

    tool_frame_rotation = kdl.Rotation.RPY(math.radians(roll), math.radians(pitch), 0)
    
    '''
    pregrasp
    '''
    init_touching_pose = kdl.Frame(tool_frame_rotation, kdl.Vector(binFrame[0][0] + preDistance, binFrame[0][1] + 0.14, binFrame[0][2] + preHeight))
    try:
        pr2_moveit_utils.go_tool_frame(left_arm_group, init_touching_pose, base_frame_id = base_frame_id, ft = ft,
                                                   wait=True, tool_x_offset = tool_size[0])
    except:
        rospy.logerr('failed to init top grasping pose')
        return

    '''
    reaching
    '''

    reaching_pose = kdl.Frame(tool_frame_rotation, kdl.Vector(binFrame[0][0] + reach_in, binFrame[0][1] + 0.14, binFrame[0][2] + preHeight))
    try:
        pr2_moveit_utils.go_tool_frame(left_arm_group, reaching_pose, base_frame_id = base_frame_id, ft = ft,
                                                   wait=True, tool_x_offset = tool_size[0])
    except:
        rospy.logerr('failed to reach top grasping pose')
        return


    '''
    touching
    '''

    stepDistance = (binFrame[0][2] + preHeight - bottomHeight) / (trials - 1)
    for i in range(trials):
        height = binFrame[0][2] + preHeight - i * stepDistance
        touching_pose = kdl.Frame(tool_frame_rotation, kdl.Vector(binFrame[0][0] + reach_in, binFrame[0][1] + 0.14, height))

        try:
            pr2_moveit_utils.go_tool_frame(left_arm_group, touching_pose, base_frame_id = base_frame_id, ft = ft,
                                                       wait=True, tool_x_offset = tool_size[0])
        except:
            rospy.logerr('best touch height found is (heigher than expected): %4f' % height)
            return

    rospy.logerr('best touch height found is: %4f' % height)
    return

if __name__ == '__main__':
    rospy.init_node('touching_test')
    row = 'row_2'
    left_arm = moveit_commander.MoveGroupCommander('left_arm')
    torso = moveit_commander.MoveGroupCommander('torso')

    while not rospy.is_shutdown():
        try:
            left_arm_joint_pos_dict = rospy.get_param('/left_arm_joint_pos_dict')
            break
        except:
            rospy.sleep(random.uniform(0,1))
            continue

    start_pose = left_arm_joint_pos_dict['start']
    left_arm.set_joint_value_target(start_pose)
    left_arm.go(wait=True)

    prePose = left_arm_joint_pos_dict['pregrasp'][row]
    left_arm.set_joint_value_target(prePose)
    left_arm.go(wait=True)

    touch_shelf_test_dict = rospy.get_param('/touch_shelf_test_dict')
    touchRowBottom(torso, left_arm, row, ft = touch_shelf_test_dict['ft'])
    rospy.spin()