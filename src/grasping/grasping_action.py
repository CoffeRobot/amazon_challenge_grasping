#!/usr/bin/python
import moveit_commander

import rospy

import actionlib

import amazon_challenge_bt_actions.msg

from std_msgs.msg import String
import sys

import tf
import PyKDL as kdl
import pr2_moveit_utils.pr2_moveit_utils as pr2_moveit_utils
from pr2_controllers_msgs.msg import Pr2GripperCommand, JointControllerState
from geometry_msgs.msg import Pose, PoseStamped
from tf_conversions import posemath
import math
from calibrateBase import baseMove
from amazon_challenge_motion.bt_motion import BTMotion
import amazon_challenge_bt_actions.msg
from grasping.generate_object_dict import *
import random
from simtrack_nodes.srv import SwitchObjects


class BTAction(object):
    # create messages that are used to publish feedback/result
    _feedback = amazon_challenge_bt_actions.msg.BTFeedback()
    _result = amazon_challenge_bt_actions.msg.BTResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, amazon_challenge_bt_actions.msg.BTAction,
                                                execute_cb=self.execute_cb, auto_start=False)
        self.pub_grasped = rospy.Publisher('object_grasped', String)
        self.pub_pose = rospy.Publisher('hand_pose', PoseStamped)
        self.pub_rate = rospy.Rate(30)
        self.poseFromSimtrack = True
        self.xLength = 0



        moveit_commander.roscpp_initialize(sys.argv)
        rospy.Subscriber("/amazon_next_task", String, self.get_task)
        self._item = ""
        self._bin = ""
        self.l_gripper_pub = rospy.Publisher('/l_gripper_controller/command', Pr2GripperCommand)
        self.r_gripper_pub = rospy.Publisher('/r_gripper_controller/command', Pr2GripperCommand)


        while not rospy.is_shutdown():
            try:
                self.grasping_param_dict = rospy.get_param('/grasping_param_dict')
                self.grasp_max_effort_dict = rospy.get_param('/grasp_max_effort_dict')
                break
            except:
                rospy.sleep(random.uniform(0,2))
                continue

        self.pre_distance = self.grasping_param_dict['pre_distance'] # should get from grasping_dict
        self.lifting_height = self.grasping_param_dict['lifting_height']
        self.topGraspHeight = self.grasping_param_dict['topGraspHeight']
        self.topGraspingFrame = self.grasping_param_dict['topGraspingFrame']
        self.sideGraspingTrialAngles = self.grasping_param_dict['sideGraspingTrialAngles']
        self.sideGraspingTolerance = self.grasping_param_dict['sideGraspingTolerance']
        self.base_retreat_distance = self.grasping_param_dict['base_retreat_distance']
        self.topGraspingTrials = self.grasping_param_dict['topGraspingTrials']
        self.sideGraspingTrials = self.grasping_param_dict['sideGraspingTrials']
        self.gripperWidth = self.grasping_param_dict['gripperWidth']
        self.topGraspingPitch = self.grasping_param_dict['topGraspingPitch']
        self.topGraspingRoll = self.grasping_param_dict['topGraspingRoll']
        self.topGraspingPitchTolerance = self.grasping_param_dict['topGraspingPitchTolerance']
        self.topGraspingPitchTrials = self.grasping_param_dict['topGraspingPitchTrials']
        self.topGraspingShakingNumber = self.grasping_param_dict['topGraspingShakingNumber']
        self.topGraspingReachSeg = self.grasping_param_dict['topGraspingReachSeg']
        self.topGraspingYawTolerance = self.grasping_param_dict['topGraspingYawTolerance']
        self.topGraspingYawTrials = self.grasping_param_dict['topGraspingYawTrials']
        self.sideGraspingSegReach = self.grasping_param_dict['sideGraspingSegReach']
        self.topGraspingTouchSteps = self.grasping_param_dict['topGraspingTouchSteps']
        self.topGraspingTouchTolerance = self.grasping_param_dict['topGraspingTouchTolerance']
        self.topGraspingYshiftTolerance = self.grasping_param_dict['topGraspingYshiftTolerance']
        self.topGraspingLiftingHeight = self.grasping_param_dict['topGraspingLiftingHeight']
        self.topGraspingMaxReachingHeight = self.grasping_param_dict['topGraspingMaxReachingHeight']
        self.dictObj = objDict()
        self.objSpec = {}
        self.topGrasping_pre_distance = self.grasping_param_dict['topGrasping_pre_distance']

        # get ft_switch from the contest mode
        while not rospy.is_shutdown():
            try:
                mode = rospy.get_param('/contest')
                if mode:
                    self.ft_switch = False
                else:
                    self.ft_switch = True
                break
            except:
                rospy.sleep(random.uniform(0,1))
                pass

        # get base_move parameters
        while not rospy.is_shutdown():
            try:
                self._base_move_params = rospy.get_param('/base_move')
                self.base_pos_dict = rospy.get_param('/base_pos_dict')
                self.left_arm_joint_pos_dict = rospy.get_param('/left_arm_joint_pos_dict')
                self.right_arm_joint_pos_dict = rospy.get_param('/right_arm_joint_pos_dict')
                self.grasp_check_dict = rospy.get_param('/grasp_check_dict')
                self._timeout = rospy.get_param(rospy.get_name() + '/timeout')
                break
            except:
                rospy.sleep(random.uniform(0,1))
                continue


        while not rospy.is_shutdown():
            try:
                self._tool_size = rospy.get_param('/tool_size', [0.16, 0.02, 0.04])
                break
            except:
                rospy.sleep(random.uniform(0,1))
                continue


        while not rospy.is_shutdown():
            try:
                self.listener = tf.TransformListener()
                self.robot = moveit_commander.RobotCommander()
                self.left_arm = self.robot.get_group('left_arm')
                self.right_arm = self.robot.get_group('right_arm')
                self._arms = self.robot.get_group('arms')
                self._bm = baseMove.baseMove(verbose=False)
                self._bm.setPosTolerance(self._base_move_params['pos_tolerance'])
                self._bm.setAngTolerance(self._base_move_params['ang_tolerance'])
                self._bm.setLinearGain(self._base_move_params['linear_gain'])
                self._bm.setAngularGain(self._base_move_params['angular_gain'])
                break
            except:
                rospy.sleep(random.uniform(0,2))
                pass

        self._as.start()
        rospy.loginfo('Grasping action ready')

    def flush(self):
        self._item = ""
        self._bin = ""
        self.objSpec = {}

    def transformPoseToRobotFrame(self, planPose, planner_frame):

        pre_pose_stamped = PoseStamped()
        pre_pose_stamped.pose = posemath.toMsg(planPose)
        pre_pose_stamped.header.stamp = rospy.Time()
        pre_pose_stamped.header.frame_id = planner_frame

        r = rospy.Rate(1.0)
        while not rospy.is_shutdown() and not self._exit:

            try:
                robotPose = self.listener.transformPose('/base_link', pre_pose_stamped)
                break
            except:
                r.sleep()
                pass

        self.pub_pose.publish(robotPose)
        return robotPose



    def RPYFromQuaternion(self, q):
        return tf.transformations.euler_from_quaternion([q[0], q[1], q[2], q[3]])


    def timer_callback(self, event):
        rospy.logerr('[' + rospy.get_name() + ']: TIMED OUT!')

        # pull the base back 60 cm

        self.left_arm.stop()
        self.right_arm.stop()

        base_pos_goal = [-1.42, self._bm.trans[1], self._bm.trans[2], 0.0, 0.0, 0.0]

        self._bm.goAngle(base_pos_goal[5])
        self._bm.goPosition(base_pos_goal[0:2])
        self._bm.goAngle(base_pos_goal[5])

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
            self.set_status("FAILURE")
            return True

        return False

    def shutdown_simtrack(self):
        # get simtrack switch objects service
        while not rospy.is_shutdown():
            try:
                rospy.wait_for_service('/simtrack/switch_objects', 10.0)
                break
            except:
                rospy.loginfo('[' + rospy.get_name() + ']: waiting for simtrack switch object service')
                continue

        simtrack_switch_objects_srv = rospy.ServiceProxy('/simtrack/switch_objects', SwitchObjects)

        simtrack_switch_objects_srv.call()

    def execute_cb(self, goal):
        #self.shutdown_simtrack()
        rospy.sleep(1.0)
        self._exit = False
        self.timer = rospy.Timer(rospy.Duration(self._timeout), self.timer_callback, oneshot=True)

        # publish info to the console for the user
        rospy.loginfo('Starting Grasping')
        try:
            self.objSpec = self.dictObj.getEntry(self._item)
        except Exception, e:
            rospy.logerr('Cannot access object spec from grasp dict')
            self.set_status('FAILURE')
            self.timer.shutdown()
            return

        self.pre_distance = self.objSpec.pre_distance # this is decided upon per object
        self.xLength = self.objSpec.xLength
        # start executing the action
        # check that preempt has not been requested by the client
        if self._as.is_preempt_requested():
            rospy.loginfo('Action Halted')
            self._as.set_preempted()
            self.timer.shutdown()
            return

        rospy.loginfo('Executing Grasping')
        status = False
        for gs in self.objSpec.graspStrategy:
            if gs == 0:
                rospy.loginfo("sideGrasping is chosen")
                for i in range(self.sideGraspingTrials):
                    status = self.sideGrasping()
                    if status:
                        break
                    if self.execute_exit():
                        return
                if status:
                    break
            elif gs == 1:
                rospy.loginfo("topGrasping is chosen")
                for i in range(self.topGraspingTrials):
                    status = self.topGrasping()
                    if status:
                        break
                    if self.execute_exit():
                        return
                if status:
                    break
            else:
                self.flush()
                rospy.logerr('No strategy found to grasp')
                self.set_status('FAILURE')

        if status:
            self.set_status('SUCCESS')
        else:
            self.set_status('FAILURE')
        self.timer.shutdown()
        return


    def topGrasping(self):

        r = rospy.Rate(1.0)
        while not rospy.is_shutdown():

            if self._exit:
                return False
            try:
                tp = self.listener.lookupTransform('/base_link', "/" + self._item + "_detector", rospy.Time(0))
                binFrame = self.listener.lookupTransform("/" + "shelf_" + self._bin, "/" + self._item + "_detector", rospy.Time(0))
                liftShift = 0.15 - binFrame[0][1]
                rospy.loginfo('got new object pose')
                tpRPY = self.RPYFromQuaternion(tp[1])
                self.poseFromSimtrack = True
                break
            except:
                try:
                    tp = self.listener.lookupTransform('/base_link', "/" + self._item + "_detector_seg", rospy.Time(0))
                    binFrame = self.listener.lookupTransform("/" + "shelf_" + self._bin, "/" + self._item + "_detector_seg", rospy.Time(0))
                    liftShift = 0.15 - binFrame[0][1]
                    rospy.loginfo('got new object pose')
                    tpRPY = self.RPYFromQuaternion(tp[1])
                    self.poseFromSimtrack = False
                    break
                except:
                    r.sleep()
                    continue
        self.open_left_gripper()

        if self._exit:
            return False

        for i in range(self.topGraspingPitchTrials):

            if self._exit:
                return False

            tgp = self.topGraspingPitch - self.topGraspingPitchTolerance / (self.topGraspingPitchTrials - 1) * i
            rospy.loginfo('topGraspingPitch now: %4f' % tgp)

            for j in range(self.topGraspingYawTrials):
                if self._exit:
                    return False
                y_shift_step = 0.15 / (self.topGraspingYawTrials - 1.0)
                reach_Y_shift_step = self.topGraspingYshiftTolerance / (self.topGraspingYawTrials - 1.0)
                
                if binFrame[0][1] >= 0.15:
                    tgy = self.topGraspingYawTolerance / (self.topGraspingYawTrials - 1) * j
                    y_shift_now = - y_shift_step * j
                    reach_Y_shift = - reach_Y_shift_step * j
                else:
                    tgy = -self.topGraspingYawTolerance / (self.topGraspingYawTrials - 1) * j
                    y_shift_now = y_shift_step * j
                    reach_Y_shift = reach_Y_shift_step * j


                rospy.loginfo('topGraspingYaw now: %4f' % tgy)
                tool_frame_rotation = kdl.Rotation.RPY(math.radians(self.topGraspingRoll), math.radians(tgp), math.radians(tgy))
                reachingHeight = max(self.topGraspingMaxReachingHeight, tp[0][2] + self.topGraspHeight)
                '''
                PRE-GRASPING
                '''

                pre_pose = kdl.Frame(tool_frame_rotation, kdl.Vector( tp[0][0] + self.topGrasping_pre_distance, tp[0][1] + y_shift_now, reachingHeight))


                try:
                    pr2_moveit_utils.go_tool_frame(self.left_arm, pre_pose, base_frame_id = self.topGraspingFrame, ft=self.ft_switch,
                                                   wait=True, tool_x_offset=self._tool_size[0])
                except:
                    rospy.logerr('exception in PRE-GRASPING')
                    continue


                '''
                REACHING
                '''
                if self.poseFromSimtrack:
                    reaching_pose = kdl.Frame(tool_frame_rotation, kdl.Vector( tp[0][0], tp[0][1] + reach_Y_shift, tp[0][2] + self.topGraspHeight))
                else:
                    reaching_pose = kdl.Frame(tool_frame_rotation, kdl.Vector( tp[0][0] + self.topGraspingReachSeg, tp[0][1] + reach_Y_shift, tp[0][2] + reachingHeight))
               
                try:
                    pr2_moveit_utils.go_tool_frame(self.left_arm, reaching_pose, base_frame_id = self.topGraspingFrame, ft=self.ft_switch,
                                                   wait=True, tool_x_offset=self._tool_size[0])
                except:
                    rospy.logerr('exception in REACHING')
                    continue

                '''
                TOUCHING
                '''

                row_height = self.grasping_param_dict['row_height'][self.get_row()]
                touching_height = max(tp[0][2], row_height)
                if self.poseFromSimtrack:
                    touching_pose = kdl.Frame(tool_frame_rotation, kdl.Vector( tp[0][0], tp[0][1] + reach_Y_shift, touching_height))
                else:
                    touching_pose = kdl.Frame(tool_frame_rotation, kdl.Vector( tp[0][0] + self.topGraspingReachSeg, tp[0][1] + reach_Y_shift, touching_height))
                rospy.loginfo("touching_height: %4f" % touching_height)
                
                try:
                    pr2_moveit_utils.go_tool_frame(self.left_arm, touching_pose, base_frame_id = self.topGraspingFrame, ft=self.ft_switch,
                                                   wait=True, tool_x_offset=self._tool_size[0])
                except:
                    if not self.iterative_touch(touching_pose):
                        rospy.logerr('exception in TOUCHING')
                        continue
                    else:
                        pass

                '''
                SMART-SHAKING
                '''

                if self.poseFromSimtrack:
                    shaking_pose1 = kdl.Frame(tool_frame_rotation, kdl.Vector( tp[0][0], tp[0][1] + 0.01, touching_height))
                    shaking_pose2 = kdl.Frame(tool_frame_rotation, kdl.Vector( tp[0][0], tp[0][1] - 0.01, touching_height))
                else:
                    shaking_pose1 = kdl.Frame(tool_frame_rotation, kdl.Vector( tp[0][0] + self.topGraspingReachSeg, tp[0][1] + reach_Y_shift + 0.01, touching_height))
                    shaking_pose2 = kdl.Frame(tool_frame_rotation, kdl.Vector( tp[0][0] + self.topGraspingReachSeg, tp[0][1] + reach_Y_shift - 0.01, touching_height))
                
                for i in range(self.topGraspingShakingNumber):
                    if self._exit:
                        return False
                    try:
                        pr2_moveit_utils.go_tool_frame(self.left_arm, shaking_pose1, base_frame_id = self.topGraspingFrame, ft=self.ft_switch,
                                                       wait=True, tool_x_offset=self._tool_size[0])
                    except:
                        rospy.logerr('exception in SMART-SHAKING 1, never mind')

                    try:
                        pr2_moveit_utils.go_tool_frame(self.left_arm, shaking_pose2, base_frame_id = self.topGraspingFrame, ft=self.ft_switch,
                                                       wait=True, tool_x_offset=self._tool_size[0])
                    except:
                        rospy.logerr('exception in SMART-SHAKING 2, never mind')


                '''
                GRASPING
                '''

                self.close_left_gripper()

                '''
                LIFTING
                '''

                lifting_pose = kdl.Frame(tool_frame_rotation, kdl.Vector( tp[0][0], tp[0][1] + liftShift, tp[0][2] + self.topGraspingLiftingHeight))
                


                try:
                    pr2_moveit_utils.go_tool_frame(self.left_arm, lifting_pose, base_frame_id = self.topGraspingFrame, ft=self.ft_switch,
                                                   wait=True, tool_x_offset=self._tool_size[0])
                except:
                    self.open_left_gripper()
                    rospy.logerr('exception in LIFTING')
                    continue

                '''
                RETREATING
                '''
                rospy.loginfo('RETREATING')



                try:
                    column = self.get_column()
                    base_pos_goal = self.base_pos_dict[column]
                    base_pos_goal[0] -= abs(self.base_retreat_distance)
                    self.go_base_pos_async(base_pos_goal)
                except Exception, e:
                    self.open_left_gripper()

                    rospy.logerr('exception in RETREATING')
                    continue
                return True



        return False

    def sideGrasping(self):

        r = rospy.Rate(1.0)
        while not rospy.is_shutdown():

            if self._exit:
                return False
            try:
                tp = self.listener.lookupTransform('/base_link', "/" + self._item + "_detector", rospy.Time(0))
                binFrame = self.listener.lookupTransform("/" + "shelf_" + self._bin, "/" + self._item + "_detector", rospy.Time(0))
                liftShift = 0.15 - binFrame[0][1]
                rospy.loginfo('got new object pose')
                tpRPY = self.RPYFromQuaternion(tp[1])
                objBinRPY = self.RPYFromQuaternion(binFrame[1])
                planner_frame = '/' + self._item + "_detector"
                self.poseFromSimtrack = True
                break
            except:
                try:
                    tp = self.listener.lookupTransform('/base_link', "/" + self._item + "_detector_seg", rospy.Time(0))
                    binFrame = self.listener.lookupTransform("/" + "shelf_" + self._bin, "/" + self._item + "_detector_seg", rospy.Time(0))
                    planner_frame = '/' + self._item + "_detector_seg"
                    liftShift = 0.15 - binFrame[0][1]
                    rospy.loginfo('got new object pose')
                    tpRPY = self.RPYFromQuaternion(tp[1])
                    objBinRPY = self.RPYFromQuaternion(binFrame[1])
                    self.poseFromSimtrack = False
                    break
                except:
                    r.sleep()
                    continue

        self.open_left_gripper()
        row_height = self.grasping_param_dict['row_height'][self.get_row()]
        
        if tp[0][2] - row_height < 0.02:
            return False

        if abs(objBinRPY[1]) > 0.5:
            rospy.logerr('require pushing the object')
            return False

        angle_step = 0

        if objBinRPY[2] < 0:
            angle_step = -self.sideGraspingTolerance / (self.sideGraspingTrialAngles - 1.0)
        else:
            angle_step = self.sideGraspingTolerance / (self.sideGraspingTrialAngles - 1.0)


        for i in range(self.sideGraspingTrialAngles):

            if self._exit:
                return False

            yaw_now = math.radians(angle_step * i)
            x_shift_now = (self.pre_distance + self.xLength) * math.cos(yaw_now)
            y_shift_now = (self.pre_distance + self.xLength) * math.sin(yaw_now)
            rospy.loginfo('yaw_now: %4f, y_shift_now: %4f' % (yaw_now, y_shift_now))
            '''
            PRE-GRASPING
            '''
            rospy.loginfo('PRE-GRASPING')


            rospy.logerr(yaw_now)
            pre_pose = kdl.Frame(kdl.Rotation.RPY(0, 0, yaw_now), kdl.Vector( x_shift_now, y_shift_now, 0))
            pre_pose_robot = self.transformPoseToRobotFrame(pre_pose, planner_frame)

            if self._exit:
                return False

            try:
                pr2_moveit_utils.go_tool_frame(self.left_arm, pre_pose_robot.pose, base_frame_id = pre_pose_robot.header.frame_id, ft=self.ft_switch,
                                               wait=True, tool_x_offset=self._tool_size[0])
            except Exception, e:
                rospy.logerr('exception in PRE-GRASPING')
                continue

            '''
            REACHING
            '''
            rospy.loginfo('REACHING')
            if self.poseFromSimtrack:
                reaching_pose = kdl.Frame(kdl.Rotation.RPY(0, 0, yaw_now), kdl.Vector( 0.0, 0.0, 0))
                reaching_pose_robot = self.transformPoseToRobotFrame(reaching_pose, planner_frame)
            else:
                reaching_pose = kdl.Frame(kdl.Rotation.RPY(0, 0, yaw_now), kdl.Vector( self.sideGraspingSegReach, 0.0, 0))
                reaching_pose_robot = self.transformPoseToRobotFrame(reaching_pose, planner_frame)

            if self._exit:
                return False

            try:
                pr2_moveit_utils.go_tool_frame(self.left_arm, reaching_pose_robot.pose, base_frame_id = reaching_pose_robot.header.frame_id, ft=self.ft_switch,
                                               wait=True, tool_x_offset=self._tool_size[0])
            except:
                rospy.logerr('exception in REACHING')
                continue

            '''
            GRASPING
            '''
            rospy.loginfo('GRASPING')
            self.close_left_gripper()

            '''
            CHECKING THE GRIPPER ANGLE POS
            '''
            rospy.loginfo('[grasp_object]: Checking gripper angle pos')
            # action has failed... return false
            if not self.check_gripper_angle():
                rospy.logerr('[grasp_object]: grasping action failed')
                self.open_left_gripper()
                continue


            '''
            LIFTING
            '''

            rospy.loginfo('LIFTING')

            lifting_pose = kdl.Frame(kdl.Rotation.RPY(tpRPY[0], tpRPY[1], 0), kdl.Vector( tp[0][0], tp[0][1] + liftShift, tp[0][2] + self.lifting_height))

        
            try:
                pr2_moveit_utils.go_tool_frame(self.left_arm, lifting_pose, base_frame_id = 'base_link', ft=self.ft_switch,
                                               wait=True, tool_x_offset=self._tool_size[0])
            except:
                self.open_left_gripper()
                rospy.logerr('exception in LIFTING')
                continue

            '''
            RETREATING
            '''
            rospy.loginfo('RETREATING')

            try:
                column = self.get_column()
                base_pos_goal = self.base_pos_dict[column]
                base_pos_goal[0] -= abs(self.base_retreat_distance)
                self.go_base_pos_async(base_pos_goal)
            except Exception, e:
                # rospy.logerr(e)
                self.flush()

                self.open_left_gripper()

                rospy.logerr('exception in RETREATING')
                continue

            rospy.loginfo('Grasping successful')
            self.flush()
            return True



        #IF THE ACTION HAS FAILED
        self.flush()
        return False



    def iterative_touch(self, touching_pose):
        '''
        Warning: this function now works for only left_arm
        '''
        step_size = self.topGraspingTouchTolerance / self.topGraspingTouchSteps

        for i in range(1,self.topGraspingTouchSteps + 1):
            if self._exit:
                return False
            touching_pose.p[2] += step_size
            try:
                pr2_moveit_utils.go_tool_frame(self.left_arm, touching_pose, base_frame_id = self.topGraspingFrame, ft=self.ft_switch,
                                                       wait=True, tool_x_offset=self._tool_size[0])
                return True
            except:
                pass
        return False



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
        self.flush()
        text = msg.data
        text = text.replace('[','')
        text = text.replace(']','')
        words = text.split(',')
        self._bin = words[0]
        self._item = words[1]


    def go_left_gripper(self, position, max_effort):
        """Move left gripper to position with max_effort
        """
        ope = Pr2GripperCommand()
        ope.position = position
        ope.max_effort = max_effort
        self.l_gripper_pub.publish(ope)

    def go_right_gripper(self, position, max_effort):
        """Move right gripper to position with max_effort
        """
        ope = Pr2GripperCommand()
        ope.position = position
        ope.max_effort = max_effort
        self.r_gripper_pub.publish(ope)

    def close_left_gripper(self):
        max_effort = self.grasp_max_effort_dict[self._item]
        self.go_left_gripper(0, 40)
        rospy.sleep(4)

    def close_right_gripper(self):
        max_effort = self.grasp_max_effort_dict[self._item]
        self.go_right_gripper(0, 40)
        rospy.sleep(4)

    def open_left_gripper(self):
        self.go_left_gripper(10, 40)
        rospy.sleep(2)

    def open_right_gripper(self):
        self.go_right_gripper(10, 40)
        rospy.sleep(2)

    def go_base_pos_async(self, base_pos_goal):

        angle = base_pos_goal[5]
        pos = base_pos_goal[0:2]
        r = rospy.Rate(20.0)

        # check for preemption while the base hasn't reach goal configuration
        while not self._bm.goAngle(angle) and not rospy.is_shutdown():

            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                #HERE THE CODE TO EXECUTE WHEN THE  BEHAVIOR TREE DOES HALT THE ACTION
                group.stop()
                rospy.loginfo('[pregrasp_server]: action halted while moving base')
                self._as.set_preempted()
                self._success = False
                return False

            #HERE THE CODE TO EXECUTE AS LONG AS THE BEHAVIOR TREE DOES NOT HALT THE ACTION
            r.sleep()

        while not self._bm.goPosition(pos) and not rospy.is_shutdown():

            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                #HERE THE CODE TO EXECUTE WHEN THE  BEHAVIOR TREE DOES HALT THE ACTION
                group.stop()
                rospy.loginfo('[pregrasp_server]: action halted while moving base')
                self._as.set_preempted()
                self._success = False
                return False

            #HERE THE CODE TO EXECUTE AS LONG AS THE BEHAVIOR TREE DOES NOT HALT THE ACTION
            r.sleep()

        while not self._bm.goAngle(angle) and not rospy.is_shutdown():

            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                #HERE THE CODE TO EXECUTE WHEN THE  BEHAVIOR TREE DOES HALT THE ACTION
                group.stop()
                rospy.loginfo('[pregrasp_server]: action halted while moving base')
                self._as.set_preempted()
                self._success = False
                return False

            #HERE THE CODE TO EXECUTE AS LONG AS THE BEHAVIOR TREE DOES NOT HALT THE ACTION
            r.sleep()

        return True

    def get_column(self):
        '''
        For setting the base pose
        '''
        while not rospy.is_shutdown():
            try:
                if self._bin=='bin_A' or self._bin=='bin_D' or self._bin=='bin_G' or self._bin=='bin_J':
                    return 'column_1'

                elif self._bin=='bin_B' or self._bin=='bin_E' or self._bin=='bin_H' or self._bin=='bin_K':
                    return 'column_2'

                elif self._bin=='bin_C' or self._bin=='bin_F' or self._bin=='bin_I' or self._bin=='bin_L':
                    return 'column_3'

            except:
                pass

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

    def gripper_controller_state_callback(self, msg):
        self._gripper_pos = msg.process_value
        self._gripper_vel = msg.process_value_dot
        self._got_gripper_controller_state = True

    def check_gripper_angle(self):
        '''
        Checks gripper angle to indicate success or failure
        '''
        # return true if we do not want to check the gripper angle pos
        if not self.grasp_check_dict[self._item]['check_gripper_angle']:
            rospy.logwarn('[grasp_action]: no checking of gripper angle pos for item ' + self._item + '')
            return True

        self._gripper_pos = 0.0
        self._gripper_vel = 10.0
        self._got_gripper_controller_state = False

        l_gripper_controller_state_sub = rospy.Subscriber('/l_gripper_controller/state', JointControllerState, self.gripper_controller_state_callback)
        rospy.sleep(1.0)

        r = rospy.Rate(1.0)
        t_init = rospy.Time.now()

        while not rospy.is_shutdown() and not self._got_gripper_controller_state and not abs(self._gripper_vel)<self.grasp_check_dict['gripper_vel_threshold']:

            # timeout
            if(rospy.Time.now()-t_init).to_sec()>self.grasp_check_dict['timeout']:
                rospy.logerr('[grasp_object]: check gripper angle timed out')
                return False

            r.sleep()

        l_gripper_controller_state_sub.unregister()

        if self._gripper_pos<self.grasp_check_dict[self._item]['angle_threshold']:
            rospy.logerr('[grasp_object]: object not grasped, check gripper angle failed')
            return False

        rospy.loginfo('[grasp_object]: check gripper angle succeeded!')
        return True



if __name__ == '__main__':
    rospy.init_node('grasp_object')
    BTAction(rospy.get_name())
    rospy.spin()
