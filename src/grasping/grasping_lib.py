#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import sys
import tf
import PyKDL as kdl
from tf_conversions import posemath
import numpy
from grasping.generate_object_dict import *
from grasping.myTypes import *

def getGraspingAxis(bin_frame, obj_frame, object_name, simtrackUsed):

    '''
    this function assumes everything is represented in the quaternions in the /base_link frame
    '''
    if object_name.endswith('_scan'):
        object_name = object_name[:-5]
    dictObj = objDict()
    objSpec = dictObj.getEntry(object_name)

    F_bin_frame = posemath.fromTf(bin_frame)
    F_obj_frame = posemath.fromTf(obj_frame)

    objRed = F_obj_frame.M * kdl.Vector(1.0, 0.0, 0.0)
    objGreen = F_obj_frame.M * kdl.Vector(0.0, 1.0, 0.0)
    objBlue = F_obj_frame.M * kdl.Vector(0.0, 0.0, 1.0)
    
    binRed = F_bin_frame.M * kdl.Vector(1.0, 0.0, 0.0)
    binGreen = F_bin_frame.M * kdl.Vector(0.0, 1.0, 0.0)
    binBlue = F_bin_frame.M * kdl.Vector(0.0, 0.0, 1.0)
    
    rRProj = kdl.dot(objRed , binRed)
    gRProj = kdl.dot(objGreen, binRed)
    bRProj = kdl.dot(objBlue, binRed)

    tmpApproach1 = [abs(rRProj), abs(gRProj), abs(bRProj)]

    if simtrackUsed:
        for i in range(3):
            if i in objSpec.invalidApproachAxis:
                tmpApproach1[i] = 0

    tmpApproach2 = [rRProj, gRProj, bRProj]
    axisApproach = tmpApproach1.index(max(tmpApproach1))


    objAxes = [objRed, objGreen, objBlue]
    tmpGrasp1 = []

    for i in range(3):
        if simtrackUsed:
            if i == axisApproach or i in objSpec.invalidGraspAxis:
                tmpGrasp1.append(0)
                continue
        tmpGrasp1.append(kdl.dot(objAxes[i], binBlue))

    tmpGrasp2 = [abs(t) for t in tmpGrasp1]

    axisGrasp = tmpGrasp2.index(max(tmpGrasp2))


    return axisApproach, tmpApproach2[axisApproach]/abs(tmpApproach2[axisApproach]), axisGrasp, tmpGrasp1[axisGrasp]/abs(tmpGrasp1[axisGrasp])




def getGraspFrame(listener, shelf_bin, object_name, simtrackUsed):

    got = False
    
    for i in range(10):
        try:
            bin_frame = listener.lookupTransform('/base_link', shelf_bin, rospy.Time(0))
            obj_frame = listener.lookupTransform('/base_link', object_name, rospy.Time(0))
            got = True
            break
        except:
            rospy.sleep(0.1)
            pass
    if not got:
        raise Exception('getGraspFrame failed')

    try:
        approchVec, approchDir, graspingVec, graspingDir = getGraspingAxis(bin_frame, obj_frame, object_name, simtrackUsed)
    except Exception, e:
        rospy.logerr(e)

    F_bin_frame = posemath.fromTf(bin_frame)
    F_obj_frame = posemath.fromTf(obj_frame)

    objRed = F_obj_frame.M * kdl.Vector(1.0, 0.0, 0.0)
    objGreen = F_obj_frame.M * kdl.Vector(0.0, 1.0, 0.0)
    objBlue = F_obj_frame.M * kdl.Vector(0.0, 0.0, 1.0)


    objAxes = [objRed, objGreen, objBlue]

    x = [a*approchDir for a in objAxes[approchVec]]
    z = [g*(-graspingDir) for g in objAxes[graspingVec]]

    y = numpy.cross(z, x)
    

    r = kdl.Rotation()
    for i in range(3):
        for j in range(3):
            if i == 0:
                r[j,i] = x[j]
            elif i == 1:
                r[j,i] = y[j]
            else:
                r[j,i] = z[j]

    t = F_obj_frame.p - F_bin_frame.p

    f = kdl.Frame(r, t)
    graspFrame = posemath.toTf(f)

    return graspFrame

def publishGraspingFrame(bin_name, obj_name):
    
    br = tf.TransformBroadcaster()
    listener = tf.TransformListener()
    r = rospy.Rate(10)

    while not rospy.is_shutdown():
        graspFrame = getGraspFrame(listener, bin_name, obj_name)

        br.sendTransform(graspFrame[0], graspFrame[1], \
                 rospy.Time.now(), \
                 "/grasp_frame",  \
                 bin_name \
                 )
        r.sleep()



if __name__ == "__main__":

    rospy.init_node('grasping_lib_test')
    publishGraspingFrame('shelf_bin_H', 'cheezit_big_original')