#!/usr/bin/python

import pickle
from collections import namedtuple
import rospkg
import rospy
from grasping.myTypes import *
# objAttr = namedtuple('objAttr', ['name', 'invalidApproachAxis', 'invalidGraspAxis', 'graspStrategy'])

'''
name: name of the object
invalidApproachAxis <list>: the axis along which the object cannot be approached
invalidGraspAxis <list>: the axis around which the object cannot be grasped
graspStrategy: 0 -> sideGrasp, 1 -> topGrasp, 2 -> boundingBoxGrasp

axis mapping: (x,y,z) -> (0,1,2)
'''




class objDict:

    def __init__(self):
        self.dict = {}

        # get filepath of shelf stl file
        rp = rospkg.RosPack()
        try:
            dict_fp = rp.get_path('amazon_challenge_grasping')

        except:
            rospy.logerr('[generate_object_dict]: object dict file not found!')
            sys.exit(1)

        dict_fp = dict_fp + '/config/grasp_dict.dat'
        self.fileName = dict_fp
        self.loaded = {}



    def makeDict(self):

        invalidApproachAxis = [2]
        invalidGraspAxis = [2]
        cheezit_big_original = objAttr('cheezit_big_original', invalidApproachAxis, invalidGraspAxis, 0)
        self.dict['cheezit_big_original'] = cheezit_big_original

    def saveDict(self):

        with open(self.fileName, 'wb') as handle:
            pickle.dump(self.dict, handle)

    def loadDict(self):
       
        with open(self.fileName, 'rb') as handle:
            self.loaded = pickle.load(handle)

    def getEntry(self, name):
        
        name = name.strip('/')

        if not self.loaded:
            self.loadDict()
        return self.loaded[name]



if __name__ == "__main__":

    rospy.init_node('test')
    dictObj = objDict()
    dictObj.makeDict()
    dictObj.saveDict()
    dictObj.loadDict()
    c = dictObj.getEntry('cheezit_big_original')
    print c
