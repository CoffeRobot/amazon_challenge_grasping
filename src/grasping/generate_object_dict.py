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
easy: True/False

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
        cheezit_big_original = objAttr('cheezit_big_original', invalidApproachAxis, invalidGraspAxis, [0], True, -0.2, True, 0.11)
        self.dict['cheezit_big_original'] = cheezit_big_original

        invalidApproachAxis = [2]
        invalidGraspAxis = [2]
        crayola_64_ct = objAttr('crayola_64_ct', invalidApproachAxis, invalidGraspAxis, [0], True, -0.15, True, 0.065)
        self.dict['crayola_64_ct'] = crayola_64_ct

        invalidApproachAxis = [1]
        invalidGraspAxis = [2]
        elmers_washable_no_run_school_glue = objAttr('elmers_washable_no_run_school_glue', invalidApproachAxis, invalidGraspAxis, [0,1], True, -0.15, True, 0)
        self.dict['elmers_washable_no_run_school_glue'] = elmers_washable_no_run_school_glue

        invalidApproachAxis = []
        invalidGraspAxis = []
        first_years_take_and_toss_straw_cup = objAttr('first_years_take_and_toss_straw_cup', invalidApproachAxis, invalidGraspAxis, [1,0], True, -0.15, False, 0)
        self.dict['first_years_take_and_toss_straw_cup'] = first_years_take_and_toss_straw_cup

        invalidApproachAxis = []
        invalidGraspAxis = []
        kong_air_dog_squeakair_tennis_ball = objAttr('kong_air_dog_squeakair_tennis_ball', invalidApproachAxis, invalidGraspAxis, [1], True, -0.1, False, 0)
        self.dict['kong_air_dog_squeakair_tennis_ball'] = kong_air_dog_squeakair_tennis_ball

        invalidApproachAxis = []
        invalidGraspAxis = []
        kong_duck_dog_toy = objAttr('kong_duck_dog_toy', invalidApproachAxis, invalidGraspAxis, [1], True, -0.15, False, 0)
        self.dict['kong_duck_dog_toy'] = kong_duck_dog_toy

        invalidApproachAxis = []
        invalidGraspAxis = []
        kong_sitting_frog_dog_toy = objAttr('kong_sitting_frog_dog_toy', invalidApproachAxis, invalidGraspAxis, [1], True, -0.15, False, 0)
        self.dict['kong_sitting_frog_dog_toy'] = kong_sitting_frog_dog_toy

        invalidApproachAxis = []
        invalidGraspAxis = []
        kyjen_squeakin_eggs_plush_puppies = objAttr('kyjen_squeakin_eggs_plush_puppies', invalidApproachAxis, invalidGraspAxis, [1], True, -0.1, False, 0)
        self.dict['kyjen_squeakin_eggs_plush_puppies'] = kyjen_squeakin_eggs_plush_puppies


        invalidApproachAxis = [2]
        invalidGraspAxis = [2]
        laugh_out_loud_joke_book = objAttr('laugh_out_loud_joke_book', invalidApproachAxis, invalidGraspAxis, [0], False, -0.1, True, 0.064)
        self.dict['laugh_out_loud_joke_book'] = laugh_out_loud_joke_book

        invalidApproachAxis = [2]
        invalidGraspAxis = [2]
        mead_index_cards = objAttr('mead_index_cards', invalidApproachAxis, invalidGraspAxis, [1], True, -0.15, True, 0)
        self.dict['mead_index_cards'] = mead_index_cards

        invalidApproachAxis = []
        invalidGraspAxis = []
        munchkin_white_hot_duck_bath_toy = objAttr('munchkin_white_hot_duck_bath_toy', invalidApproachAxis, invalidGraspAxis, [1], True, -0.15, False, 0)
        self.dict['munchkin_white_hot_duck_bath_toy'] = munchkin_white_hot_duck_bath_toy

        invalidApproachAxis = [2]
        invalidGraspAxis = [2]
        oreo_mega_stuf = objAttr('oreo_mega_stuf', invalidApproachAxis, invalidGraspAxis, [0], True, -0.2, True, 0.11)
        self.dict['oreo_mega_stuf'] = oreo_mega_stuf

        invalidApproachAxis = [0]
        invalidGraspAxis = [0]
        paper_mate_12_count_mirado_black_warrior = objAttr('paper_mate_12_count_mirado_black_warrior', invalidApproachAxis, invalidGraspAxis, [1,0], True, -0.1, False, 0)
        self.dict['paper_mate_12_count_mirado_black_warrior'] = paper_mate_12_count_mirado_black_warrior

        invalidApproachAxis = [2]
        invalidGraspAxis = [2]
        sharpie_accent_tank_style_highlighters = objAttr('sharpie_accent_tank_style_highlighters', invalidApproachAxis, invalidGraspAxis, [0], False, -0.15, True, 0)
        self.dict['sharpie_accent_tank_style_highlighters'] = sharpie_accent_tank_style_highlighters

        invalidApproachAxis = [2]
        invalidGraspAxis = [2]
        stanley_66_052 = objAttr('stanley_66_052', invalidApproachAxis, invalidGraspAxis, [1,0], True, -0.1, False, 0)
        self.dict['stanley_66_052'] = stanley_66_052

        invalidApproachAxis = []
        invalidGraspAxis = []
        champion_copper_plus_spark_plug = objAttr('champion_copper_plus_spark_plug', invalidApproachAxis, invalidGraspAxis, [1], True, -0.1, True, 0)
        self.dict['champion_copper_plus_spark_plug'] = champion_copper_plus_spark_plug

        invalidApproachAxis = []
        invalidGraspAxis = []
        dr_browns_bottle_brush = objAttr('dr_browns_bottle_brush', invalidApproachAxis, invalidGraspAxis, [1], True, -0.1, True, 0)
        self.dict['dr_browns_bottle_brush'] = dr_browns_bottle_brush

        invalidApproachAxis = []
        invalidGraspAxis = []
        expo_dry_erase_board_eraser = objAttr('expo_dry_erase_board_eraser', invalidApproachAxis, invalidGraspAxis, [1], True, -0.1, True, 0)
        self.dict['expo_dry_erase_board_eraser'] = expo_dry_erase_board_eraser

        invalidApproachAxis = [2]
        invalidGraspAxis = [2]
        feline_greenies_dental_treats = objAttr('feline_greenies_dental_treats', invalidApproachAxis, invalidGraspAxis, [0], True, -0.1, True, 0.075)
        self.dict['feline_greenies_dental_treats'] = feline_greenies_dental_treats

        invalidApproachAxis = [2]
        invalidGraspAxis = [2]
        genuine_joe_plastic_stir_sticks = objAttr('genuine_joe_plastic_stir_sticks', invalidApproachAxis, invalidGraspAxis, [0], False, -0.1, True, 0.075)
        self.dict['genuine_joe_plastic_stir_sticks'] = genuine_joe_plastic_stir_sticks

        invalidApproachAxis = []
        invalidGraspAxis = []
        highland_6539_self_stick_notes = objAttr('highland_6539_self_stick_notes', invalidApproachAxis, invalidGraspAxis, [1], True, -0.1, False, 0)
        self.dict['highland_6539_self_stick_notes'] = highland_6539_self_stick_notes

        invalidApproachAxis = [2]
        invalidGraspAxis = [2]
        mark_twain_huckleberry_finn = objAttr('mark_twain_huckleberry_finn', invalidApproachAxis, invalidGraspAxis, [0], False, -0.1, True, 0.064)
        self.dict['mark_twain_huckleberry_finn'] = mark_twain_huckleberry_finn

        invalidApproachAxis = []
        invalidGraspAxis = []
        mommys_helper_outlet_plugs = objAttr('mommys_helper_outlet_plugs', invalidApproachAxis, invalidGraspAxis, [1], True, -0.1, True, 0)
        self.dict['mommys_helper_outlet_plugs'] = mommys_helper_outlet_plugs

        invalidApproachAxis = []
        invalidGraspAxis = []
        munchkin_white_hot_duck_bath_toy = objAttr('munchkin_white_hot_duck_bath_toy', invalidApproachAxis, invalidGraspAxis, [1], True, -0.1, False, 0)
        self.dict['munchkin_white_hot_duck_bath_toy'] = munchkin_white_hot_duck_bath_toy

        invalidApproachAxis = []
        invalidGraspAxis = []
        rolodex_jumbo_pencil_cup = objAttr('rolodex_jumbo_pencil_cup', invalidApproachAxis, invalidGraspAxis, [0], False, -0.15, False, 0)
        self.dict['rolodex_jumbo_pencil_cup'] = rolodex_jumbo_pencil_cup

        invalidApproachAxis = []
        invalidGraspAxis = []
        safety_works_safety_glasses = objAttr('safety_works_safety_glasses', invalidApproachAxis, invalidGraspAxis, [1], True, -0.15, False, 0)
        self.dict['safety_works_safety_glasses'] = safety_works_safety_glasses

        # this is here just for safety
        invalidApproachAxis = []
        invalidGraspAxis = []
        dummyEntry = objAttr('dummyEntry', invalidApproachAxis, invalidGraspAxis, [1,0], True, -0.15, False, 0)
        self.dict['dummyEntry'] = dummyEntry














    def saveDict(self):
        with open(self.fileName, 'wb') as handle:
            pickle.dump(self.dict, handle)

    def loadDict(self):
        with open(self.fileName, 'rb') as handle:
            self.loaded = pickle.load(handle)
    def getSize(self):
        if not self.loaded:
            self.loadDict()

        return len(self.loaded)

    def getEntry(self, name):

        name = name.lstrip('/')
        if not self.loaded:
            self.loadDict()

        try:
            return self.loaded[name]
        except:
            rospy.logerr('object not in the objDict, using the dummyEntry')
            return self.loaded['dummyEntry']




if __name__ == "__main__":

    rospy.init_node('test')
    dictObj = objDict()
    dictObj.makeDict()
    dictObj.saveDict()
    dictObj.loadDict()
    c = dictObj.getEntry('elmers_washable_no_run_school_glue')
    print c
    print dictObj.getSize()
