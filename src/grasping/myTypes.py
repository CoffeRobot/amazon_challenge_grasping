#!/usr/bin/python


from collections import namedtuple

objAttr = namedtuple('objAttr', ['name', 'invalidApproachAxis', 'invalidGraspAxis', 'graspStrategy', 'easy', 'pre_distance', 'simtrack', 'xLength'])

shelfInfo = namedtuple('shelfInfo', ['odomL', 'odomL_rot', 'odomR', 'odomR_rot', 'timestamp'])