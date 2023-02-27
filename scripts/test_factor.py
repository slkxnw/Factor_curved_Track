#!/home/chenz/anaconda3/envs/surroundCam_cap/bin/python
# -*- coding: utf-8 -*-


import rospy
import numpy as np
import time
import pickle

import argparse
import os
import math
import random
import copy

from track_msgs.msg import Detection_list
from track_msgs.msg import Pairs
from track_msgs.msg import StampArray
from track_msgs.msg import Detection
from track_msgs.msg import Information
from track_msgs.srv import Trk_pred
from track_msgs.srv import Trk_update
from track_msgs.srv import Data_association, Data_associationResponse
from track_msgs.srv import Trk_state_store


dt = 0.1
# 车辆初始状态
x0 = 0
y0 = 0
th0 = 0
v0 = 10
a0 = 0
w0 = 0.0001


#车辆状态更新
def updateCarState(state):
    [x,y,th,v,a,w] = state
    dth = w * dt
    dv = a * dt
    dx = ((v * w + a * dth) * math.sin(th + dth) + a * math.cos(th + dth)
         - v * w * math.sin(th) - a * math.cos(th)) / ((w + 1e-9) * (w + 1e-9))
    dy = ((-v * w - a * dth) * math.cos(th + dth) + a * math.sin(th + dth)
         + v * w * math.cos(th) - a * math.sin(th)) / ((w + 1e-9) * (w + 1e-9))
    # print('plus:', dx,dy)
    state[0] = state[0] + dx
    state[1] = state[1] + dy
    state[2] = state[2] + dth
    state[3] = state[3] + dv
    # state[4] = state[4] + random.gauss(0, 0.05)
    state[4] = random.gauss(0, 0.05)
    print('gt_state:',state)
    return state


if __name__ == '__main__':
    rospy.init_node('test_factor', anonymous=True)
    rospy.wait_for_service('/trk_predict')
    rospy.wait_for_service('/trk_update')

    try:
        process_trk_update = rospy.ServiceProxy('/trk_update', Trk_update)
    except rospy.ServiceException as e:
        rospy.logwarn(e)
    try:
        get_trk_preds = rospy.ServiceProxy('/trk_predict', Trk_pred)
    except rospy.ServiceException as e:
        rospy.logwarn(e)
    
    frame = 0
    state = [x0, y0, th0, v0, a0, w0]
    states = []
    while(frame < 40):
        preds = get_trk_preds(frame / 10);
        preds = preds.trk_predicts
        for pred in preds.detecs:
            print('frame:',frame)
            print('pred:',[pred.pos.x, pred.pos.y, pred.pos.z])
        
        pub_match = Pairs()
        pub_match.header.stamp.secs = frame

        pub_undets = StampArray()
        pub_undets.header = pub_match.header

        pub_untrks = StampArray()
        pub_untrks.header = pub_match.header

        if(frame == 0):
            pub_undets.ids.data = [0]
        else:
            pub_match.dets.data = [0]
            pub_match.trk.data = [0]
            state = updateCarState(state)
        states.append(copy.deepcopy(state))
        det = Detection()
        det.alp = state[2] + random.gauss(0, 0.02)
        det.pos.x = state[0] + random.gauss(0, 0.1)
        det.pos.y = state[1] + random.gauss(0, 0.05)
        det.pos.z = 1
        det.siz.x = 2
        det.siz.y = 3
        det.siz.z = 2
        info = Information()
        info.orin = 0
        info.score = 5
        info.type = 0

        dets = Detection_list()
        dets.header = pub_match.header
        dets.detecs.append(det)
        dets.infos.append(info)

        update_res = process_trk_update(pub_match, pub_undets, pub_untrks, dets)
        frame = frame + 1
    for item in states:
        print(item)
    pickle.dump(states, open('CV_states.pkl','wb'))


