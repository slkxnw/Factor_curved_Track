#!/home/chenz/anaconda3/envs/surroundCam_cap/bin/python
# -*- coding: utf-8 -*-
#读取某个轨迹检测结果序列文件，并按照帧率发布结果

import rospy
import numpy as np
import warnings
import argparse
import os

from track_msgs.msg import Detection_list
from track_msgs.msg import Detection
from track_msgs.msg import Information
from track_msgs.srv import Det_pub,Det_pubResponse


def parse_args():
    parser = argparse.ArgumentParser(description='Detections_Puber')
    parser.add_argument('--datadir', type=str, default='/home/chenz/GD/dataset')
    parser.add_argument('--dataset', type=str, default='KITTI', help='KITTI, nuScenes')
    parser.add_argument('--split', type=str, default='val', help='train, val, test')
    parser.add_argument('--det_name', type=str, default='pointrcnn', help='pointrcnn')
    parser.add_argument('--categ', type=str, default='Car',help='Car, Cyclist, Pedestrain')
    parser.add_argument('--seqs', type=str, default='0019')
    parser.add_argument('__name', type=str)
    parser.add_argument('__log', type=str)
    args = parser.parse_args()
    return args

def load_detection(file):

	# load from raw file
	with warnings.catch_warnings():
		warnings.simplefilter("ignore")
		dets = np.loadtxt(file, delimiter=',') 	# load detections, N x 15

	if len(dets.shape) == 1: dets = np.expand_dims(dets, axis=0) 	
	if dets.shape[1] == 0:		# if no detection in a sequence
		return [], False
	else:
		return dets, True

def get_frame_det(dets_all, frame):
	
	# get irrelevant information associated with an object, not used for associationg
	ori_array = dets_all[dets_all[:, 0] == frame, -1].reshape((-1, 1))		# orientation alpha
	other_array = dets_all[dets_all[:, 0] == frame, 1:7] 					# other information, e.g, typr + 2D box + score ...
	additional_info = np.concatenate((ori_array, other_array), axis=1)		

	# get 3D box:3D BBOX (h, w, l, x, y, z, rot_y)
	dets = dets_all[dets_all[:, 0] == frame, 7:14]		

	return dets, additional_info

def pub_callback(req):
    dets_frame, infos_frame = get_frame_det(dets_read, req.frame_id)
    det_list = Detection_list()
        # TODO 确认坐标系，看了kittidevkit，z轴是向前的，那么我们需要的是x和z的坐标位置
        # 从kitti-devkit给的图来看，roty就是w
    for det,info in zip(dets_frame, infos_frame):
        # if(info[6] < 0):
            # continue
        inf = Information()
        inf.type = int(info[1])
        inf.score = info[6]# 实际上是检测的score
        inf.orin = info[0]
            # print(inf)
        # det h,w,l,x,y,z,th
        det_ = Detection()
        det_.siz.x  = det[0]
        det_.siz.y  = det[1]
        det_.siz.z  = det[2]
        # z是向前的方向
        det_.pos.x = det[3]
        det_.pos.y = det[4]
        det_.pos.z = det[5]

        det_.alp = det[6]
        det_list.detecs.append(det_)
        det_list.infos.append(inf)
        
        # TODO 这里将时间戳设为frameid，为了方便在数据关联的数据对齐中，使用ros的同步方法
        # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
        # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    det_list.header.stamp.secs = req.frame_id
    det_list.header.stamp.nsecs = 0

    res = Det_pubResponse()
    res.dets = det_list
    rospy.loginfo("Thete is %d detes in frame %d", len(det_list.detecs), req.frame_id)
    return res

def detection_puber(args):
    rospy.init_node('detection_puber', anonymous = True)

    global dets_read

    path = os.path.join(args.datadir, args.dataset, 'trking', args.det_name + '_' + args.categ + '_' + args.split, args.seqs + '.txt')   
    dets_read, flg = load_detection(path)
    if not flg:
        rospy.INFO('No Detections in %s', args.seqs)
    

    s = rospy.Service('/det_pub', Det_pub, pub_callback)
    rospy.spin()

# TODO:需要设置退出，当遍历当前seq所有帧后，结束程序,目前是按照val的0001序列设计的退出，它共有446帧，因此循环这些次

# TODO 将和dets-trans的服务的客户端改到这里，符合后续实车实验要求
if __name__ == '__main__':
    args = parse_args()
    try:
        detection_puber(args)
    except rospy.ROSInterruptException:
        pass