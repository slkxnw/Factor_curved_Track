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


def parse_args():
    parser = argparse.ArgumentParser(description='Detections_Puber')
    parser.add_argument('--datadir', type=str, default='/home/chenz/GD/dataset')
    parser.add_argument('--dataset', type=str, default='KITTI', help='KITTI, nuScenes')
    parser.add_argument('--split', type=str, default='val', help='train, val, test')
    parser.add_argument('--det_name', type=str, default='pointrcnn', help='pointrcnn')
    parser.add_argument('--categ', type=str, default='Car',help='Car, Cyclist, Pedestrain')
    parser.add_argument('--seqs', type=str, default='0001')
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

def detection_puber(args):
    rospy.init_node('detection_puber', anonymous = True)

    detection_res_pub = rospy.Publisher('/orin_detections', Detection_list, queue_size = 10)

    # TODO：数据关联接收来自两个节点的数据，为了不造成数据匹配的误差问题，将数据发布频率降低
    rate = rospy.Rate(10)
    
    path = os.path.join(args.datadir, args.dataset, 'trking', args.det_name + '_' + args.categ + '_' + args.val, args.seqs + '.txt')
    
    dets, flg = load_detection(path)
    if not flg:
        rospy.INFO('No Detections in %s', args.seqs)
    
    frame_id = 0

    while not rospy.is_shutdown():
        dets_frame,infos_frame = get_frame_det(dets, frame_id)
        det_list = Detection_list()
        # TODO：这里可能有问题，主要在于
        # 第一，在msg文件中设置了几个默认值，不知是否可行
        # 第二，不知道ros的数组在Python对应什么格式，目前是按照对应list来看的，
        # 因为，ROS的UInt16MultiArray Message在Python是一个类，其中self.data = []
        # TODO 确认坐标系，看了kittidevkit，z轴是向前的，那么我们需要的是x和z的坐标位置
        # 从kitti-devkit给的图来看，roty就是w
        for det,info in zip(dets_frame, infos_frame):
            inf = Information()
            inf.type = info[0]
            inf.score = info[5]# 实际上是检测的score
            inf.orin = info[6]
            det_ = Detection()
            det_.siz = det[0:3]
            det_.pos = det[3:6]
            det_.alp = det[7]
            det_list.detecs.append(det_)
            det_list.infos.append(inf)
        
        # TODO 这里将时间戳设为frameid，为了方便在数据关联的数据对齐中，使用ros的同步方法
        det_list.header.stamp.sec = frame_id
        det_list.header.stame.nsec = 0
        
        detection_res_pub.publish(det_list)
        rospy.loginfo("pub frame %d with %d detections", frame_id, det_list.infos.size())
        frame_id = frame_id + 1
        if(frame_id == 447):
            break
        rate.sleep()

# TODO:需要设置退出，当遍历当前seq所有帧后，结束程序,目前是按照val的0001序列设计的退出，它共有446帧，因此循环这些次

# TODO:需要设置可视化，或者跟踪结果存储，后面这个可以在backend析构函数中实现
if __name__ == '__main__':
    args = parse_args()
    try:
        detection_puber(args)
    except rospy.ROSInterruptException:
        pass