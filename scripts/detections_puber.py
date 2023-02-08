#!/home/chenz/anaconda3/envs/surroundCam_cap/bin/python
#读取某个轨迹检测结果序列文件，并按照帧率发布结果

import rospy
import numpy as np
import warnings
import argparse
import os
from std_msgs.msg import Float64MultiArray
from Factor_curved_Track.msg import Detection_list
from Factor_curved_Track.msg import Detection
from Factor_curved_Track.msg import Information


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
	ori_array = dets_all[dets_all[:, 0] == frame, -1].reshape((-1, 1))		# orientation
	other_array = dets_all[dets_all[:, 0] == frame, 1:7] 					# other information, e.g, 2D box, ...
	additional_info = np.concatenate((ori_array, other_array), axis=1)		

	# get 3D box
	dets = dets_all[dets_all[:, 0] == frame, 7:14]		

	return dets, additional_info

def detection_puber(args):
    rospy.init_node('detection_puber', anonymous = True)

    detection_res_pub = rospy.Publisher('/detections', Detection_list, queue_size = 10)

    # TODO：数据关联接收来自两个节点的数据，为了不造成数据匹配的误差问题，将数据发布频率降低
    rate = rospy.Rate(10)
    
    path = os.path.join(args.datadir, args.dataset, args.det_name + '_' + args.categ + '_' + args.val, args.seqs + '.txt');
    
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
        for det,info in zip(dets_frame, infos_frame):
            inf = Information()
            inf.type = info[0]
            inf.unknow = info[5]
            inf.orin = info[6]
            det_ = Detection()
            det_.siz = det[0:3]
            det_.pos = det[3:6]
            det_.alp = det[7]
            det_list.detecs.append(det_)
            det_list.infos.append(inf)
        detection_res_pub.publish(det_list)
        rospy.loginfo("pub frame %d with %d detections", frame_id, det_list.infos.size())
        frame_id = frame_id + 1

# TODO:设置退出，当遍历当前seq所有帧后，结束程序

if __name__ == '__main__':
    args = parse_args()
    try:
        detection_puber(args)
    except rospy.ROSInterruptException:
        pass