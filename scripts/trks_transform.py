#!/home/chenz/anaconda3/envs/surroundCam_cap/bin/python
# -*- coding: utf-8 -*-
# 将trks从当前起始帧坐标系转换到当前帧坐标系，目前是基于读取的kitti官方的轨迹文件实现，后续改成基于来自slam算法定位信息实现

import rospy
import numpy as np
import message_filters
from .kitti_oxts import load_oxts_packets_and_poses

import argparse
import os

from track_msgs.msg import Detection_list
from track_msgs.msg import StampArray

def parse_args():
    parser = argparse.ArgumentParser(description='cord_transform')
    parser.add_argument('--datadir', type=str, default='/home/chenz/GD/dataset')
    parser.add_argument('--dataset', type=str, default='KITTI', help='KITTI, nuScenes')
    parser.add_argument('--split', type=str, default='training', help='training, testing')
    parser.add_argument('--seqs', type=str, default='0001')
    args = parser.parse_args()
    return args

def save_results(res, save_trk_file, eval_file, det_id2str, frame, score_threshold):

	# box3d in the format of h, w, l, x, y, z, theta in camera coordinate
	# TODO这个Orintmp是如何更新的没有看到
	# TODO这个id_tmp是什么，是每条轨迹的绝对id，不是相对于某一时刻的活跃轨迹的排名
	bbox3d_tmp, id_tmp, ori_tmp, type_tmp, bbox2d_tmp_trk, conf_tmp = \
		res[0:7], res[7], res[8], det_id2str[res[9]], res[10:14], res[14] 		
	 
	# save in detection format with track ID, can be used for dection evaluation and tracking visualization
	str_to_srite = '%s -1 -1 %f %f %f %f %f %f %f %f %f %f %f %f %f %d\n' % (type_tmp, ori_tmp,
		bbox2d_tmp_trk[0], bbox2d_tmp_trk[1], bbox2d_tmp_trk[2], bbox2d_tmp_trk[3], 
		bbox3d_tmp[0], bbox3d_tmp[1], bbox3d_tmp[2], bbox3d_tmp[3], bbox3d_tmp[4], bbox3d_tmp[5], bbox3d_tmp[6], conf_tmp, id_tmp)
	save_trk_file.write(str_to_srite)

	# save in tracking format, for 3D MOT evaluation
	# 记录当前帧的跟踪结果，bbox3d数据均为使用检测结果更新后的数据
	# frame是当前帧的id
	# id_tmp： 在整个跟踪算法过程中，会有很多条轨迹出现消失，中间也可能会将一条轨迹分成两条，
	# 不管怎么样，id_tmp是在整个跟踪算法运行期间，某一条轨迹的绝对id
	# 一下的格式和kitti官方一致
	if conf_tmp >= score_threshold:
		str_to_srite = '%d %d %s 0 0 %f %f %f %f %f %f %f %f %f %f %f %f %f\n' % (frame, id_tmp, 
			type_tmp, ori_tmp, bbox2d_tmp_trk[0], bbox2d_tmp_trk[1], bbox2d_tmp_trk[2], bbox2d_tmp_trk[3], 
			bbox3d_tmp[0], bbox3d_tmp[1], bbox3d_tmp[2], bbox3d_tmp[3], bbox3d_tmp[4], bbox3d_tmp[5], bbox3d_tmp[6], conf_tmp)
		eval_file.write(str_to_srite)

def transform_callback(trks, trks_id, args):
    imu_pose = args[0]
    # 检测结果帧对应的车辆位姿（以初始时刻的坐标为原点，坐标方向为正东）
    # stamp使用frameid代替
    ego_Oxt = imu_pose[int(trks.header.stamp.sec)]
    ego_trans = ego_Oxt.T_w_imu[0:3, 3]
    ego_rotZ = ego_Oxt.packet.yaw
    # 这里，将trk车辆在全局坐标系下的pos/roty减去自车在全局坐标系下的pos/roty
    # TODO：位置可以直接减，roty直接减的可行性需要分析一下
    for trk in trks.detecs:
        trk.pos = trk.pos - ego_trans
        trk.alp = trk.alp - ego_rotZ
        while(trk.alp > 3.14159 / 2):
            trk.alp -= 3.14159
        while(trk.alp < -3.14159 / 2):
            trk.alp += 3.14159
        
            






def transform(args):
    #TODO 添加接收来自slam的本车位置msg的功能
    oxt_path = os.path.join(args.datadir, args.dataset, "oxts" ,args.val, args.seqs + '.txt')
    #返回的imupose是OxtsData的list，每个OxtsData包含一条原始的oxt数据，和变换后的，相较于起始帧位置的SE3矩阵
    #Poses are given in an East-North-Up coordinate system， whose origin is the first GPS position.
    imu_pose = load_oxts_packets_and_poses(oxt_path)
    rospy.init_node('trks_transform', anonymous=True)

    trks = message_filters.Subscriber('/tracks_cur_state', Detection_list, queue_size = 10)
    
    trk_ids = message_filters.Subscriber('/tracks_id', StampArray, queue_size = 10)

    
    ts = message_filters.ApproximateTimeSynchronizer([trks, trk_ids], 10, 1, allow_headerless= True)

    ts.registerCallback(transform_callback, (imu_pose))

    rospy.spin()



if __name__ == '__main__':
    args = parse_args()
    try:
        transform(args)
    except rospy.ROSInterruptException:
        pass