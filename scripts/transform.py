#!/home/chenz/anaconda3/envs/surroundCam_cap/bin/python
# -*- coding: utf-8 -*-
# 将检测结果从当前帧坐标系转换到起始帧坐标系，目前是基于kitti官方的轨迹文件实现，后续改成可以接收来自slam算法的结果

import rospy
import numpy as np
from .kitti_oxts import load_oxts_packets_and_poses

import argparse
import os

from track_msgs.msg import Detection_list

def parse_args():
    parser = argparse.ArgumentParser(description='cord_transform')
    parser.add_argument('--datadir', type=str, default='/home/chenz/GD/dataset')
    parser.add_argument('--dataset', type=str, default='KITTI', help='KITTI, nuScenes')
    parser.add_argument('--split', type=str, default='val', help='train, val, test')
    parser.add_argument('--seqs', type=str, default='0001')
    args = parser.parse_args()
    return args


def get_ego_traj(imu_poses, frame, pref, futf, inverse=False, only_fut=False):
    # compute the motion of the ego vehicle for ego-motion compensation
    # using the current frame as the coordinate
    # current frame means one frame prior to future, and also the last frame of the past
    
    # compute the start and end frame to retrieve the imu poses
    num_frames = imu_poses.shape[0]
    assert frame >= 0 and frame <= num_frames - 1, 'error'
    if inverse:             # pre and fut are inverse, i.e., inverse ego motion compensation
        start = min(frame+pref-1, num_frames-1)
        end   = max(frame-futf-1, -1)
        index = [*range(start, end, -1)]
    else:
        start = max(frame-pref+1, 0)
        end   = min(frame+futf+1, num_frames)
        index = [*range(start, end)]
    
    # compute frame offset due to sequence boundary
    left  = start - (frame-pref+1)
    right = (frame+futf+1) - end

    # compute relative transition compared to the current frame of the ego
    all_world_xyz = imu_poses[index, :3, 3]    # N x 3, only translation, frame = 10-19 for fut only (0-19 for all)
    cur_world_xyz = imu_poses[frame]                        # 4 x 4, frame = 9
    T_world2imu   = np.linalg.inv(cur_world_xyz)            
    all_world_hom = np.concatenate((all_world_xyz, np.ones((all_world_xyz.shape[0], 1))), axis=1)       # N x 4
    all_xyz = all_world_hom.dot(T_world2imu.T)[:, :3]       # N x 3

    # compute relative rotation compared to the current frame of the ego
    all_world_rot = imu_poses[index, :3, :3]   # N x 3 x 3, only rotation
    cur_world_rot = imu_poses[frame, :3, :3]                # 3 x 3, frame = 9
    T_world2imu_rot = np.linalg.inv(cur_world_rot)        
    all_rot_list = list()
    for frame in range(all_world_rot.shape[0]):
        all_rot_tmp = all_world_rot[frame].dot(T_world2imu_rot)   # 3 x 3
        all_rot_list.append(all_rot_tmp)
    
    if only_fut:
        fut_xyz, fut_rot_list = all_xyz[pref-left:], all_rot_list[pref-left:]
        return fut_xyz, fut_rot_list, left, right
    else:
        return all_xyz, all_rot_list, left, right

def transform_callback(dets, args):
    imu_pose = args[0]
    dets_puber = args[1]
    # 检测结果帧对应的车辆位姿（以初始时刻的坐标为原点，坐标方向为正东）
    # stamp使用frameid代替
    ego_Oxt = imu_pose[dets.header.stamp.sec]
    ego_trans = ego_Oxt.T_w_imu[0:3, 3]
    ego_rotZ = ego_Oxt.packet.yaw
    # 这里，将自车在全局坐标系下的roty和检测结果车辆在自车坐标系下的roty相加
    # TODO 将结果调整到正副pi/2中
    for det in dets.detecs:
        det.pos = det.pos + ego_trans
        det.alp = det.alp + ego_rotZ
        while(det.alp > 3.14159 / 2):
            det.alp -= 3.14159
        while(det.alp < -3.14159 * 2):
            det.alp += 3.14159
            
    
    dets_puber.publish(dets)





def transform(args):
    #TODO 添加接收来自slam的本车位置msg的功能
    oxt_path = os.path.join(args.datadir, args.dataset, "oxts" ,args.val, args.seqs + '.txt')
    #返回的imupose是OxtsData的list，每个OxtsData包含一条原始的oxt数据，和变换后的，相较于起始帧位置的SE3矩阵
    imu_pose = load_oxts_packets_and_poses(oxt_path)
    rospy.init_node('cord_transform', anonymous=True)
    
    dets_puber = rospy.Publisher('/detections', Detection_list, queue_size = 10)

    rospy.Subscriber("/orin_detections", Detection_list, transform_callback, (imu_pose, dets_puber))

    rospy.spin()



if __name__ == '__main__':
    args = parse_args()
    try:
        transform(args)
    except rospy.ROSInterruptException:
        pass