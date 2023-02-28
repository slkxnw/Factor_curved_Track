#!/home/chenz/anaconda3/envs/surroundCam_cap/bin/python
# -*- coding: utf-8 -*-
# 将检测结果从当前帧坐标系转换到起始帧坐标系，目前是基于读取的kitti官方的轨迹文件实现，后续改成基于来自slam算法定位信息实现

import rospy
import numpy as np
from kitti_oxts import load_oxts_packets_and_poses
from kitti_calib import Calibration
import time

import argparse
import os

from track_msgs.msg import Detection_list
from track_msgs.srv import Det_pub
from track_msgs.srv import Data_association

# TODO :设置两种模式，跑数据集，使用服务获取原始检测结果，跑试车，订阅topic

seq_length = {'0001':447, '0006':270, '0008':390, '0010':294, '0012':78, '0013':340, '0014':106, '0015':376, '0016':209, '0018':339, '0019':1059}

def parse_args():
    parser = argparse.ArgumentParser(description='cord_transform')
    parser.add_argument('--datadir', type=str, default='/home/chenz/GD/dataset')
    parser.add_argument('--dataset', type=str, default='KITTI', help='KITTI, nuScenes')
    parser.add_argument('--split', type=str, default='training', help='training, testing')
    parser.add_argument('--seqs', type=str, default='0001')
    parser.add_argument('__name', type=str)
    parser.add_argument('__log', type=str)
    args = parser.parse_args()
    return args


# 实际上就是不进行任何变换
def transform_callback(dets):
    # imu_pose = args[0]
    # dets_puber = args[1]
    # 检测结果帧对应的车辆位姿（以初始时刻的坐标为原点，坐标方向为正东）
    # stamp使用frameid代替
    ego_Oxt = imu_pose[int(dets.header.stamp.secs)]
    ego_trans = ego_Oxt.T_w_imu[0:3, 3]
    ego_rot = ego_Oxt.T_w_imu[0:3, 0:3]
    # print("ego:",ego_trans)
    ego_rotZ = ego_Oxt.packet.yaw
    # 这里，将自车在全局坐标系下的roty和检测结果车辆在自车坐标系下的roty相加
    if(int(dets.header.stamp.secs) % 1 == 0):
        rospy.loginfo("Transform cord of dets in frame %d", int(dets.header.stamp.secs))
    # dets_puber.publish(dets)
    res = process_asso(dets)
    rospy.loginfo("Frame %d association state is %d", int(dets.header.stamp.secs),res.success)
    


def transform(args):
    #TODO 添加接收来自slam的本车位置msg的功能
    oxt_path = [os.path.join(args.datadir, args.dataset, "oxts" ,args.split, args.seqs + '.txt')]
    calib_path = os.path.join('/home/chenz/GD/Trk/AB3DMOT/data/KITTI/tracking/training/calib', args.seqs + '.txt')
    #返回的imupose是OxtsData的list，每个OxtsData包含一条原始的oxt数据，和变换后的，相较于起始帧位置的SE3矩阵
    #Poses are given in an East-North-Up coordinate system， whose origin is the first GPS position.
    global imu_pose, calib

    calib = Calibration(calib_path)
    imu_pose = load_oxts_packets_and_poses(oxt_path)
    # print('length of imu_pose is %d', len(imu_pose))
    
    rospy.init_node('dets_transform', anonymous=True)
    rospy.loginfo('there is %d imupose', len(imu_pose))

    # dets_puber = rospy.Publisher('/detections', Detection_list, queue_size = 10)

    rate = rospy.Rate(8)
    rospy.wait_for_service('/det_pub')
    rospy.wait_for_service('/data_association')
    try:
        get_dets_orin = rospy.ServiceProxy('/det_pub', Det_pub)
    except rospy.ServiceException as e:
        rospy.logwarn(e)
    global process_asso
    try:
        process_asso = rospy.ServiceProxy('/data_association', Data_association)
    except rospy.ServiceException as e:
        rospy.logwarn(e)
    frame = 0
    while frame < seq_length[args.seqs]:
    # while frame < 10:
        res = get_dets_orin(frame)
        # transform_callback(dets,(imu_pose, dets_puber))
        transform_callback(res.dets)
        rate.sleep()
        # time.sleep(2)
        frame = frame + 1 
    # 
    # rospy.loginfo("Transform cord of dets")

    # rospy.Subscriber("/orin_detections", Detection_list, transform_callback, (imu_pose, dets_puber))

    # rospy.spin()


if __name__ == '__main__':
    args = parse_args()
    try:
        transform(args)
    except rospy.ROSInterruptException:
        pass