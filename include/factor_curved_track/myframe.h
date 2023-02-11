#pragma once
#ifndef MYTRK_FRAME_H
#define MYTRK_FRAME_H

#include "mycommon_include.h"
#include "mytrk_list.h"

namespace mytrk
{

class myTrkList;


//这个结构里面存放每一帧上，某一个目标的状态，就是CTRA的六个数据
//这个里面有函数，实现对目标状态的更新
//初始化数据来自于目标检测结果和上一个时刻的状态递推
struct myFrame
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    
    typedef std::shared_ptr<myFrame> Ptr;

    unsigned long id_;
    bool is_measure_ = true;
    double time_stamp_;
    //x,y,theta,v,a,omega
    Vec6 obj_state_;

    std::mutex data_lck_;

public:
    myFrame() {};

    //初始化
    myFrame(long id, double time_stamp, Vec6 obj_state, bool is_measure) : 
    id_(id), time_stamp_(time_stamp), obj_state_(obj_state), is_measure_(is_measure) {};

    //返回当前帧的目标检测位置信息
    Vec6 ObjState()
    {
        std::unique_lock<std::mutex> lck(data_lck_);
        return obj_state_;
    }
    //返回当前帧的时间戳
    double ObjTimestamp()
    {
        std::unique_lock<std::mutex> lck(data_lck_);
        return time_stamp_;
    }
    //给定当前帧的车辆检测结果（其应该是绝对位置），设置本帧的目标位置信息
    void SetObjState(const Vec6 &obj_state)
    {
        std::unique_lock<std::mutex> lck(data_lck_);
        obj_state_ = obj_state;
    }

    /// 工厂构建模式，批量分配id,用于测试，正常应用中不会批量构建frame
    static std::shared_ptr<myFrame> CreateFrame();

};

} // namespace mytrk


#endif