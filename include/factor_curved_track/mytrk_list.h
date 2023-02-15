#pragma once

#ifndef MYTRK_LIST_H
#define MYTRK_LIST_H

#include "mycommon_include.h"
#include "myframe.h"

namespace mytrk
{
struct myFrame;

//这是一个跟踪目标的状态列表，里面有存放一系列帧，最新的20个帧默认都是关键帧，对他们进行优化
//里面可以进行帧相关的操作，插入新的帧，移除旧的关键帧
class myTrkList
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    typedef std::shared_ptr<myTrkList> Ptr;
    typedef std::unordered_map<unsigned long, std::shared_ptr<myFrame> > KeyframeType;

    //航迹管理相关的操作
    //构造函数
    myTrkList(unsigned long id) : obj_id_(id) {};

    //设置轨迹的id
    void SetObjID(unsigned long id)
    {
        std::unique_lock<std::mutex> lck(data_mutex_);
        obj_id_ = id;
    }

    int GetObjID()
    {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return obj_id_;
    }

    //清空轨迹/删除轨迹,可能是合并两条轨迹后，把多的一条删了,实际上，把关键帧列表清空
    void CleanList()
    {
        std::unique_lock<std::mutex> lck(data_mutex_);
        keyframes_.clear();
        active_keyframes_.clear();
    }

    //合并轨迹，这个并不需要，各自帧的id相互独立，因此将两个trklist合并，要修改每一帧的id，较为复杂
    //直接在frontend里面多放几个trklist就好了
    void ConcatTrkList(myTrkList::Ptr old_trk_list);

    //帧相关的操作
    //插入关键帧到跟踪目标状态列表
    void InsertKeyframe(std::shared_ptr<myFrame> kf);

    //返回所有的关键帧
    KeyframeType GetAllKeyframe()
    {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return keyframes_;
    }

    //返回正在优化的关键帧
    KeyframeType GetActivateKeyframe()
    {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return active_keyframes_;
    }
    int GetKeyframeNum()
    {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return active_keyframes_.size();
    }
 

private:

    //设置最近帧和非最近帧Active与否
    //如果有非观测帧，有限删除非观测帧，但如果最老的观测帧和当前帧时间差超过max_time_gap，就删掉最老的观测帧
    //并且，如果最老旧的非观测帧距离当前帧，时间差距过小，也不删除非关键帧
    void ResetKeyframe();

    
    //TODO：添加一个measure-listtype，存放frontend计算的measure
    std::mutex data_mutex_;
    KeyframeType keyframes_;
    KeyframeType active_keyframes_;

    std::shared_ptr<myFrame> cur_frame_ = nullptr;

    unsigned long obj_id_;
    int num_active_keyframes = 20;
    double max_obs_time_gap = 3.0;
    double max_ext_time_gap = 0.7;  

};


} // namespace mytrk


#endif