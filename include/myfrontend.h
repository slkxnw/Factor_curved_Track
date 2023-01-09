#pragma once

#ifndef MYTRK_FRONTEND_H
#define MYTRK_FRONTEND_H

#include "mycommon_include.h"
#include "myframe.h"
#include "mytrk_list.h"

namespace mytrk
{
struct myFrame;
class myTrkList;

//结合检测结果，结合上一针的状态，初始化Frame，并分配给map，然后构建一个因子图，进行优化
class myFrontend
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::unique_ptr<myFrontend> Ptr;
    // typedef std::vector<myTrkList::Ptr> TrkListType;

    //构造函数中，启动循环，并且挂起
    myFrontend();

    //航迹管理相关，初始化放在private里

    //合并两个轨迹
    void ConcatTrklist(myTrkList::Ptr old_trk_list);

    //删除轨迹
    void ClearTrklist();

    //设置轨迹的id
    void SetTrkId(unsigned long id)
    {
        trk_list_->SetObjID(id);
    }

    //结束该线程
    void Stop();

    //和frame相关操作，插入frame和获取上一帧状态放在private里

    //基于观测和上一帧数据，确定帧状态,同时更新teklist的帧数量信息
    myFrame::Ptr CreateMeasureFrame(Vec3 measure, double time, bool is_measure = true);

    //基于外推生成本帧状态,同时更新teklist的帧数量信息
    myFrame::Ptr CreateExtrpFrame(double time);
    
    //优化相关的操作

    //有了新的观测，触发优化,在外部使用这个frontend类的地方调用这个函数
    void UpdateTrkList();

    //TODO：得到边的测量，并把数据添加给measure——list
    //TODO：设计一个measurelist，他是tek-list的一部分，存放这里计算的结果
    Vec6 CalEdgeMeasure();

    //数据关联相关操作
    Vec3 PredictPostion(double time);

    Vec6 PredictState(double time);

    // void SetObjSize(Vec3 size)
    // {
    //     obj_size_ = size;
    // }
    // Vec3 GetObjSize()
    // {
    //     return obj_size_;
    // }


private:

    //初始化轨迹,设定初始车速为14，初始加速度和横摆角速度为0，根据匹配的观测结果和分配的目标id，初始化轨迹
    bool BuildInitTrkList(Vec3 measure, double time, unsigned int id);

    //初始化轨迹使用
    void FrontendLoop();
    

    //获取上一帧状态
    void GetLastFrameInfo()
    {
        last_state_ = last_frame_->ObjState();
        last_timestamp_ = last_frame_->ObjTimestamp();
    };

    //插入关键帧
    bool InsertKeyFrame()
    {
        trk_list_->InsertKeyframe(cur_frame_);
    }

    //TODO：在设置因子图的时候，从trklist中取measure-list的数据，赋给edge
    void SetMeasurement();

    //更新关键帧的状态
    void Optimize(myTrkList::KeyframeType &keyframes);

    std::thread frontend_thread;
    std::atomic<bool> frontend_running_;

    std::mutex data_mutex_;
    std::condition_variable trk_list_update_;
    
    myFrame::Ptr last_frame_ = nullptr;
    myFrame::Ptr cur_frame_ = nullptr;

    myTrkList::Ptr trk_list_ = nullptr;
    unsigned long num_of_frames = 0;


    Vec6 last_state_;
    double last_timestamp_;
    // Vec3 obj_size_;

};


} // namespace mytrk




#endif
