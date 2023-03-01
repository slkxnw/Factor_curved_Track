#pragma once

#ifndef MYTRK_FRONTEND_H
#define MYTRK_FRONTEND_H

#include "mycommon_include.h"
#include "myframe.h"
#include "mytrk_list.h"
#include "myKF.h"
#include "myEKF.h"

namespace mytrk
{
struct myFrame;
class myTrkList;
class CV_KF;

//结合检测结果，结合上一针的状态，初始化Frame，并分配给map，然后构建一个因子图，进行优化
class myFrontend
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    //TODO 确定shared_ptr和unique_ptr的区别
    typedef std::shared_ptr<myFrontend> Ptr;
    // typedef std::vector<myTrkList::Ptr> TrkListType;

    //构造函数中，启动循环，并且挂起
    myFrontend();

    //航迹管理相关，初始化放在private里

    //初始化轨迹,设定初始车速为0，初始加速度和横摆角速度为0，根据匹配的观测结果和分配的目标id，初始化轨迹
    //TODO：确定合适的初始车速，之前取14，以便于符合匝道工况，但后来想了想，还是取为0，这样，匹配错误会少一点？
    bool BuildInitTrkList(Vec3 measure, double time, unsigned int id);
    bool BuildInitKF(Vec6 state, Vec3 vars);
    bool BuildInitCA_EKF(Vec8 state);
    bool BuildInitAB3D_KF(Vec10 state);

    //合并两个轨迹
    void ConcatTrklist(std::shared_ptr<myTrkList> new_trk_list);

    //提取出本frontend中的trklist
    std::shared_ptr<myTrkList> GetTrklist()
    {
        // std::unique_lock<std::mutex> lock(data_mutex_);
        return trk_list_;
    }

    //删除轨迹
    void ClearTrklist();

    //设置轨迹的id
    void SetTrkId(unsigned long id)
    {
        trk_list_->SetObjID(id);
    }

    int GetTrkId()
    {
        return trk_list_->GetObjID();
    }

    //结束该线程
    void Stop();

    //和frame相关操作

    //插入关键帧
    bool InsertKeyFrame()
    {
        trk_list_->InsertKeyframe(cur_frame_);
        return true;
    }

    //基于观测和上一帧数据，确定帧状态,同时更新teklist的帧数量信息
    myFrame::Ptr CreateMeasureFrame(Vec3 measure, double time, bool is_measure = true);

    //基于最近一次观测帧数据，外推生成本帧状态,同时更新teklist的帧数量信息
    myFrame::Ptr CreateExtrpFrame(double time);
    
    //优化相关的操作

    //有了新的观测，触发优化,在外部使用这个frontend类的地方调用这个函数
    void UpdateTrkList();

    void UpdateTrkListKF();
    void UpdateTrkListCA_EKF();
    void UpdateTrkListAB3D_KF();

    void UpdateAB3D_KFPred(Vec3 new_pos)
    {
        ab3d_kf_->SetState(new_pos);
    }


    //TODO：得到边的测量，并把数据添加给measure——list
    //TODO：设计一个measurelist，他是tek-list的一部分，存放这里计算的结果
    Vec6 CalEdgeMeasure();

    //数据关联相关操作
    //返回预测时刻的 x y theta
    Vec3 PredictPostion(double time);

    Vec3 PredictPostionKF(double time);
    Vec3 PredictPostionCA_EKF(double time);
    Vec3 PredictPostionAB3D_KF(double time);
    //
    Vec6 PredictState(double time);

    myFrame::Ptr GetLastfeame()
    {
        return last_frame_;
    }
    double GetLaststamp()
    {
        return last_timestamp_;
    }
    int GetDetectedTime()
    {
        return detected_time;
    }
    //评估相关，将当前状态输出
    //不参与更新的目标参数，的初始化和更新
    void SetObjSize(Vec3 size)
    {
        obj_size_ = size;
    }
    Vec3 GetObjSize()
    {
        return obj_size_;
    }
    void SetObjZ(double z)
    {
        z_ = z;
    }
    double GetObjZ()
    {
        return z_;
    }
    void SetObjObsrvAgl(double agl)
    {
        observ_agl= agl;
    }
    double GetObjObsrvAgl()
    {
        return observ_agl;
    }
    void SetObjObsrvConf(double conf)
    {
        observ_conf= conf;
    }
    double GetObjObsrvConf()
    {
        return observ_conf;
    }
    Vec3 GetCurPosition();


private:

    //初始化轨迹使用
    void FrontendLoop();
    

    //获取上一帧状态
    void GetLastFrameInfo()
    {
        if(last_frame_ != nullptr){
            last_state_ = last_frame_->ObjState();
            last_timestamp_ = last_frame_->ObjTimestamp();
        }
    };

    //TODO：在设置因子图的时候，从trklist中取measure-list的数据，赋给edge
    void SetMeasurement();

    //更新关键帧的状态,参考slam14讲，利用g2o和李代数进行位姿图优化的代码
    void Optimize(myTrkList::KeyframeType &keyframes, std::vector<int> &kf_ids);

    std::thread frontend_thread;
    std::atomic<bool> frontend_running_;

    std::mutex data_mutex_;
    std::condition_variable trk_list_update_;
    
    myFrame::Ptr last_frame_ = nullptr;
    myFrame::Ptr cur_frame_ = nullptr;

    std::shared_ptr<myTrkList> trk_list_ = nullptr;
    std::shared_ptr<CV_KF> kf_ = nullptr;
    std::shared_ptr<CA_EKF> ca_ekf_ = nullptr;
    std::shared_ptr<AB3D_KF> ab3d_kf_ = nullptr;   
    unsigned long num_of_frames = 0;


    Vec6 last_state_;
    double last_timestamp_;
    //目标其他参数，不参与更新，使用观测给出的数据
    Vec3 obj_size_;
    double z_;
    double observ_agl;
    double observ_conf;
    int detected_time = 1;

};


} // namespace mytrk




#endif
