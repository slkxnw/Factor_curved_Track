#pragma once
#ifndef MYTRK_BACKEND_H
#define MYTRK_BACKEND_H

#include "mycommon_include.h"
#include "myfrontend.h"
#include "mytrk_list.h"

namespace mytrk
{
class myFrontend;
class myTrkList;

class myBackend
{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<myBackend> Ptr;
    typedef std::unordered_map<unsigned long, std::shared_ptr<myFrontend> > Frontendtype;
    typedef std::unordered_map<unsigned long, Vec9> ObjInfotype;
    typedef std::unordered_map<unsigned long, std::shared_ptr<myTrkList> > TrkListType;

    //和航迹管理有关
    //生成新航迹
    void InitObj(std::vector<Vec9> &od_res, double time);
    //合并航迹
    void ConcatObj(unsigned long old_obj_id, unsigned long new_obj_id)
    {
        std::shared_ptr<myTrkList> new_trk_list = obj_list_[new_obj_id]->GetTrklist();
        obj_list_[old_obj_id]->ConcatTrklist(new_trk_list);
    }
    //删除航迹
    void StopObj(std::vector<unsigned long> dead_ids);

    void RemoveUnmatchTrk(unsigned long id);


    //和数据匹配结果相关
    //将检测结果添加到对应的frontend，并且更新
    void UpdateObjState(std::unordered_map<unsigned long, Vec9> &matches, double time);

    //给出，给定时刻的目标位置预测结果
    ObjInfotype GetStatePrediction(double time);
    ObjInfotype GetStatePredictionAll(double time);
    //更新KF预测结果
    void UpdateKFpred(std::unordered_map<unsigned long, Vec3> &new_pos);
    //获取更新后的轨迹当前状态，
    ObjInfotype GetStateCur();

    std::vector<unsigned long> GetObjIDlist()
    {
        // std::unique_lock<std::mutex> lck(data_lck_);
        return obj_id_list;
    }
    //无用，输出当前有匹配检测的轨迹
    // std::vector<unsigned long> GetObjwithdetIDlist()
    // {
    //     // std::unique_lock<std::mutex> lck(data_lck_);
    //     std::vector<unsigned long> new_list;
    //     new_list.clear();
    //     new_list.swap(objwithdet_id_list);
    //     return new_list;
    // }

    Frontendtype GetObjlist()
    {
        // std::unique_lock<std::mutex> lck(data_lck_);
        return obj_list_;
    }

private:

    unsigned long num_of_obj = 0;
    //滤波和优化切换
    int kf_opt_thres = 100;
    //0,AB3dmot;1, CV_KF; 2, CA_EKF
    //后面改用enum
    int filter_type = 0;
    std::mutex data_lck_;

    Frontendtype obj_list_;
    // TrkListType dead_obj_list;
    ObjInfotype state_prediction_list_;
    //当前帧所有有检测匹配的轨迹id列表，以及他们更新后的状态
    ObjInfotype state_cur_list_;
    //所有活跃轨迹的id列表
    std::vector<unsigned long> obj_id_list;
    //所有有检测匹配的轨迹id列表，实际上无用
    // std::vector<unsigned long> objwithdet_id_list;
    //KF初始状态假设,x,y,th速度,和加速度以及方差
    Vec3 vel = {0.0, 0.0, 0.0};
    //加速度方差
    Vec3 acc = {0.5, 0.1, 0.05};
    //加速度
    Vec2 acc_value = {0.0, 0.0};

};


} // namespace mytrk





#endif