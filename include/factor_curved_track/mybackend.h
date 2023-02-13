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
    void InitObj(std::vector<Vec8> &od_res, double time);
    //合并航迹
    void ConcatObj(unsigned long old_obj_id, unsigned long new_obj_id)
    {
        std::shared_ptr<myTrkList> new_trk_list = obj_list_[new_obj_id]->GetTrklist();
        obj_list_[old_obj_id]->ConcatTrklist(new_trk_list);
    }
    //删除航迹
    void StopObj(std::vector<unsigned long> dead_ids);


    //和数据匹配结果相关
    //将检测结果添加到对应的frontend，并且更新
    void UpdateObjState(std::unordered_map<unsigned long, Vec8> &matches, double time);

    //给出，给定时刻的目标位置预测结果
    ObjInfotype GetStatePrediction(double time);
    //获取更新后的轨迹当前状态，
    ObjInfotype GetStateCur();

    std::vector<unsigned long> GetObjIDlist()
    {
        std::unique_lock<std::mutex> lck(data_lck_);
        return obj_id_list;
    }

    Frontendtype GetObjlist()
    {
        std::unique_lock<std::mutex> lck(data_lck_);
        return obj_list_;
    }

private:

    unsigned long num_of_obj = 0;
    std::mutex data_lck_;

    Frontendtype obj_list_;
    TrkListType dead_obj_list;
    ObjInfotype state_prediction_list_;
    ObjInfotype state_cur_list_;
    std::vector<unsigned long> obj_id_list;

};


} // namespace mytrk





#endif