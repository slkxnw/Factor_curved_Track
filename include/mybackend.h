#pragma once
#ifndef MYTRK_BACKEND_H
#define MYTRK_BACKEND_H

#include "mycommon_include.h"
#include "myfrontend.h"

namespace mytrk
{
class myFrontend;

class myBackend
{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::unique_ptr<myBackend> Ptr;
    typedef std::unordered_map<unsigned long, myFrontend::Ptr> Frontendtype;
    typedef std::unordered_map<unsigned long, Vec6> PredictObjtype;

    //和航迹管理有关
    //生成新航迹
    void InitObj(std::vector<Vec7> od_res);
    //合并航迹
    void ConcatObj(unsigned long old_obj_id, unsigned long new_obj_id);
    //删除航迹
    void StopObj(unsigned long id);

    //和数据匹配结果相关
    //将检测结果添加到对应的frontend，并且更新
    void UpdateObjState(std::unordered_map<unsigned long, Vec7> matches);

    //给出，给定时刻的目标位置预测结果
    PredictObjtype GetStatePrediction(double time);


private:

    unsigned long num_of_obj = 0;

    Frontendtype obj_list_;
    PredictObjtype state_prediction_list_;


};


} // namespace mytrk





#endif