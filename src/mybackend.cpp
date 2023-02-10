#include "factor_curved_track/mybackend.h"

namespace mytrk
{

void myBackend::InitObj(std::vector<Vec7> &od_res, double time)
{
    std::unique_lock<std::mutex> lck(data_lck_);
    Vec3 measure;
    for (auto &od :od_res)
    {
        myFrontend::Ptr new_frontend = myFrontend::Ptr(new myFrontend);
        //x,y,theta
        measure << od[0], od[1], od[6];
        new_frontend->BuildInitTrkList(measure, time, num_of_obj);
        // TODO这里将插入pair改成常用的key-value插入方式
        //这样也不对
        obj_list_[num_of_obj] = new_frontend;
        Vec6 state;
        state << od[0], od[1], od[6], od[3], od[4], od[5];
        state_prediction_list_[num_of_obj] = od;
        num_of_obj++;
    }
}

void myBackend::UpdateObjState(std::unordered_map<unsigned long, Vec7> &matches, double time)
{
    Vec3 measure;
    for (auto &match :matches)
    {
        //检测结果为 x,y,z,w,h,l,theta
        measure << match.second[0], match.second[1], match.second[6];
        obj_list_[match.first]->CreateMeasureFrame(measure, time);
        obj_list_[match.first]->InsertKeyFrame();
        obj_list_[match.first]->UpdateTrkList();
    }
}

void myBackend::StopObj(std::vector<unsigned long> dead_ids)
{
    for(auto &id : dead_ids)
    {
        dead_obj_list.insert(make_pair(id, obj_list_[id]->GetTrklist()));
        obj_list_[id]->Stop();
        obj_list_.erase(id);
        state_prediction_list_.erase(id);
    }
}
       


myBackend::PredictObjtype myBackend::GetStatePrediction(double time)
{
    Vec3 position_prediction;
    obj_id_list.clear();
    //TODO 需要更新目标的尺寸，现在假设目标尺寸没变动过
    for (auto &state_pair : state_prediction_list_)
    {
        position_prediction = obj_list_[state_pair.first]->PredictPostion(time);
        // state_pair.second.block<2, 1>(0, 0) = position_prediction;
        state_pair.second[0] = position_prediction[0];
        state_pair.second[1] = position_prediction[1];
        state_pair.second[6] = position_prediction[2];
        obj_id_list.push_back(state_pair.first);
    }
    return state_prediction_list_;
}


} // namespace mytrk
