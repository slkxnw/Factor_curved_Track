#include "mybackend.h"

namespace mytrk
{

void myBackend::InitObj(std::vector<Vec7> &od_res, double time)
{
    Vec3 measure;
    for (auto &od :od_res)
    {
        myFrontend::Ptr new_frontend = myFrontend::Ptr(new myFrontend);
        //x,y,theta
        measure << od[0], od[1], od[6];
        new_frontend->BuildInitTrkList(measure, time, num_of_obj);
        obj_list_.insert(make_pair(num_of_obj, new_frontend));
        Vec6 state;
        state << od[0], od[1], od[6], od[3], od[4], od[5];
        state_prediction_list_[num_of_obj] = state;
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
    for (auto &state_pair : state_prediction_list_)
    {
        position_prediction = obj_list_[state_pair.first]->PredictPostion(time);
        state_pair.second.block<3, 1>(0, 0) = position_prediction;
    }
    return state_prediction_list_;
}


} // namespace mytrk
