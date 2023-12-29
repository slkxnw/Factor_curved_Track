# include<ros/ros.h>
#include "factor_curved_track/mybackend.h"
# include<unistd.h>

namespace mytrk
{

void myBackend::InitObj(std::vector<Vec9> &od_res, double time)
{
    // std::unique_lock<std::mutex> lck(data_lck_);
    Vec3 measure;
    Vec3 box_size;
    Vec6 kf_state;
    Vec8 ca_ekf_state;
    Vec6 ctra_ekf_state;
    Vec10 ab3d_kf_state;
    for (auto &od :od_res)
    {
        myFrontend::Ptr new_frontend = myFrontend::Ptr(new myFrontend);
        
        //ab3dmot-kf,x, y, z, theta, l, w, h, dx, dy, dz
        ab3d_kf_state << od[0], od[1], od[2], od[6], od[3], od[4], od[5], vel[0], vel[1], vel[2];
        new_frontend->BuildInitAB3D_KF(ab3d_kf_state);
        //KF,x,y,th,vx,vy,w
        kf_state << od[0], od[1], od[6], vel[0], vel[1], vel[2];
        new_frontend->BuildInitKF(kf_state, acc);
        //ca_ekf,x,y,th,vx,vy,w,ax,ay
        ca_ekf_state << od[0], od[1], od[6], vel[0], vel[1], vel[2], acc_value[0], acc_value[1];
        new_frontend->BuildInitCA_EKF(ca_ekf_state);
        //ctra_ekf,x,y,th,v,a,w
        double init_v = sqrt(vel[0] * vel[0] + vel[1] * vel[1]);
        double init_a = sqrt(acc_value[0] * acc_value[0] + acc_value[1] * acc_value[1]);
        ctra_ekf_state << od[0], od[1], od[6], init_v, init_a, vel[2];
        new_frontend->BuildInitCTRA_EKF(ctra_ekf_state);
        //因子图
        //x,y,theta
        measure << od[0], od[1], od[6];
        new_frontend->BuildInitTrkList(measure, time, num_of_obj);
        //size
        box_size << od[3], od[4], od[5];
        new_frontend->SetObjSize(box_size);
        //Z
        new_frontend->SetObjZ(od[2]);
        //当前帧观测角
        new_frontend->SetObjObsrvAgl(od[7]);
        //当前帧的置信度
        new_frontend->SetObjObsrvConf(od[8]);
        //hashmap插入方式
        obj_list_[num_of_obj] = new_frontend;
        // Vec6 state;
        // state << od[0], od[1], od[6], od[3], od[4], od[5];
        state_prediction_list_[num_of_obj] = od;
        state_cur_list_[num_of_obj] = od;
        // obj_id_list.push_back(num_of_obj);
        // objwithdet_id_list.push_back(num_of_obj);
        num_of_obj++;
    }
}

void myBackend::UpdateObjState(std::unordered_map<unsigned long, Vec9> &matches, double time)
{
    Vec3 measure;
    Vec3 box_size;
    // objwithdet_id_list.clear();
    for (auto &match :matches)
    {
        obj_list_[match.first]->ResetTimeSinceUpdate();
        //检测结果为 x,y,z,w,h,l,theta
        //观测数据，x,y,theta
        measure << match.second[0], match.second[1], match.second[6];
        obj_list_[match.first]->CreateMeasureFrame(measure, time);
        obj_list_[match.first]->InsertKeyFrame();
        //检测框大小
        box_size << match.second[3], match.second[4], match.second[5];
        obj_list_[match.first]->SetObjSize(box_size);
        //Z & 观测角 & 置信度
        obj_list_[match.first]->SetObjZ(match.second[2]);
        obj_list_[match.first]->SetObjObsrvAgl(match.second[7]);
        obj_list_[match.first]->SetObjObsrvConf(match.second[8]);
        
        //至少有7帧数据时，启动因子图优化,少于7帧的时候，使用kf优化
        //TODO 在这里启动多线程优化，而非在前段启动多线程优化
        //TODO 添加kf功能，当帧数较少的时候，使用kf来更新
        int num_of_kf = obj_list_[match.first]->GetTrklist()->GetKeyframeNum();
        ROS_INFO("Trere is %d kf in trk %d", num_of_kf, match.first);
        if(num_of_kf > kf_opt_thres)
        {
            obj_list_[match.first]->UpdateTrkList();
        }
        else
        {
            if (filter_type == 1)
                //使用CV+KF
                obj_list_[match.first]->UpdateTrkListKF();
            else if (filter_type == 2)
                //使用CA + EKF
                obj_list_[match.first]->UpdateTrkListCA_EKF();
            else if (filter_type == 0)
                //使用AB3DMOT方法
                obj_list_[match.first]->UpdateTrkListAB3D_KF();
            else if (filter_type == 3)
                //使用CTRA + EKF
                obj_list_[match.first]->UpdateTrkListCTRA_EKF();
        }
        //如果某个轨迹有了匹配，就在state_cur_list_加上它，用state_prediction_list_[match.first]做一个赋值，
        //后面获取当前状态的时候，会更新掉相关数据
        state_cur_list_[match.first] = state_prediction_list_[match.first];
        //获取完有检测的轨迹列表后，直接将它清空，在这里处理有检测的轨迹加上对应的
        // objwithdet_id_list.push_back(match.first);
    }
    // usleep(0.5);

}

void myBackend::StopObj(std::vector<unsigned long> dead_ids)
{
    for(auto &id : dead_ids)
    {
        // dead_obj_list.insert(make_pair(id, obj_list_[id]->GetTrklist()));
        obj_list_[id]->Stop();
        auto it = obj_list_.find(id);
        std::cout<<"delete trk_"<<id<<" at:"<<std::endl;
        std::cout<<obj_list_[id]->GetCurPosition().transpose()<<std::endl;
        obj_list_.erase(it);
        if(obj_list_.find(id) == obj_list_.end())
            std::cout<<"delete trk_"<<id<<" succeed!"<<std::endl;
        state_prediction_list_.erase(id);
        state_cur_list_.erase(id);
    }
}
       
void myBackend::RemoveUnmatchTrk(unsigned long id)
{
    state_cur_list_.erase(id);
}

myBackend::ObjInfotype myBackend::GetStatePrediction(double time)
{
    Vec3 position_prediction;
    Vec3 obj_size;
    obj_id_list.clear();
    for (auto &state_pair : state_prediction_list_)
    {
        //x,y,theta
        int num_of_kf = obj_list_[state_pair.first]->GetTrklist()->GetKeyframeNum();
        if(num_of_kf > kf_opt_thres)
        {
            position_prediction = obj_list_[state_pair.first]->PredictPostion(time);
        }
        else
        {
            if (filter_type == 1)
                //使用CV+KF
                position_prediction = obj_list_[state_pair.first]->PredictPostionKF(time);
            else if (filter_type == 2)
                //使用CA + EKF
                position_prediction = obj_list_[state_pair.first]->PredictPostionCA_EKF(time);
            else if (filter_type == 0)
                //使用AB3DMOT方法
                position_prediction = obj_list_[state_pair.first]->PredictPostionAB3D_KF(time);
            else if (filter_type == 3)
                //使用CA + EKF
                position_prediction = obj_list_[state_pair.first]->PredictPostionCTRA_EKF(time);
            
        }
        
        // state_pair.second.block<2, 1>(0, 0) = position_prediction;
        //state_pair.second: x,y,z,l,w,h,th,obsryagl,conf
        state_pair.second[0] = position_prediction[0];
        state_pair.second[1] = position_prediction[1];
        state_pair.second[6] = position_prediction[2];
        //size
        obj_size = obj_list_[state_pair.first]->GetObjSize();
        state_pair.second[3] = obj_size[0];
        state_pair.second[4] = obj_size[1];
        state_pair.second[5] = obj_size[2];
        //z & 观测角 & 检测置信度
        state_pair.second[2] = obj_list_[state_pair.first]->GetObjZ();
        state_pair.second[7] = obj_list_[state_pair.first]->GetObjObsrvAgl();
        state_pair.second[8] = obj_list_[state_pair.first]->GetObjObsrvConf();

        obj_id_list.push_back(state_pair.first);
    }
    
    return state_prediction_list_;
}

myBackend::ObjInfotype myBackend::GetStatePredictionAll(double time)
{
    Vec3 position_prediction;
    Vec3 obj_size;
    Vec9 preds;
    ObjInfotype prediction_list;
    obj_id_list.clear();
    for (auto &pair : obj_list_)
    {
        //x,y,theta
        int num_of_kf = obj_list_[pair.first]->GetTrklist()->GetKeyframeNum();
        if(num_of_kf > kf_opt_thres)
        {
            position_prediction = obj_list_[pair.first]->PredictPostion(time);
        }
        else
        {
            if (filter_type == 1)
                //使用CV+KF
                position_prediction = obj_list_[pair.first]->PredictPostionKF(time);
            else if (filter_type == 2)
                //使用CA + EKF
                position_prediction = obj_list_[pair.first]->PredictPostionCA_EKF(time);
            else if (filter_type == 0)
                //使用AB3DMOT方法
                position_prediction = obj_list_[pair.first]->PredictPostionAB3D_KF(time);
            else if (filter_type == 3)
                //使用CA + EKF
                position_prediction = obj_list_[pair.first]->PredictPostionCTRA_EKF(time);
            
        }
        
        // state_pair.second.block<2, 1>(0, 0) = position_prediction;
        //state_pair.second: x,y,z,l,w,h,th,obsryagl,conf
        preds[0] = position_prediction[0];
        preds[1] = position_prediction[1];
        preds[6] = position_prediction[2];
        //size
        obj_size = obj_list_[pair.first]->GetObjSize();
        preds[3] = obj_size[0];
        preds[4] = obj_size[1];
        preds[5] = obj_size[2];
        //z & 观测角 & 检测置信度
        preds[2] = obj_list_[pair.first]->GetObjZ();
        preds[7] = obj_list_[pair.first]->GetObjObsrvAgl();
        preds[8] = obj_list_[pair.first]->GetObjObsrvConf();

        obj_id_list.push_back(pair.first);
        prediction_list[pair.first] = preds;
    }
    
    return prediction_list;
}

myBackend::ObjInfotype myBackend::GetStateCur()
{
    Vec3 cur_position;
    Vec3 obj_size;
    // std::cout<<state_cur_list_.size()<<std::endl;
    for (auto &state_pair : state_cur_list_)
    {
        //x,y,theta
        cur_position = obj_list_[state_pair.first]->GetCurPosition();
        // state_pair.second.block<2, 1>(0, 0) = position_prediction;
        state_pair.second[0] = cur_position[0];
        state_pair.second[1] = cur_position[1];
        state_pair.second[6] = cur_position[2];
        //size
        obj_size = obj_list_[state_pair.first]->GetObjSize();
        state_pair.second[3] = obj_size[0];
        state_pair.second[4] = obj_size[1];
        state_pair.second[5] = obj_size[2];
        //z & 观测角 & 检测置信度
        state_pair.second[2] = obj_list_[state_pair.first]->GetObjZ();
        state_pair.second[7] = obj_list_[state_pair.first]->GetObjObsrvAgl();
        state_pair.second[8] = obj_list_[state_pair.first]->GetObjObsrvConf();
    }
    return state_cur_list_;
}

void myBackend::UpdateKFpred(std::unordered_map<unsigned long, Vec3> &new_pos)
{
    for(auto &pair : new_pos)
    {
        obj_list_[pair.first]->UpdateAB3D_KFPred(pair.second);
    }
}


} // namespace mytrk
