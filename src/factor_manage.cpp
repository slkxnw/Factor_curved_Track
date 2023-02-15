/**
* 这个节点管理backend，用于初始化节点和航迹管理
*/
# include<ros/ros.h>
# include<ros/subscriber.h>
# include<ros/publisher.h>
# include<std_msgs/UInt8MultiArray.h>
# include<message_filters/subscriber.h>
# include<message_filters/synchronizer.h>
# include<message_filters/sync_policies/approximate_time.h>
# include<message_filters/sync_policies/exact_time.h>
# include"factor_curved_track/mybackend.h"
# include"factor_curved_track/myframe.h"
# include"factor_curved_track/myfrontend.h"
# include"factor_curved_track/mytrk_list.h"
# include"track_msgs/Detection_list.h"
# include"track_msgs/Detection.h"
# include"track_msgs/Pairs.h"
# include"track_msgs/StampArray.h"
# include"track_msgs/Trk_pred.h"
# include"track_msgs/Trk_update.h"
# include"track_msgs/Trk_state_store.h"

// 一些全局变量，这样不用向回调函数传参
ros::Publisher trk_predict_pub;
ros::Publisher trk_cur_pub;
ros::Publisher trk_id_pub;
mytrk::myBackend backend;
ros::ServiceClient trk_store;

//可以将1，3，4结合到一起
// void callback(const track_msgs::PairsConstPtr &match_pair, 
//                     const track_msgs::StampArrayConstPtr &unmatch_trk, 
//                     const track_msgs::Detection_listConstPtr &dets, 
//                     const track_msgs::StampArrayConstPtr &unmatch_det)
// {
//     double time = dets->header.stamp.sec * 0.1;
//     int delet_cnt = 0;
//     std::unordered_map<unsigned long, Vec9> matches;
//     std::vector<unsigned long> dead_ids;
//     std::vector<Vec9> od_res;
//     Vec9 det;
//     unsigned long backend_id;
//     track_msgs::Detection trk_;
//     track_msgs::Information info_;
//     track_msgs::Detection_list trks_pred;
//     track_msgs::Detection_list trks_cur;
//     track_msgs::StampArray active_ids;
//     //更新匹配到的轨迹    
//     //需要修改一下,发布的trk信息的顺序，和后端当前的objlist中（obj_id, frontend_ptr）对的循序一致
//     //因此，首先获取后端当前的objlist，然后得到match_pair中第i对匹配中，trk对应的id，用[]操作符取objlist中这个id对应的frontend_ptr，
//     //再从frontend-ptr获取它在后端中的分配到的obj_id
//     //不对，[]操作符是按照key取值
//     //最后还是在backend里维护一个objid列表，每次预测位置的时候更新这个列表
//     //TODO 使用检测结果单独更新目标的z和bbox信息,以及将当前帧坐标系下的观测角
//     //TODO 确认下面这些id匹配是否有问题
//     //TODO 确认坐标系，看了kittidevkit，z轴是向前的，和在因子图后端定义的不一样，因此
//     //根据det更新和初始化trk时，需要交换一下位置dets的y和z的位置，发布从后端trk获取到的位置时，也要交换y和z的位置
//     for(int i = 0; i < match_pair->trk.data.size(); ++i)
//     {
//         det << dets->detecs[i].pos.x, dets->detecs[i].pos.z, dets->detecs[i].pos.y, 
//             dets->detecs[i].siz.x, dets->detecs[i].siz.y, dets->detecs[i].siz.z, 
//             double(dets->detecs[i].alp), dets->infos[i].orin, dets->infos[i].score;
//         backend_id = backend->GetObjIDlist()[int(match_pair->trk.data[i])];
//         // auto hash_ptr = backend->GetObjlist().at(5);
//         // hash_ptr
//         matches[backend_id] = det;
//     }
//     backend->UpdateObjState(matches, time);
//     //删除老旧轨迹
//     for(int i= 0; i < unmatch_trk->ids.data.size(); ++i)
//     {
//         backend_id = backend->GetObjIDlist()[int(unmatch_trk->ids.data[i])];
//         if((time - backend->GetObjlist()[backend_id]->GetLastfeame()->time_stamp_) > 1.5)
//             dead_ids.push_back(backend_id);      
//     }
//     backend->StopObj(dead_ids);
//     //初始化新轨迹
//     for(auto & id : unmatch_det->ids.data)
//     {
//         det << dets->detecs[id].pos.x, dets->detecs[id].pos.z, dets->detecs[id].pos.y, 
//                 dets->detecs[id].siz.x, dets->detecs[id].siz.y, dets->detecs[id].siz.z, 
//                 dets->detecs[id].alp, dets->infos[id].orin, dets->infos[id].score;
//         od_res.push_back(det);
//     }
//     backend->InitObj(od_res, time);
//     //
//     //按照常理，trk的预测应该是检测结果时刻的，但是在AB3DMOT中，检测的时刻是如何定义的，原始kitti数据中似乎没有给定时刻
//     //看一下ab3dmot的代码：按照kitti的10Hz频率搞的
//     //TODO :目前是测试代码，预测固定时间间隔后的trk状态，实际上要改成预测检测结果时刻的trk状态
//     auto trk_state_pred = backend->GetStatePrediction(time + 0.1);
//     //将预测结果按照顺序，生成trk并放入列表中
//     trks_pred.header.stamp.sec = dets->header.stamp.sec + 1;
//     for(auto &pair : trk_state_pred)
//     {
//         trk_.pos.x = pair.second[0];
//         //因子图坐标系和kitti坐标系不一样
//         trk_.pos.z = pair.second[1];
//         trk_.pos.y = pair.second[2];
//         trk_.siz.x = pair.second[3];
//         trk_.siz.y = pair.second[4];
//         trk_.siz.z = pair.second[5];
//         trk_.alp = pair.second[6];
//         trks_pred.detecs.push_back(trk_);
//         //这里是当前帧的观测角
//         info_.orin = pair.second[7];
//         info_.type = 0;
//         info_.score = pair.second[8];
//         trks_pred.infos.push_back(info_);
//     }
//     trk_predict_pub.publish(trks_pred);
//     //发布轨迹当前状态，包括观测角/z等数据,这里的观测角/z和上面轨迹预测状态的是一样的，都使用最近的检测数据的参数
//     auto trks_state_cur = backend->GetStateCur();
//     trks_cur.header = dets->header;
//     for(auto &pair : trks_state_cur)
//     {
//         trk_.pos.x = pair.second[0];
//         //因子图坐标系和kitti坐标系不一样
//         trk_.pos.z = pair.second[1];
//         trk_.pos.y = pair.second[2];
//         trk_.siz.x = pair.second[3];
//         trk_.siz.y = pair.second[4];
//         trk_.siz.z = pair.second[5];
//         trk_.alp = pair.second[6];
//         trks_cur.detecs.push_back(trk_);
//         //这里是当前帧的观测角
//         info_.orin = pair.second[7];
//         info_.type = 0;
//         info_.score = pair.second[8];
//         trks_cur.infos.push_back(info_);
//     }
//     trk_cur_pub.publish(trks_cur);
//     //发布活跃轨迹id
//     auto obj_ids = backend->GetObjIDlist();
//     active_ids.header = dets->header;
//     for(auto &id : obj_ids)
//         active_ids.ids.data.push_back(id);
//     trk_id_pub.publish(active_ids);
//     if(int(dets->header.stamp.sec) % 5 == 0)
//         ROS_INFO("Frame %d optimization finished with %d trks updated, %d trks initialed, %d trks deleted",
//                 int(dets->header.stamp.sec), matches.size(), od_res.size(), dead_ids.size());
// }

bool predict_callback(track_msgs::Trk_pred::Request &request, track_msgs::Trk_pred::Response &response)
{
    track_msgs::Detection_list trks_pred;
    track_msgs::Detection trk_;
    track_msgs::Information info_;
    auto trk_state_pred = backend.GetStatePrediction(request.pred_time);
    ROS_INFO("there is %d trks", trk_state_pred.size());
    //将预测结果按照顺序，生成trk并放入列表中
    trks_pred.header.stamp.sec = request.pred_time * 10;
    for(auto &pair : trk_state_pred)
    {
        trk_.pos.x = pair.second[0];
        //因子图坐标系和kitti坐标系不一样
        trk_.pos.z = pair.second[1];
        trk_.pos.y = pair.second[2];
        trk_.siz.x = pair.second[3];
        trk_.siz.y = pair.second[4];
        trk_.siz.z = pair.second[5];
        trk_.alp = pair.second[6];

        trks_pred.detecs.push_back(trk_);
        //这里是当前帧的观测角
        info_.orin = pair.second[7];
        info_.type = 0;
        info_.score = pair.second[8];
        trks_pred.infos.push_back(info_);
    }
    response.trk_predicts = trks_pred;

    return true;
}

bool update_callback(track_msgs::Trk_update::Request& request, track_msgs::Trk_update::Response& response)
{
    //TODO 服务传入的请求信息好像不能是指针，因此下面的赋值会很消耗时间和空间，两个想法，一个是不赋值了，一个是取指针，但是原始指针
    //目前使用不赋值
    // track_msgs::Pairs match_pair = request.matches;
    // track_msgs::StampArray unmatch_dets = request.unmatch_dets;
    // track_msgs::StampArray unmatch_trks = request.unmatch_trks;
    // track_msgs::Detection_list dets = request.dets;

    double time = request.dets.header.stamp.sec * 0.1;
    std::unordered_map<unsigned long, Vec9> matches;
    std::vector<unsigned long> dead_ids;
    std::vector<Vec9> od_res;
    Vec9 det;
    unsigned long backend_id;
    track_msgs::Detection trk_;
    track_msgs::Information info_;
    track_msgs::Detection_list trks_pred;
    track_msgs::Detection_list trks_cur;
    track_msgs::StampArray active_ids;

    //更新匹配到的轨迹
    
    //需要修改一下,发布的trk信息的顺序，和后端当前的objlist中（obj_id, frontend_ptr）对的循序一致
    //在backend里维护一个objid列表，每次预测位置的时候更新这个列表
    //TODO 确认下面这些id匹配是否有问题
    //TODO 确认坐标系，看了kittidevkit，z轴是向前的，和在因子图后端定义的不一样，因此
    //根据det更新和初始化trk时，需要交换一下位置dets的y和z的位置，发布从后端trk获取到的位置时，也要交换y和z的位置
    //TODO 是否会出现，还没有更新完，就发布的情况，也就是说，目前没有机制，使得更新完之前，不能从trk-list里获取最新状态
    for(int i = 0; i < request.matches.trk.data.size(); ++i)
    {
        det << request.dets.detecs[i].pos.x, request.dets.detecs[i].pos.z, request.dets.detecs[i].pos.y, 
            request.dets.detecs[i].siz.x, request.dets.detecs[i].siz.y, request.dets.detecs[i].siz.z, 
            double(request.dets.detecs[i].alp), request.dets.infos[i].orin, request.dets.infos[i].score;
        backend_id = backend.GetObjIDlist()[int(request.matches.trk.data[i])];
        matches[backend_id] = det;
    }
    backend.UpdateObjState(matches, time);
    //删除老旧轨迹
    for(int i= 0; i < request.unmatch_trks.ids.data.size(); ++i)
    {
        backend_id = backend.GetObjIDlist()[int(request.unmatch_trks.ids.data[i])];
        if((time - backend.GetObjlist()[backend_id]->GetLastfeame()->time_stamp_) > 1.5)
            dead_ids.push_back(backend_id);
       
    }
    backend.StopObj(dead_ids);
    //初始化新轨迹
    for(auto & id : request.unmatch_dets.ids.data)
    {
        det << request.dets.detecs[id].pos.x, request.dets.detecs[id].pos.z, request.dets.detecs[id].pos.y, 
            request.dets.detecs[id].siz.x, request.dets.detecs[id].siz.y, request.dets.detecs[id].siz.z, 
            double(request.dets.detecs[id].alp), request.dets.infos[id].orin, request.dets.infos[id].score;
        od_res.push_back(det);
    }
    backend.InitObj(od_res, time);
    ROS_INFO("Update success!");

    //发布轨迹当前状态，包括观测角/z等数据,这里的观测角/z和上面轨迹预测状态的是一样的，都使用最近的检测数据的参数
    auto trks_state_cur = backend.GetStateCur();
    trks_cur.header = request.dets.header;
    for(auto &pair : trks_state_cur)
    {
        trk_.pos.x = pair.second[0];
        //因子图坐标系和kitti坐标系不一样
        trk_.pos.z = pair.second[1];
        trk_.pos.y = pair.second[2];
        trk_.siz.x = pair.second[3];
        trk_.siz.y = pair.second[4];
        trk_.siz.z = pair.second[5];
        trk_.alp = pair.second[6];

        trks_cur.detecs.push_back(trk_);
        //这里是当前帧的观测角
        info_.orin = pair.second[7];
        info_.type = 0;
        info_.score = pair.second[8];
        trks_cur.infos.push_back(info_);
    }

    //发布活跃轨迹id
    auto obj_ids = backend.GetObjIDlist();
    active_ids.header = request.dets.header;
    for(auto &id : obj_ids)
        active_ids.ids.data.push_back(id);
    
    track_msgs::Trk_state_store srv;
    srv.request.detecs = trks_cur.detecs;
    srv.request.infos = trks_cur.infos;
    srv.request.header = trks_cur.header;
    srv.request.ids = active_ids.ids;

    trk_store.call(srv);

    if(int(request.dets.header.stamp.sec) % 1 == 0)
        ROS_INFO("Frame %d optimization finished with %d trks updated, %d trks initialed, %d trks deleted, and state save %d",
                int(request.dets.header.stamp.sec), matches.size(), od_res.size(), dead_ids.size(), int(srv.response.success));

    return true;
}

int main(int argc, char** argv)
{
    //TODO 添加Log信息
    // std::cout<<"test"<<std::endl;
    ros::init(argc, argv, "factor_manager");

    ros::NodeHandle nh;
    std::vector<Vec9> init_bd;
    mytrk::myBackend backend;

    ros::service::waitForService("/trk_state_store");
    ros::ServiceClient trk_store = nh.serviceClient<track_msgs::Trk_state_store>("/trk_state_store");
    //轨迹预测服务
    ros::ServiceServer trk_predict = nh.advertiseService("/trk_predict", predict_callback);
    //轨迹更新服务
    ros::ServiceServer trk_update = nh.advertiseService("/trk_update", update_callback);
    //因子管理后端
    // init_bd.push_back(Vec9::Zero());
    
    // backend->InitObj(init_bd, 0);

    // //发布trks预测结果
    // ros::Publisher trk_predict_pub = nh.advertise<track_msgs::Detection_list>("/tracks_prediction", 10);
    // //发布trks当前状态
    // ros::Publisher trk_cur_pub = nh.advertise<track_msgs::Detection_list>("/tracks_cur_state", 10);
    // //发布trks的id
    // ros::Publisher trk_id_pub = nh.advertise<track_msgs::StampArray>("/tracks_ids", 10);

    


    ros::spin();

    return 0;
}