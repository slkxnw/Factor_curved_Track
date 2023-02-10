/**
* 这个节点管理backend，用于初始化节点和航迹管理
*/
# include<ros/ros.h>
# include<std_msgs/UInt8MultiArray.h>
# include<message_filters/subscriber.h>
# include<message_filters/synchronizer.h>
# include<message_filters/sync_policies/approximate_time.h>
# include"factor_curved_track/mybackend.h"
# include"track_msgs/Detection_list.h"
# include"track_msgs/Detection.h"
# include"track_msgs/Pairs.h"
# include"track_msgs/StampArray.h"

void processCallback(const track_msgs::Pairs &match_pair, const track_msgs::StampArray &unmatch_trk, 
                    const track_msgs::StampArray &unmatch_det, const track_msgs::Detection_list &dets,
                    const mytrk::myBackend::Ptr backend, const ros::Publisher &trk_predict_pub)
{
    double time = dets.header.stamp.sec * 0.1;
    std::unordered_map<unsigned long, Vec7> matches;
    std::vector<unsigned long> dead_ids;
    std::vector<Vec7> od_res;
    Vec7 det;
    unsigned long backend_id;
    //更新匹配到的轨迹
    //TODO 这里有问题：match_pair中trk的id是从0开始的，对应的是上一次发布的trkpred的顺序，不是对应的backend中的obj_id，已经修改了一下
    //需要修改一下,发布的trk信息的顺序，和后端当前的objlist中（obj_id, frontend_ptr）对的循序一致
    //因此，首先获取后端当前的objlist，然后取match_pair中第i对匹配中，trk对应的id，取objlist中这个id对应的frontend_ptr，
    //再从frontend-ptr获取它在后端中的分配到的objid
    //实际上，objlist是哈希表，我们希望获取它的第i个k-v对，并得到他的key值

    //
    for(int i = 0; i < match_pair.trk.data.size(); ++i)
    {
        det << dets.detecs[i].pos.x, dets.detecs[i].pos.y, dets.detecs[i].pos.z, 
                dets.detecs[i].siz.x, dets.detecs[i].siz.y, dets.detecs[i].siz.z, dets.detecs[i].alp;
        backend_id = backend->GetObjlist()[int(match_pair.trk.data[i])]->GetTrkId();
        // auto hash_ptr = backend->GetObjlist().begin();
        // hash_ptr
        matches[backend_id] = det;
    }
    backend->UpdateObjState(matches, time);
    //删除老旧轨迹
    for(auto & id : unmatch_trk.ids.data)
    {
        if((time - backend->GetObjlist()[id]->GetLastfeame()->time_stamp_) > 0.15)
            dead_ids.push_back(id);
    }
    backend->StopObj(dead_ids);
    //初始化新轨迹
    for(auto & id : unmatch_det.ids.data)
    {
        det << dets.detecs[id].pos.x, dets.detecs[id].pos.y, dets.detecs[id].pos.z, 
                dets.detecs[id].siz.x, dets.detecs[id].siz.y, dets.detecs[id].siz.z, dets.detecs[id].alp;
        od_res.push_back(det);
    }
    backend->InitObj(od_res, time);

    //
    //按照常理，trk的预测应该是检测结果时刻的，但是在AB3DMOT中，检测的时刻是如何定义的，原始kitti数据中似乎没有给定时刻
    //看一下ab3dmot的代码：按照kitti的10Hz频率搞的
    //TODO :目前是测试代码，预测固定时间间隔后的trk状态，实际上要改成预测检测结果时刻的trk状态
    auto trk_pred = backend->GetStatePrediction(time + 0.1);

    track_msgs::Detection trk_;
    for(auto &pair : trk_pred)
    {
        trk
    }


}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "factor_manager");

    ros::NodeHandle nh;

    auto trk_predict_pub = nh.advertise<track_msgs::Detection_list>("/tracks", 10);

    mytrk::myBackend::Ptr backend_p = mytrk::myBackend::Ptr(new mytrk::myBackend);


    message_filters::Subscriber<track_msgs::Pairs> matched_pair_sub(nh, "/matched_pair", 10);
    message_filters::Subscriber<track_msgs::StampArray> unmatched_trk_sub(nh, "/unmatched_trk", 10);
    message_filters::Subscriber<track_msgs::StampArray> unmatched_det_sub(nh, "/unmatched_det", 10);
    message_filters::Subscriber<track_msgs::Detection_list> dets_sub(nh, "/detections", 10);

    typedef message_filters::sync_policies::ApproximateTime<track_msgs::Pairs, track_msgs::StampArray, 
    track_msgs::StampArray, track_msgs::Detection_list> MySynPolicy;
    
    message_filters::Synchronizer<MySynPolicy> sync(matched_pair_sub, unmatched_trk_sub, unmatched_det_sub, dets_sub);

    sync.registerCallback(boost::bind(&processCallback, _1, _2, _3, _4, &backend_p, trk_predict_pub));

    ros::spin();

    return 0;


}