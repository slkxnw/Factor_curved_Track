/**
* 这个节点管理backend，用于初始化节点和航迹管理
*/
# include<ros/ros.h>
# include<message_filters/subscriber.h>
# include<message_filters/synchronizer.h>
# include<message_filters/sync_policies/approximate_time.h>
# include"include/mybackend.h"
# include"track_msgs/Detection_list.h"
# include"track_msgs/Pairs.h"
# include"track_msgs/StampArray.h"

void processCallback(const track_msgs::Pairs &match_pair, const track_msgs::StampArray &unmatch_trk, 
                    const track_msgs::StampArray &unmatch_det, const track_msgs::Detection_list &dets,
                    mytrk::myBackend::Ptr backend, const ros::Publisher &trk_predict_pub)
{
    double time = dets.header.stamp.sec * 0.01;
    std::unordered_map<unsigned long, Vec7> matches;
    std::vector<unsigned long> dead_ids;
    std::vector<Vec7> od_res;
    Vec7 det;
    //更新匹配到的轨迹
    for(int i = 0; i < match_pair.trk.data.size(); ++i)
    {
        det << dets.detecs[i].pos.x, dets.detecs[i].pos.y, dets.detecs[i].pos.z, 
                dets.detecs[i].siz.x, dets.detecs[i].siz.y, dets.detecs[i].siz.z, dets.detecs[i].alp;
        matches[match_pair.trk.data[i]] = det;
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

    //TODO：发布trk预测结果，问题在于
    //按照常理，trk的预测应该是检测结果时刻的，但是在AB3DMOT中，检测的时刻是如何定义的，原始kitti数据中似乎没有给定时刻
    //看一下ab3dmot的代码
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