/**
* 这个节点管理backend，用于初始化节点和航迹管理
*/
# include<ros/ros.h>
# include<message_filters/subscriber.h>
# include<message_filters/synchronizer.h>
# include<message_filters/sync_policies/approximate_time.h>
# include"include/mybackend.h"
// # include"Factor_curved_track/Detection_list.h"
// # include"Factor_curved_Track/Pairs.h"
// # include"Factor_curved_Track/stampArray.h"

void processCallback(const Factor_curved_Track::Pairs &match_pair, const Factor_curved_Track::StampArray &unmatch_trk, 
                    const Factor_curved_Track::StampArray &unmatch_det, const Factor_curved_Track::Detection_list &dets,
                    mytrk::myBackend::Ptr backend)
{
    
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "factor_manager");

    ros::NodeHandle nh;

    mytrk::myBackend::Ptr backend_p = mytrk::myBackend::Ptr(new mytrk::myBackend);


    auto matched_pair_sub = message_filters::Subscriber<Factor_curved_Track::Pairs>(nh, "/matched_pair", 10);
    auto unmatched_trk_sub = message_filters::Subscriber<Factor_curved_Track::StampArray>(nh, "/unmatched_trk", 10);
    auto unmatched_det_sub = message_filters::Subscriber<Factor_curved_Track::StampArray>(nh, "/unmatched_det", 10);
    auto dets_sub = message_filters::Subscriber<Factor_curved_Track::Detection_list>(nh, "/detections", 10);

    typedef message_filters::sync_policies::ApproximateTime<Factor_curved_Track::Pairs, Factor_curved_Track::StampArray, 
    Factor_curved_Track::StampArray, Factor_curved_Track::Detection_list> MySynPolicy;
    
    message_filters::Synchronizer<MySynPolicy> sync(matched_pair_sub, unmatched_trk_sub, unmatched_det_sub, dets_sub);

    sync.registerCallback(boost::bind(&processCallback, _1, _2, _3, _4, backend_p));

    ros::spin();

    return 0;


}