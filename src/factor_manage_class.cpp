/**
* 这个节点管理backend，用于初始化节点和航迹管理,使用类封装
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
# include"geometry_msgs/AccelStamped.h"
# include"geometry_msgs/PointStamped.h"

void processCallback(const geometry_msgs::AccelStampedPtr)
{
    
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "factor_manager");

    ros::NodeHandle nh;

    //发布trks预测结果
    auto trk_predict_pub = nh.advertise<track_msgs::Detection_list>("/tracks", 10);

    mytrk::myBackend::Ptr backend_p = mytrk::myBackend::Ptr(new mytrk::myBackend);


    message_filters::Subscriber<track_msgs::Pairs> matched_pair_sub(nh, "/matched_pair", 10);
    message_filters::Subscriber<track_msgs::StampArray> unmatched_trk_sub(nh, "/unmatched_trk", 10);
    message_filters::Subscriber<track_msgs::StampArray> unmatched_det_sub(nh, "/unmatched_det", 10);
    message_filters::Subscriber<track_msgs::Detection_list> dets_sub(nh, "/detections", 10);

    typedef message_filters::sync_policies::ApproximateTime<track_msgs::Pairs, 
                                                            track_msgs::StampArray,
                                                            track_msgs::Detection_list, 
                                                            track_msgs::StampArray> MySynPolicy;
    
    message_filters::Synchronizer<MySynPolicy> sync(MySynPolicy(10), 
                                                    matched_pair_sub, 
                                                    unmatched_trk_sub,  
                                                    dets_sub, 
                                                    unmatched_det_sub);

    sync.registerCallback(boost::bind(processCallback, _1, _2, _3, _4, backend_p, trk_predict_pub));

    ros::spin();

    return 0;
}

