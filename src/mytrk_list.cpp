# include "factor_curved_track/mytrk_list.h"


namespace mytrk
{

void myTrkList::InsertKeyframe(std::shared_ptr<myFrame> kf)
{
    cur_frame_ = kf;
    if(keyframes_.find(kf->id_) == keyframes_.end())
    {
        keyframes_.insert(std::make_pair(kf->id_, kf));
        active_keyframes_.insert(std::make_pair(kf->id_, kf));
        active_keyframes_ids.push_back(kf->id_);
    }
    else
    {
        keyframes_[kf->id_] = kf;
        active_keyframes_[kf->id_] = kf;
    }

    if(active_keyframes_.size() > num_active_keyframes)
        ResetKeyframe();
}

void myTrkList::ResetKeyframe()
{
    if (cur_frame_ == nullptr)
        return;
    double oldest_time = cur_frame_->time_stamp_, oldest_notmea_time = cur_frame_->time_stamp_;
    unsigned long oldest_id = 0, not_measure_id = -1;
    for(auto & kf : active_keyframes_)
    {
        std::cout<<"id:"<<kf.first<<" "<<"time:"<<kf.second->time_stamp_<<std::endl;
        double time = kf.second->time_stamp_;
        if(oldest_time > time)
        {
            oldest_time = time;
            oldest_id = kf.first;
        }

        if(!kf.second->is_measure_)
        {
            oldest_notmea_time = oldest_notmea_time < time? oldest_time:time;
            not_measure_id = kf.first;
        }
    }
    myFrame::Ptr frame_to_remove = nullptr;
    //正常来说，观测帧100ms一帧数据，20帧数据时间差为2s，max_obs_time_gap取为3秒，如果超出三秒，说明中间有非观测帧
    if (cur_frame_->time_stamp_ - oldest_time > max_obs_time_gap)
    {
        frame_to_remove = keyframes_.at(oldest_id);
        LOG(INFO) << "remove keyframe of obj_"<<obj_id_<<" at " << frame_to_remove->id_ << "for time gap";
    }
    //如果中间最老旧的非观测帧，和当前帧时间差大于max_ext_time_gap，删除最老旧的非观测帧
    else if (!not_measure_id && oldest_notmea_time > max_ext_time_gap)
    {
        frame_to_remove = keyframes_.at(not_measure_id);
        LOG(INFO) << "remove keyframe of obj_"<<obj_id_<<" at " << frame_to_remove->id_ << "for not measurment";
    }
    //没有非观测帧,或者非观测帧和当前帧时间差距过小
    else
    {
        frame_to_remove = keyframes_.at(oldest_id);
        LOG(INFO) << "remove keyframe of obj_"<<obj_id_<<" at " << frame_to_remove->id_;
    }

    active_keyframes_.erase(frame_to_remove->id_);
    auto er_pos = std::find(active_keyframes_ids.cbegin(), active_keyframes_ids.cend(), int(frame_to_remove->id_));
    active_keyframes_ids.erase(er_pos);
}

} // namespace mytrk
