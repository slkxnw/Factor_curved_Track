# include "myfrontend.h"

namespace mytrk
{

bool myFrontend::BuildInitTrkList(Vec3 measure, double time, unsigned int id)
{
    if (cur_frame_ != nullptr)
    {
        LOG(INFO) << "Not a empty trk_list!";
        return false;
    }

    last_state_ << 0.0, 0.0, 0.0, 14.0, 0.0, 0.0;
    CreateMeasureFrame(measure, time);

    myTrkList::Ptr new_trk = myTrkList::Ptr(new myTrkList);
    new_trk->InsertKeyframe(cur_frame_);
    new_trk->SetObjID(id);

    trk_list_ = new_trk;
    return true;
}

void myFrontend::ConcatTrklist(myTrkList::Ptr new_trk_list)
{
    myTrkList::KeyframeType new_frames = new_trk_list->GetAllKeyframe();
    for(auto &kf : new_frames)
    {
        myFrame::Ptr tmp = CreateMeasureFrame(kf.second->obj_state_, kf.second->time_stamp_, kf.second->is_measure_);
        trk_list_->InsertKeyframe(tmp);
    }
}

void myFrontend::Stop()
{
    frontend_running_.store(true);
    trk_list_update_.notify_one();
    frontend_thread.join();
}

} // namespace mytrk
