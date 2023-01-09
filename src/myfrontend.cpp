# include "myfrontend.h"

namespace mytrk
{

myFrontend::myFrontend()
{
    frontend_running_.store(true);
    frontend_thread = std::thread(std::bind(FrontendLoop, this));
}


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
    
    new_trk->SetObjID(id);
    new_trk->InsertKeyframe(cur_frame_);

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

myFrame::Ptr myFrontend::CreateMeasureFrame(Vec3 measure, double time, bool is_measure = true)
{
    
    myFrame::Ptr new_frame(new myFrame);
    last_state_[3] += last_state_[4] * (time - last_timestamp_);
    Vec6 cur_state;
    cur_state << measure[0], measure[1], measure[2], last_state_[3], last_state_[4], last_state_[5];
    new_frame->id_ = num_of_frames++;
    new_frame->obj_state_ = cur_state;
    new_frame->time_stamp_ = time;
    new_frame->is_measure_ = is_measure;
    
    last_frame_ = cur_frame_;
    cur_frame_ = new_frame;
    return new_frame;
}

myFrame::Ptr myFrontend::CreateExtrpFrame(double time)
{
    myFrame::Ptr new_frame(new myFrame);
    
    Vec6 cur_state = PredictState(time);
    new_frame->id_ = num_of_frames++;
    new_frame->obj_state_ = cur_state;
    new_frame->time_stamp_ = time;
    new_frame->is_measure_ = false;
        
    last_frame_ = cur_frame_;
    cur_frame_ = new_frame;
    return new_frame;
}

void myFrontend::FrontendLoop()
{
    while(frontend_running_.load())
    {
        std::unique_lock<std::mutex> lock(data_mutex_);
        trk_list_update_.wait(lock);

        auto active_kfs = trk_list_->GetActivateKeyframe();
        Optimize(active_kfs);
    }
}

void myFrontend::UpdateTrkList()
{
    std::unique_lock<std::mutex> lock(data_mutex_);
    trk_list_update_.notify_one();
}

Vec3 myFrontend::PredictPostion(double time)
{
    Vec3 cur_position;
    double th = last_state_[2];
    double v = last_state_[3];
    double a = last_state_[4];
    double w = last_state_[5];
    double dt = time - last_timestamp_;
        
    double dth = w * dt;
    double dv = a * dt;

    //[(v(t)ω + aωT) sin(θ(t) + ωT)+a cos(θ(t) + ωT)−v(t)ω sin θ(t) − a cos θ(t)] / ω^2
    double dx = ((v * w + a * dth) * sin(th + dth) + a * cos(th + dth)
         - v * w * sin(th) - a * cos(th)) / (w * w);
    //[(−v(t)ω − aωT) cos(θ(t) + ωT)+a sin(θ(t) + ωT)+v(t)ω cos θ(t) − a sin θ(t)] / ω^2
    double dy = ((-v * w - a * dth) * cos(th + dth) + a * sin(th + dth)
         + v * w * cos(th) - a * sin(th)) / (w * w);
    cur_position<<last_state_[0] + dx, last_state_[1] + dy, last_state_[2] + dth;
    return cur_position;
}

Vec6 myFrontend::PredictState(double time)
{
    Vec6 cur_state;
    double a = last_state_[4];
    double w = last_state_[5];
    double dt = time - last_timestamp_;
        
    double dth = w * dt;
    double dv = a * dt;
    Vec3 cur_position = PredictPostion(time);
    
    cur_state << cur_position[0], cur_position[1], cur_position[2],
                 last_state_[3] + dv, last_state_[4], last_state_[5];
    return cur_state;
}

} // namespace mytrk
