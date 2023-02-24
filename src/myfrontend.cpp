# include<ros/ros.h>
# include "factor_curved_track/myfrontend.h"
# include "factor_curved_track/myg2o_simple_types.h"
namespace mytrk
{

myFrontend::myFrontend()
{
    // frontend_running_.store(true);
    // frontend_thread = std::thread(std::bind(&myFrontend::FrontendLoop, this));
    last_state_ = Vec6::Zero();
    last_state_[3] = 15;
    last_state_[4] = 0.0;
    last_state_[5] = 0.0;
    last_timestamp_ = 0;
}


bool myFrontend::BuildInitTrkList(Vec3 measure, double time, unsigned int id)
{
    if (cur_frame_ != nullptr)
    {
        LOG(INFO) << "Not a empty trk_list!";
        return false;
    }

    CreateMeasureFrame(measure, time);

    std::shared_ptr<myTrkList> new_trk = std::shared_ptr<myTrkList>(new myTrkList(id));
    
    // new_trk->SetObjID(id);
    
    new_trk->InsertKeyframe(cur_frame_);

    trk_list_ = new_trk;
    return true;
}

bool myFrontend::BuildInitKF(Vec6 state, Vec3 vars)
{
    std::shared_ptr<CV_KF> new_kf = std::shared_ptr<CV_KF>(new CV_KF(state, vars));
    kf_ = new_kf;

    return true;
}

bool myFrontend::BuildInitCA_EKF(Vec8 state)
{
    std::shared_ptr<CA_EKF> new_kf = std::shared_ptr<CA_EKF>(new CA_EKF(state));
    ca_ekf_ = new_kf;

    return true;
}

void myFrontend::ConcatTrklist(std::shared_ptr<myTrkList> new_trk_list)
{
    myTrkList::KeyframeType new_frames = new_trk_list->GetAllKeyframe();
    Vec3 measure_fake;
    for(auto &kf : new_frames)
    {
        measure_fake << kf.second->obj_state_[0], kf.second->obj_state_[1], kf.second->obj_state_[2];
        myFrame::Ptr tmp = CreateMeasureFrame(measure_fake, kf.second->time_stamp_, kf.second->is_measure_);
        trk_list_->InsertKeyframe(tmp);
    }
}

//TODO 修改结束函数
void myFrontend::Stop()
{
    // frontend_running_.store(false);
    // trk_list_update_.notify_one();
    // frontend_thread.join();
}

myFrame::Ptr myFrontend::CreateMeasureFrame(Vec3 measure, double time, bool is_measure)
{
    //TODO 确认一下这里，插入当前帧后，之前的当前帧就变成了lastframe，然后更新last_frame数据
    last_frame_ = cur_frame_;
    GetLastFrameInfo();
    myFrame::Ptr new_frame(new myFrame);
    last_state_[3] += last_state_[4] * (time - last_timestamp_);
    Vec6 cur_state;
    //x,y,theta,v,a,w
    cur_state << measure[0], measure[1], measure[2], last_state_[3], last_state_[4], last_state_[5];
    new_frame->id_ = num_of_frames++;
    new_frame->obj_state_ = cur_state;
    new_frame->time_stamp_ = time;
    new_frame->is_measure_ = true;
    
    cur_frame_ = new_frame;
    
    return new_frame;
}
//这个函数没有用到，可以认为不存在
//TODO 后续再确定，外推帧是否可以作为cur_frame_
myFrame::Ptr myFrontend::CreateExtrpFrame(double time)
{
    //TODO 这里，如果连续出现外推帧，那么last_frame会是外推帧
    last_frame_ = cur_frame_;
    GetLastFrameInfo();
    
    myFrame::Ptr new_frame(new myFrame);
    
    Vec6 cur_state = PredictState(time);
    new_frame->id_ = num_of_frames++;
    new_frame->obj_state_ = cur_state;
    new_frame->time_stamp_ = time;
    new_frame->is_measure_ = false;
        

    cur_frame_ = new_frame;
    return new_frame;
}

// void myFrontend::FrontendLoop()
// {
//     while(frontend_running_.load())
//     {
//         std::unique_lock<std::mutex> lock(data_mutex_);
//         trk_list_update_.wait(lock);

//         auto active_kfs = trk_list_->GetActivateKeyframe();
//         Optimize(active_kfs);
//     }
// }

void myFrontend::UpdateTrkList()
{
    // std::unique_lock<std::mutex> lock(data_mutex_);
    // trk_list_update_.notify_one();
    auto active_kfs = trk_list_->GetActivateKeyframe();
    auto active_kf_ids = trk_list_->GetActivateKeyframeIDs();
    Optimize(active_kfs, active_kf_ids);
}
//观测较少的时候，使用kf估计状态
void myFrontend::UpdateTrkListKF()
{
    double dt = cur_frame_->ObjTimestamp() - last_timestamp_;
    // std::cout<<dt<<" "<<dt<<std::endl;
    //x,y,th,v,a,w
    Vec6 state = cur_frame_->ObjState();
    Vec3 measure;
    measure << state[0], state[1], state[2];
    
    kf_->predict(dt);
    kf_->update(measure);
    //x,y,th,vx,vy,w
    Vec6 kf_state = kf_->GetState();
    //更新frame状态
    double v = sqrtf64(pow(kf_state[3], 2) + pow(kf_state[4], 2));
    double a = (v - last_frame_->ObjState()[3]) / dt;
    double th = kf_state[2];
    //TODO vx和vy的夹角也能表现th，是否考虑把它和上面的th综合一下呢？
    Vec6 new_state;
    new_state << kf_state[0], kf_state[1], th, v, 0, kf_state[5];

    cur_frame_->SetObjState(new_state);
}
//CA_EKF估计
void myFrontend::UpdateTrkListCA_EKF()
{
    double dt = cur_frame_->ObjTimestamp() - last_timestamp_;
    // std::cout<<dt<<" "<<dt<<std::endl;
    //x,y,th,v,a,w
    Vec6 state = cur_frame_->ObjState();
    Vec3 measure;
    measure << state[0], state[1], state[2];
    std::cout<<"start_ekf"<<std::endl;
    ca_ekf_->predict(dt);
    ca_ekf_->update(measure);
    std::cout<<"finish_ekf"<<std::endl;
    //x,y,th,vx,vy,w, ax, ay
    Vec8 kf_state = ca_ekf_->GetState();
    //更新frame状态
    double v = sqrtf64(pow(kf_state[3], 2) + pow(kf_state[4], 2));
    double a = sqrtf64(pow(kf_state[6], 2) + pow(kf_state[7], 2));
    double th = kf_state[2];
    //TODO vx和vy的夹角也能表现th，是否考虑把它和上面的th综合一下呢？
    Vec6 new_state;
    new_state << kf_state[0], kf_state[1], th, v,  a, kf_state[5];

    cur_frame_->SetObjState(new_state);
}

/**
*预测 x y theta,基于当前帧的状态，预测给定时刻的状态
*/
Vec3 myFrontend::PredictPostion(double time)
{
    Vec3 pred_position;
    Vec6 cur_state = cur_frame_->ObjState();
    double cur_time = cur_frame_->ObjTimestamp();
    double th = cur_state[2];
    double v = cur_state[3];
    double a = cur_state[4];
    double w = cur_state[5];
    double dt = time - cur_time;
        
    double dth = w * dt;
    double dv = a * dt;

    //[(v(t)ω + aωT) sin(θ(t) + ωT)+a cos(θ(t) + ωT)−v(t)ω sin θ(t) − a cos θ(t)] / ω^2
    double dx = ((v * w + a * dth) * sin(th + dth) + a * cos(th + dth)
         - v * w * sin(th) - a * cos(th)) / ((w + 1e-6) * (w + 1e-6));
    //[(−v(t)ω − aωT) cos(θ(t) + ωT)+a sin(θ(t) + ωT)+v(t)ω cos θ(t) − a sin θ(t)] / ω^2
    double dy = ((-v * w - a * dth) * cos(th + dth) + a * sin(th + dth)
         + v * w * cos(th) - a * sin(th)) / ((w + 1e-6) * (w + 1e-6));
    int id_ = trk_list_->GetObjID();
    // std::cout<<id_<<"pred_growth: dt:"<<dt<<" dv:"<<dv<<" dth:"<<dth<<" dx:"<<dx<<" dy:"<<dy<<std::endl;
    // std::cout<<id_<<"_state: x:"<<cur_state[0]<<" y:"<<cur_state[1]<<" th:"<<th<<" v:"<<v<<" a:"<<a<<" w:"<<w<<std::endl;
    pred_position<<cur_state[0] + dx, cur_state[1] + dy, cur_state[2] + dth;
    return pred_position;
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
    
    cur_state << cur_position[0], cur_position[1], cur_position[2] + dth,
                 last_state_[3] + dv, last_state_[4], last_state_[5];
    return cur_state;
}

Vec3 myFrontend::GetCurPosition()
{
    Vec3 cur_position;
    Vec6 cur_state = cur_frame_->ObjState();
    cur_position<<cur_state[0], cur_state[1], cur_state[2];
    return cur_position;
}


void myFrontend::Optimize(myTrkList::KeyframeType &keyframes, std::vector<int> &kf_ids)
{
    //TODO:确定huber鲁棒核函数参数如何设计
    
    
    double chi2_th = 5.991;  // robust kernel 阈值
    
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 6>> BlockSolverType;
    typedef g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType> LinearSolverType;
    // typedef g2o::LinearSolverCSparse<BlockSolverType::PoseMatrixType> LinearSolverType;


    //这是一个简化的写法，正常要写三行代码
    auto solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<BlockSolverType> (g2o::make_unique<LinearSolverType>()));
    // auto solver = new g2o::OptimizationAlgorithmGaussNewton(
        // g2o::make_unique<BlockSolverType> (g2o::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(false);

    int edge_cnt = 0;
    std::map<unsigned long, VertexState *> vertexs;
    std::map<unsigned long, EdgeConstVary *> edges;
    std::sort(kf_ids.begin(), kf_ids.end());
    double last_time = 0;
    unsigned long last_id = 0;

    for(int & frame_id : kf_ids)
    {
        // std::cout<<"当前帧的id:"<<frame_id<<std::endl;
        auto kf = keyframes[frame_id];
        VertexState * vertex_state = new VertexState();
        vertex_state->setId(frame_id);
        vertex_state->setEstimate(kf->obj_state_);
        optimizer.addVertex(vertex_state);
        vertexs.insert({frame_id, vertex_state});
        if(vertexs.size() > 1)
        {
            double dt = kf->time_stamp_ - last_time;
            EdgeConstVary *edge = new EdgeConstVary(dt);

            edge->setId(edge_cnt);
            edge->setVertex(0, vertexs.at(last_id));
            edge->setVertex(1, vertexs.at(frame_id));
            edge->setMeasurement(Vec6::Zero());
            edge->setInformation(Mat66::Identity());
            auto rk = new g2o::RobustKernelHuber();
            rk->setDelta(chi2_th);
            edge->setRobustKernel(rk);
            optimizer.addEdge(edge);
            edges.insert({edge_cnt, edge});
            edge_cnt++;
        }
        last_time = kf->time_stamp_;
        last_id = frame_id;
    }
    //TODO：关键帧是放在哈希表中的，然后遍历的时候在两帧之间建立边，这样会不会有问题，遍历的顺序和帧id顺序是否一致呢？进行一些修改


    //感觉应该维护一个活跃帧id列表
    // for (auto &kf_pair : keyframes)
    // {
    //     std::cout<<"当前帧的id:"<<last_id<<std::endl;
    //     auto kf = kf_pair.second;
    //     VertexState * vertex_state = new VertexState();
    //     vertex_state->setId(kf_pair.first);
    //     vertex_state->setEstimate(kf->obj_state_);
    //     optimizer.addVertex(vertex_state);
    //     vertexs.insert({kf_pair.first, vertex_state});
    //     if (last_time != 0.0)
    //     {
    //         double dt = kf->time_stamp_ - last_time;
    //         EdgeConstVary * edge = new EdgeConstVary(dt);
    //         edge->setId(edge_cnt);
    //         edge->setVertex(0, vertexs.at(last_id));
    //         edge->setVertex(1, vertexs.at(kf_pair.first));
    //         edge->setMeasurement(Vec6::Zero());
    //         edge->setInformation(Mat66::Identity());
    //         auto rk = new g2o::RobustKernelHuber();
    //         rk->setDelta(chi2_th);
    //         edge->setRobustKernel(rk);
    //         optimizer.addEdge(edge);
    //         edges.insert({edge_cnt, edge});
    //         edge_cnt++;
    //     }
    //     last_id = kf_pair.first;
    //     last_time = kf->time_stamp_;
    // }
    
    ROS_INFO("Trere is %d kf in %d", keyframes.size(), trk_list_->GetObjID());
    optimizer.initializeOptimization();
    optimizer.optimize(60);

    for(auto &v : vertexs){
        std::cout<<v.first<<' '<<v.second->estimate().transpose()<<std::endl;
        keyframes.at(v.first)->SetObjState(v.second->estimate());
    }
         
}

} // namespace mytrk
