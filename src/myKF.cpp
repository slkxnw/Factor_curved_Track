#include "factor_curved_track/myKF.h"

namespace mytrk
{

CV_KF::CV_KF(Vec6 init_state, Vec3 dt_a)
{
    x_state = init_state;
    delta_ax = dt_a[0];
    delta_ay = dt_a[1];
    delta_ath = dt_a[2];
    
    //设置A，A值不确定，x的增量和dt有关，右上角暂时初始化为1
    A = Mat66::Identity();
    A.block<3, 3>(0, 3) = Mat33::Identity();

    C = Mat36::Zero();
    C.block<3, 3>(0,0) = Mat33::Identity();

    P = Mat66::Identity();
    //借鉴AB3DMOT，初始时刻，对于速度的不确定性是非常大的，因此相关的协方差设的很大
    P.block<3,3>(3, 3) = P.block<3,3>(3, 3) * 1000; 
    P = P * 10;

    Q = Mat66::Identity();
    //AB3DMOT认为速度是常量这一假设非常强，将速度相关的协方差设置的非常小
    Q.block<3, 3>(3, 3) = Q.block<3, 3>(3, 3) * 0.01;
    // std::cout<<Q<<std::endl;
}

void CV_KF::predict(double dt)
{
    //实际上，我是按照另一个思路算的，参考下面文章中的Q推导
    //https://zhuanlan.zhihu.com/p/389589611
    // Q << 0.25 * delta_ax * delta_ax * (double)pow(dt, 4), 0, 0, 0.5 * delta_ax * delta_ax * (double)pow(dt, 3), 0, 0,
    //      0, 0.25 * delta_ay * delta_ay * (double)pow(dt, 4), 0, 0, 0.5 * delta_ay * delta_ay * (double)pow(dt, 3), 0,
    //      0, 0, 0.25 * delta_ath * delta_ath * (double)pow(dt, 4), 0, 0, 0.5 * delta_ath * delta_ath * (double)pow(dt, 3),
    //      0.5 * delta_ax * delta_ax * (double)pow(dt, 3), 0, 0, dt * dt * delta_ax * delta_ax, 0, 0,
    //      0, 0.5 * delta_ay * delta_ay * (double)pow(dt, 3), 0, 0, dt * dt * delta_ay * delta_ay, 0,
    //      0, 0, 0.5 * delta_ath * delta_ath * (double)pow(dt, 3), 0, 0, dt * dt * delta_ath * delta_ath;
    
    A.block<3, 3>(0, 3) = Mat33::Identity() * dt;
    x_state = A * x_state;
    P = A * P * (A.transpose()) + Q;
}

void CV_KF::update(Vec3 measure)
{
    Mat33 Tmp = C * P * (C.transpose()) + R;
    Mat63 K = P * (C.transpose()) * (Tmp.inverse());
    x_state = x_state + K * (measure - C * x_state);
    
    P = (Mat66::Identity() - K * C) * P;
}

AB3D_KF::AB3D_KF(Vec10 init_state)
{
    x_state = init_state;
    
    //设置A，A值不确定，x的增量和dt有关，右上角暂时初始化为1
    A = Mat1010::Identity();
    A.block<3, 3>(0, 7) = Mat33::Identity();
    std::cout<<A<<std::endl;

    C = Mat710::Zero();
    C.block<7, 7>(0,0) = Mat77::Identity();

    P = Mat1010::Identity();
    //AB3DMOT，初始时刻，对于速度的不确定性是非常大的，因此相关的协方差设的很大
    P.block<3,3>(7, 7) = P.block<3,3>(7, 7) * 1000; 
    P = P * 10;
    std::cout<<P<<std::endl;
    Q = Mat1010::Identity();
    //AB3DMOT认为速度是常量这一假设非常强，将速度相关的协方差设置的非常小
    Q.block<3, 3>(7, 7) = Q.block<3, 3>(7, 7) * 0.01;
    // std::cout<<Q<<std::endl;
}

void AB3D_KF::predict(double dt)
{
    A.block<3, 3>(0, 7) = Mat33::Identity() * dt;
    x_state = A * x_state;
    P = A * P * (A.transpose()) + Q;
}

void AB3D_KF::update(Vec7 measure)
{
    Mat77 Tmp = C * P * (C.transpose()) + R;
    Mat107 K = P * (C.transpose()) * (Tmp.inverse());
    x_state = x_state + K * (measure - C * x_state);
    
    P = (Mat1010::Identity() - K * C) * P;
}



} // namespace mytrk
