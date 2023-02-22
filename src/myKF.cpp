#include "factor_curved_track/myKF.h"

namespace mytrk
{

CV_KF::CV_KF(Vec6 init_state, Vec3 dt_a)
{
    x_state = init_state;
    delta_ax = dt_a[0];
    delta_ay = dt_a[1];
    delta_ath = dt_a[2];
    
    A = Mat66::Identity();
    A[0, 3] = 1;
    A[1, 4] = 1;
    A[2, 5] = 1;

    C = Mat36::Zero();
    C[0, 0] = 1;
    C[1, 1] = 1;
    C[2, 2] = 1;

    P = Mat66::Identity();
    //借鉴AB3DMOT，初始时刻，对于速度的不确定性是非常大的，因此相关的协方差设的很大
    P.block<3,3>(3, 3) = Mat33::Ones() * 1000; 
    P = P * 10;

    Q = Mat66::Identity();
    //AB3DMOT认为速度是常量这一假设非常强，将速度相关的协方差设置的非常小
    //实际上，我是按照另一个思路算的，参考下面文章中的Q推导
    //https://zhuanlan.zhihu.com/p/389589611
    Q.block<3, 3>(3, 3) = Mat33::Ones() * 0.01;
}

void CV_KF::predict(double dt)
{
    Q << 0.25 * delta_ax * delta_ax * (double)pow(dt, 4), 0, 0, 0.5 * delta_ax * delta_ax * (double)pow(dt, 3), 0, 0,
         0, 0.25 * delta_ay * delta_ay * (double)pow(dt, 4), 0, 0, 0.5 * delta_ay * delta_ay * (double)pow(dt, 3), 0,
         0, 0, 0.25 * delta_ath * delta_ath * (double)pow(dt, 4), 0, 0, 0.5 * delta_ath * delta_ath * (double)pow(dt, 3),
         0.5 * delta_ax * delta_ax * (double)pow(dt, 3), 0, 0, dt * dt * delta_ax * delta_ax, 0, 0,
         0, 0.5 * delta_ay * delta_ay * (double)pow(dt, 3), 0, 0, dt * dt * delta_ay * delta_ay, 0,
         0, 0, 0.5 * delta_ath * delta_ath * (double)pow(dt, 3), 0, 0, dt * dt * delta_ath * delta_ath;
    
    x_state = A * x_state;
    P = A * P * A.transpose() + Q;
}

void CV_KF::update(Vec3 measure)
{
    Mat33 Tmp = C * P * C.transpose() + Q;
    Mat63 K = P * C.transpose() * Tmp.inverse();
    x_state = x_state + K * (measure - C * x_state);
    
    P = (Mat66::Identity() - K * C) * P;
}


} // namespace mytrk
