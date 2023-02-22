#pragma once
#ifndef MYTRK_KF
#define MYTRK_KF

#include "mycommon_include.h"

namespace mytrk
{

class CV_KF
{

public:

    CV_KF(Vec3 init_state, Vec3 dt_a);

    void predict(double dt);

    void update(Vec3 measure);

    Vec6 GetState()
    {
        return x_state;
    }

private:

    //状态转移方程/运动方程变换矩阵参数
    //x_t+1 = A * x_t + B * u + w//这里没有控制量
    //w是一个多元高斯噪声，均值为0， 方差为Q
    Mat66 A;
    Mat66 Q;
    //观测方程
    //z_t = C * x_t + v
    //v是一个多远高斯噪声，均值为0，方差为R
    Mat36 C;
    Mat33 R = Mat33::Identity();
    //状态是一个高斯分布，均值为x_state，方差为P
    Vec6 x_state;
    Mat66 P;

    double delta_ax,delta_ay, delta_ath;
};

} // namespace mytrk

#endif
