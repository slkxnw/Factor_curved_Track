#pragma once
#ifndef MYTRK_KF
#define MYTRK_KF

#include "mycommon_include.h"

namespace mytrk
{

class CV_KF
{

public:

    CV_KF(Vec6 init_state, Vec3 dt_a);

    void predict(double dt);

    void update(Vec3 measure);

    Vec6 GetState()
    {
        return x_state;
    }
    Vec6 GetStatePred()
    {
        return x_state_pred;
    }

private:

    //状态转移方程/运动方程变换矩阵参数
    //x_t+1 = A * x_t + B * u + w//这里没有控制量
    //w是一个多元高斯噪声，均值为0， 方差为Q
    Mat66 A;
    Mat66 Q;
    //观测方程
    //z_t = C * x_t + v
    //v是一个多元高斯噪声，均值为0，方差为R
    Mat36 C;
    Mat33 R = Mat33::Identity();
    //状态是一个高斯分布，均值为x_state，方差为P
    Vec6 x_state;
    Vec6 x_state_pred;
    Mat66 P;
    Mat66 P_pred;

    double delta_ax,delta_ay, delta_ath;
};


class AB3D_KF
{
    public:

    AB3D_KF(Vec10 init_state);

    void predict(double dt);

    void update(Vec7 measure);

    double orin_correction(double obv_theta);

    Vec10 GetState()
    {
        return x_state;
    }
    Vec10 GetStatePred()
    {
        return x_state_pred;
    }

    double within_range(double theta);

    void SetState(Vec3 new_pos)
    {
        x_state[0] = new_pos[0];
        x_state[1] = new_pos[1];
        x_state[2] = new_pos[2];
    }
private:

    //状态转移方程/运动方程变换矩阵参数
    //x_t+1 = A * x_t + B * u + w//这里没有控制量
    //w是一个多元高斯噪声，均值为0， 方差为Q
    Mat1010 A;
    Mat1010 Q;
    //观测方程
    //z_t = C * x_t + v
    //v是一个多元高斯噪声，均值为0，方差为R
    Mat710 C;
    Mat77 R = Mat77::Identity();
    //状态是一个高斯分布，均值为x_state，方差为P
    Vec10 x_state;
    Vec10 x_state_pred;
    Mat1010 P;
    Mat1010 P_pred;

    double delta_ax,delta_ay, delta_ath;
};

} // namespace mytrk

#endif
