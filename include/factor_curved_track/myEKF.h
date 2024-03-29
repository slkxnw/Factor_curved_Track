#pragma once
#ifndef MYTRK_EKF
#define MYTRK_EKF

#include "mycommon_include.h"

namespace mytrk
{

// class CTRA_EKF : public Kalman::EKFilter<double,1>
// {

// public:

//     CTRA_EKF()
//     {
//         setDim(6, 0, 3, 3, 3);
//         T = 0.1;
//     }

//     void SetT(double T_)
//     {
//         T = T_;
//     }

// protected:

//     void makeA();
//     void makeH();
//     void makeV();
//     void makeR();
//     void makeW();
//     void makeQ();
//     void makeProcess();
//     void makeMeasure();

//     double T;

// };

class CA_EKF
{

public:

    CA_EKF(Vec8 init_state);

    void calJacobi(double dt);
    
    void predict(double dt);

    void update(Vec3 measure);

    Vec8 GetState()
    {
        return x_state;
    }
    Vec8 GetStatePred()
    {
        return x_state_pred;
    }

private:

    //状态转移方程/运动方程变换矩阵参数
    //x_t+1 = A * x_t + B * u + w//这里没有控制量
    //w是一个多元高斯噪声，均值为0， 方差为Q
    Mat88 J;
    Mat88 Q;
    //观测方程
    //z_t = C * x_t + v
    //v是一个多元高斯噪声，均值为0，方差为R
    Mat38 C;
    Mat33 R = Mat33::Identity();
    //状态是一个高斯分布，均值为x_state，方差为P
    Vec8 x_state;
    Vec8 x_state_pred;
    Mat88 P;
    Mat88 P_pred;

    double delta_ax,delta_ay, delta_ath;



};


class CTRA_EKF
{

public:

    CTRA_EKF(Vec6 init_state);

    void calJacobi(double dt);
    
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
    Mat66 J;
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


} // namespace mytrk



#endif