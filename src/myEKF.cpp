#include "factor_curved_track/myEKF.h"

namespace mytrk
{

CA_EKF::CA_EKF(Vec8 init_state)
{
    x_state = init_state;
    
    //雅克比矩阵是变化的，对角线的元素确定
    J = Mat88::Identity();
    // std::cout<<J<<std::endl;
    C = Mat38::Zero();
    C.block<3, 3>(0, 0) = Mat33::Identity();

    //对匀加速的部分更加确定，对应方差项设计的很小，控制状态转移方差，描述控制的不确定性
    Q = Mat88::Identity();
    Q.block<2, 2>(6, 6) = Q.block<2, 2>(6, 6) * 0.01;

    //初始状态分布中，状态的协方差均很大，分布都很不确定，但是，速度和加速度的不确定会更大
    P = Mat88::Identity();
    P.block<2, 2>(6, 6) = P.block<2, 2>(6, 6) * 100;
    P.block<5, 5>(3, 3) = P.block<5, 5>(3, 3) * 50;
    P = P * 10;
}

void CA_EKF::calJacobi(double dt)
{
    //dx = dt * vx + 0.5 * dt * dt * ax
    // std::cout<<J<<std::endl;
    J(0, 3) = dt;
    J(0, 6) = 0.5 * dt * dt;
    J(1, 4) = dt;
    J(1, 7) = 0.5 * dt * dt;
    J(2, 5) = dt;
    //dvx = dt * ax    
    J(3, 6) = dt;
    J(4, 7) = dt;
}

void CA_EKF::predict(double dt)
{
    calJacobi(dt);
    x_state[0] = x_state[0] + dt * x_state[3] + 0.5 * dt * dt * x_state[6];
    x_state[1] = x_state[1] + dt * x_state[4] + 0.5 * dt * dt * x_state[7];
    x_state[2] = x_state[2] + dt * x_state[5];
    x_state[3] = x_state[3] + dt * x_state[6];
    x_state[4] = x_state[4] + dt * x_state[7];

    P = J * P * J.transpose() + Q;
}

void CA_EKF::update(Vec3 measure)
{
    Mat33 tmp = C * P * C.transpose() + R;
    Mat83 K = P * C.transpose() * tmp.inverse();

    Vec3 tmp_diff = measure - C * x_state;
    x_state = x_state + K * tmp_diff;
    P = (Mat88::Identity() - K * C) * P;
}

} // namespace mytrk
