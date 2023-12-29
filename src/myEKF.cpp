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
    x_state_pred[0] = x_state[0] + dt * x_state[3] + 0.5 * dt * dt * x_state[6];
    x_state_pred[1] = x_state[1] + dt * x_state[4] + 0.5 * dt * dt * x_state[7];
    x_state_pred[2] = x_state[2] + dt * x_state[5];
    x_state_pred[3] = x_state[3] + dt * x_state[6];
    x_state_pred[4] = x_state[4] + dt * x_state[7];

    P_pred = J * P * J.transpose() + Q;
}

void CA_EKF::update(Vec3 measure)
{
    Mat33 tmp = C * P_pred * C.transpose() + R;
    Mat83 K = P_pred * C.transpose() * tmp.inverse();

    Vec3 tmp_diff = measure - C * x_state_pred;
    x_state = x_state_pred + K * tmp_diff;
    P = (Mat88::Identity() - K * C) * P_pred;
}

//CTRA
CTRA_EKF::CTRA_EKF(Vec6 init_state)
{
    x_state = init_state;
    
    //雅克比矩阵是变化的，对角线的元素确定
    J = Mat66::Identity();
    // std::cout<<J<<std::endl;
    C = Mat36::Zero();
    C.block<3, 3>(0, 0) = Mat33::Identity();

    //对匀加速的部分更加确定，对应方差项设计的很小，控制状态转移方差，描述控制的不确定性
    Q = Mat66::Identity();
    // Q.block<2, 2>(6, 6) = Q.block<2, 2>(6, 6) * 0.01;

    //初始状态分布中，状态的协方差均很大，分布都很不确定，但是，速度和加速度的不确定会更大
    P = Mat66::Identity();
    // P.block<2, 2>(6, 6) = P.block<2, 2>(6, 6) * 100;
    // P.block<5, 5>(3, 3) = P.block<5, 5>(3, 3) * 50;
    // P = P * 10;
}

void CTRA_EKF::calJacobi(double _dt)
{
    //dx = dt * vx + 0.5 * dt * dt * ax
    // std::cout<<J<<std::endl;
    double th = x_state[2];
    double v = x_state[3];
    double a = x_state[4];
    double w = x_state[5];
        
    double dth = w * _dt;
    double dv = a * _dt;
    J << 1, 0, ((v * w + a * dth) * cos(th + dth) - a * sin(th + dth) - v * w * cos(th) + a * sin(th)) / ((w + 1e-6) * (w + 1e-6)),
        (sin(th + dth) - sin(th)) / (w + 1e-6), (dth * sin(th + dth) + cos(th + dth) - cos(th)) / ((w + 1e-6) * (w + 1e-6)), 
        (v + a * _dt) / (w + 1e-6) * cos(w * _dt + th) * _dt - (v + a * _dt) / ((w + 1e-6) * (w + 1e-6)) * sin(w * _dt + th) - a * _dt / ((w + 1e-6) * (w + 1e-6)) * sin(w * _dt + th)
         - 2 * a * cos(w * _dt + th) / ((w + 1e-6) * (w + 1e-6) * (w + 1e-6)) + v * sin(th) / ((w + 1e-6) * (w + 1e-6)) + 2 * a * cos(th) / ((w + 1e-6) * (w + 1e-6) * (w + 1e-6)),
                            0, 1, ((v * w + a * dth) * sin(th + dth) + a * cos(th + dth) - v * w * sin(th) - a * cos(th)) / ((w + 1e-6) * (w + 1e-6)),
        (-cos(th + dth) + cos(th)) / (w + 1e-6), (-dth * cos(th + dth) + sin(th + dth) - sin(th)) / ((w + 1e-6) * (w + 1e-6)),
        (v + a * _dt) / (w + 1e-6) * sin(w * _dt + th) * _dt + (v + a * _dt) / ((w + 1e-6) * (w + 1e-6)) * cos(w * _dt + th) + a * _dt / ((w + 1e-6) * (w + 1e-6)) * cos(w * _dt + th)
         - 2 * a * sin(w * _dt + th) / ((w + 1e-6) * (w + 1e-6) * (w + 1e-6)) - v * cos(th) / ((w + 1e-6) * (w + 1e-6)) + 2 * a * sin(th) / ((w + 1e-6) * (w + 1e-6) * (w + 1e-6)), 
                            0, 0, 1, 0, 0, _dt,
                            0, 0, 0, 1, _dt, 0,
                            0, 0, 0, 0, 1, 0,
                            0, 0, 0, 0, 0, 1;
}

void CTRA_EKF::predict(double dt)
{
    calJacobi(dt);
    double th = x_state[2];
    double v = x_state[3];
    double a = x_state[4];
    double w = x_state[5];
        
    double dth = w * dt;
    double dv = a * dt;

    //[(v(t)ω + aωT) sin(θ(t) + ωT)+a cos(θ(t) + ωT)−v(t)ω sin θ(t) − a cos θ(t)] / ω^2
    double dx = ((v * w + a * dth) * sin(th + dth) + a * cos(th + dth)
         - v * w * sin(th) - a * cos(th)) / ((w + 1e-6) * (w + 1e-6));
    //[(−v(t)ω − aωT) cos(θ(t) + ωT)+a sin(θ(t) + ωT)+v(t)ω cos θ(t) − a sin θ(t)] / ω^2
    double dy = ((-v * w - a * dth) * cos(th + dth) + a * sin(th + dth)
         + v * w * cos(th) - a * sin(th)) / ((w + 1e-6) * (w + 1e-6));
    x_state_pred[0] = x_state[0] + dx;
    x_state_pred[1] = x_state[1] + dy;
    x_state_pred[2] = x_state[2] + dth;
    x_state_pred[3] = x_state[3] + dv;
    x_state_pred[4] = x_state[4];
    x_state_pred[5] = x_state[5];

    P_pred = J * P * J.transpose() + Q;
}

void CTRA_EKF::update(Vec3 measure)
{
    Mat33 tmp = C * P_pred * C.transpose() + R;
    Mat63 K = P_pred * C.transpose() * tmp.inverse();

    Vec3 tmp_diff = measure - C * x_state_pred;
    x_state = x_state_pred + K * tmp_diff;
    P = (Mat66::Identity() - K * C) * P_pred;
}

} // namespace mytrk
