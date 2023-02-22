#pragma once
#ifndef MYTRK_EKF
#define MYTRK_EKF

#include "mycommon_include.h"

namespace mytrk
{

class CTRA_EKF : public Kalman::EKFilter<double,1>
{

public:

    CTRA_EKF()
    {
        setDim(6, 0, 3, 3, 3);
        T = 0.1;
    }

    void SetT(double T_)
    {
        T = T_;
    }

protected:

    void makeA();
    void makeH();
    void makeV();
    void makeR();
    void makeW();
    void makeQ();
    void makeProcess();
    void makeMeasure();

    double T;

};


} // namespace mytrk



#endif