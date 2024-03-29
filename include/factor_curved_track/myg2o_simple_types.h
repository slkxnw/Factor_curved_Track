#ifndef MYTRK_SIMPLE_G2O_H
#define MYTRK_SIMPLE_G2O_H

#include "mycommon_include.h"

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_multi_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/auto_differentiation.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>

namespace mytrk
{

class VertexState : public g2o::BaseVertex<6, Vec6>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    virtual void setToOriginImpl() override
    {
        _estimate = Vec6();
    }

    virtual void oplusImpl(const double *update) override
    {
        _estimate[0] += update[0];
        _estimate[1] += update[1];
        _estimate[2] += update[2];
        _estimate[3] += update[3];
        _estimate[4] += update[4];
        _estimate[5] += update[5];
    }

    virtual bool read(std::istream &in) override 
    {return true;}

    virtual bool write(std::ostream &out) const override {return true;}

};

// x,y,theta,v,a,omega
class EdgeConstVary : public g2o::BaseBinaryEdge<6, Vec6, VertexState, VertexState>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeConstVary(const double &dt) : _dt(dt) {};

    virtual void computeError() override
    {
        VertexState *v0 = static_cast<VertexState *> (_vertices[0]);
        VertexState *v1 = static_cast<VertexState *> (_vertices[1]);

        Vec6 state_last = v0->estimate();
        Vec6 state_cur = v1->estimate();
        double th = state_last[2];
        double v = state_last[3];
        double a = state_last[4];
        double w = state_last[5];
        
        double dth = w * _dt;
        double dv = a * _dt;

        //TODO:可以把下面的计算作为measurement，在外部计算，把上一时刻的状态在外部保存就可以了
        //[(v(t)ω + aωT) sin(θ(t) + ωT)+a cos(θ(t) + ωT)−v(t)ω sin θ(t) − a cos θ(t)] / ω^2
        double dx = ((v * w + a * dth) * sin(th + dth) + a * cos(th + dth)
         - v * w * sin(th) - a * cos(th)) / ((w + 1e-6) * (w + 1e-6));
        //[(−v(t)ω − aωT) cos(θ(t) + ωT)+a sin(θ(t) + ωT)+v(t)ω cos θ(t) − a sin θ(t)] / ω^2
        double dy = ((-v * w - a * dth) * cos(th + dth) + a * sin(th + dth)
         + v * w * cos(th) - a * sin(th)) / ((w + 1e-6) * (w + 1e-6));

        //TODO 这里是不是反了，应该是后一个状态减去前一个状态，得到x，y,theta的增量，试着修改了
        _error<<dx + (state_last[0] - state_cur[0]), 
                dy + (state_last[1] - state_cur[1]),
                dth + (th - state_cur[2]),
                dv + (v - state_cur[3]),
                0 + (a - state_cur[4]),
                0 + (w - state_cur[5]);
    }

    //每个雅克比是6*6的，六个误差，分别针对每个顶点的六个参数的偏导数
    // x,y,theta,v,a,omega
    //dx, dy, dth, dv, da, dw
    virtual void linearizeOplus() override
    {
        VertexState *v0 = static_cast<VertexState *> (_vertices[0]);
        VertexState *v1 = static_cast<VertexState *> (_vertices[1]);
        Vec6 state_last = v0->estimate();
        Vec6 state_cur = v1->estimate();
        double th = state_last[2];
        double v = state_last[3];
        double a = state_last[4];
        double w = state_last[5];

        double dth = w * _dt;
        double dv = a * _dt;

        //[dx + (state_last[0] - state_cur[0])],对每个元素求偏导
        //dx = [(v(t)ω + aωT) sin(θ(t) + ωT)+a cos(θ(t) + ωT)−v(t)ω sin θ(t) − a cos θ(t)] / ω^2
        // _jacobianOplusXi << 1, 0, ((v * w + a * dth) * cos(th + dth) - a * sin(th + dth) - v * w * cos(th) + a * sin(th)) / ((w + 1e-9) * (w + 1e-9)),
        // (sin(th + dth) - sin(th)) / (w + 1e-9), (dth * sin(th + dth) + cos(th + dth) - cos(th)) / ((w + 1e-9) * (w + 1e-9)), 
        // ((v  + a * _dt) * sin(th + dth) + (v * w + a * dth) * cos(th + dth) * _dt - a * _dt * sin(th + dth) - v * sin(th)) / ((w + 1e-9) * (w + 1e-9)) 
        // - 2 * ((v * w + a * dth) * sin(th + dth) + a * cos(th + dth) - v * w * sin(th) - a * cos(th)) / ((w + 1e-9) * (w + 1e-9) * (w + 1e-9)),
        //                     0, 1, ((v * w + a * dth) * sin(th + dth) + a * cos(th + dth) - v * w * sin(th) - a * cos(th)) / ((w + 1e-9) * (w + 1e-9)),
        // (-cos(th + dth) + cos(th)) / (w + 1e-9), (-dth * cos(th + dth) +sin(th + dth) - sin(th)) / ((w + 1e-9) * (w + 1e-9)),
        // ((-v - a * _dt) * cos(th + dth) - (-v * w - a * dth) * sin(th + dth) * _dt + a * _dt * cos(th + dth) + v * cos(th)) / ((w + 1e-9) * (w + 1e-9)) 
        // - 2 * ((-v * w - a * dth) * cos(th + dth) + a * sin(th + dth) + v * w * cos(th) - a * sin(th)) / ((w + 1e-9) * (w + 1e-9) * (w + 1e-9)), 
        //                     0, 0, 1, 0, 0, _dt,
        //                     0, 0, 0, 1, _dt, 0,
        //                     0, 0, 0, 0, 1, 0,
        //                     0, 0, 0, 0, 0, 1;
        _jacobianOplusXi << 1, 0, ((v * w + a * dth) * cos(th + dth) - a * sin(th + dth) - v * w * cos(th) + a * sin(th)) / ((w + 1e-6) * (w + 1e-6)),
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
        _jacobianOplusXj = Mat66::Identity() * -1;
    }

    virtual bool read(std::istream &in) override {return true;}

    virtual bool write(std::ostream &out) const override {return true;}

private:
    double _dt;

};

class EdgeMeasure : public g2o::BaseUnaryEdge<3, Vec3, VertexState>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeMeasure(const double& dt ):_dt(dt){};

    virtual void computeError() override
    {
        VertexState *v0 = static_cast<VertexState *> (_vertices[0]);

        Vec6 state = v0->estimate();

        _error<<_measurement[0] - state[0], 
                _measurement[1] - state[1],
                _measurement[2] - state[2];
    }

    //每个雅克比是3*6的，3个误差，分别针对每个顶点的六个参数的偏导数
    // x,y,theta,v,a,omega
    //dx, dy, dth, dv, da, dw
    virtual void linearizeOplus() override
    {
        VertexState *v0 = static_cast<VertexState *> (_vertices[0]);
        VertexState *v1 = static_cast<VertexState *> (_vertices[1]);
        Vec6 state_last = v0->estimate();
        Vec6 state_cur = v1->estimate();
        double th = state_last[2];
        double v = state_last[3];
        double a = state_last[4];
        double w = state_last[5];

        double dth = w * _dt;
        double dv = a * _dt;

        //[dx + (state_last[0] - state_cur[0])],对每个元素求偏导
        //dx = [(v(t)ω + aωT) sin(θ(t) + ωT)+a cos(θ(t) + ωT)−v(t)ω sin θ(t) − a cos θ(t)] / ω^2
        // _jacobianOplusXi << 1, 0, ((v * w + a * dth) * cos(th + dth) - a * sin(th + dth) - v * w * cos(th) + a * sin(th)) / ((w + 1e-9) * (w + 1e-9)),
        // (sin(th + dth) - sin(th)) / (w + 1e-9), (dth * sin(th + dth) + cos(th + dth) - cos(th)) / ((w + 1e-9) * (w + 1e-9)), 
        // ((v  + a * _dt) * sin(th + dth) + (v * w + a * dth) * cos(th + dth) * _dt - a * _dt * sin(th + dth) - v * sin(th)) / ((w + 1e-9) * (w + 1e-9)) 
        // - 2 * ((v * w + a * dth) * sin(th + dth) + a * cos(th + dth) - v * w * sin(th) - a * cos(th)) / ((w + 1e-9) * (w + 1e-9) * (w + 1e-9)),
        //                     0, 1, ((v * w + a * dth) * sin(th + dth) + a * cos(th + dth) - v * w * sin(th) - a * cos(th)) / ((w + 1e-9) * (w + 1e-9)),
        // (-cos(th + dth) + cos(th)) / (w + 1e-9), (-dth * cos(th + dth) +sin(th + dth) - sin(th)) / ((w + 1e-9) * (w + 1e-9)),
        // ((-v - a * _dt) * cos(th + dth) - (-v * w - a * dth) * sin(th + dth) * _dt + a * _dt * cos(th + dth) + v * cos(th)) / ((w + 1e-9) * (w + 1e-9)) 
        // - 2 * ((-v * w - a * dth) * cos(th + dth) + a * sin(th + dth) + v * w * cos(th) - a * sin(th)) / ((w + 1e-9) * (w + 1e-9) * (w + 1e-9)), 
        //                     0, 0, 1, 0, 0, _dt,
        //                     0, 0, 0, 1, _dt, 0,
        //                     0, 0, 0, 0, 1, 0,
        //                     0, 0, 0, 0, 0, 1;
        _jacobianOplusXi << -1, 0, 0, 0, 0, 0,
                            0, -1, 0, 0, 0, 0,
                            0, 0, -1, 0, 0, 0;
    }

    virtual bool read(std::istream &in) override {return true;}

    virtual bool write(std::ostream &out) const override {return true;}

private:
    double _dt; 

};

} // namespace mytrk



#endif