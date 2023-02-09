#ifndef MYTRK_G2O_H
#define MYTRK_G2O_H

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

namespace mytrk
{

class VertexXYThta : public g2o::BaseVertex<3, Vec3>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    
    virtual void setToOriginImpl() override
    {
        _estimate = Vec3::Zero();
    }

    virtual void oplusImpl(const double *update) override
    {
        _estimate[0] += update[0];
        _estimate[1] += update[1];
        _estimate[2] += update[2];
    }

    virtual bool read(std::istream &in) override {return true;}

    virtual bool write(std::ostream &out) const override {return true;}

};

class VertexVAOmg : public g2o::BaseVertex<3, Vec3>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    
    virtual void setToOriginImpl() override
    {
        _estimate = Vec3::Zero();
    }

    virtual void oplusImpl(const double *update) override
    {
        _estimate[0] += update[0];
        _estimate[1] += update[1];
        _estimate[2] += update[2];
    }

    virtual bool read(std::istream &in) override {return true;}

    virtual bool write(std::ostream &out) const override {return true;}

};

class EdgePositon : public g2o::BaseMultiEdge<2, Vec2>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgePositon() {}

    virtual void computeError() override
    {
        const VertexXYThta *v0 = static_cast<VertexXYThta *> (_vertices[0]);
        const VertexXYThta *v1 = static_cast<VertexXYThta *> (_vertices[1]);
        const VertexVAOmg *v2 = static_cast<VertexVAOmg *> (_vertices[2]);
        const VertexVAOmg *v3 = static_cast<VertexVAOmg *> (_vertices[3]);


        
    }

    G2O_MAKE_AUTO_AD_FUNCTIONS;



};

} // namespace mytrk    


#endif