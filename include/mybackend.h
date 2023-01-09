#pragma once
#ifndef MYTRK_BACKEND_H
#define MYTRK_BACKEND_H

#include "mycommon_include.h"
#include "myfrontend.h"

namespace mytrk
{
class myFrontend;

class myBackend
{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::unique_ptr<myBackend> Ptr;
    typedef std::unordered_map<unsigned long, myFrontend::Ptr> Frontendtype;
    typedef std::unordered_map<unsigned long, Vec3> Vec3list;

    void InitFrontend();

    void UpdateObjState(unsigned long id, Vec7 od_reault);


private:

    unsigned long num_of_obj = 0;

    Frontendtype obj_list_;
    Vec3list predict_postion_list_;
    Vec3list obj_size_list_;


};


} // namespace mytrk





#endif