#include "factor_curved_track/myframe.h"

namespace mytrk
{

myFrame::Ptr myFrame::CreateFrame()
{
    static long factory_id = 0;
    myFrame::Ptr new_frame(new myFrame);
    //简化了factory_id++,提高速度
    new_frame->id_ = factory_id++;
    return new_frame;
}

} // namespace mytrk

