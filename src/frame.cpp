#include "include/frame.h"

namespace slamlite
{
    Frame::Frame(long id,  double time_stamp,
    const SE3 &pose, const Mat &img, const Mat &img_right)
    : _id(id), _time_stamp(time_stamp), _pose(pose),
    _img(img), _img_right(img_right) {}

    Frame::Ptr Frame::CreateFrame()
    {
        static int64_t factory_id = 0;
        Frame::Ptr new_frame(new Frame);
        new_frame->_id = ++factory_id;
        return new_frame;
    }

    void Frame::SetKeyframe()
    {
        static int64_t keyframe_factory_id = 0;
        _is_keyframe = true;
        _keyframe_id = keyframe_factory_id;
    }
} // namespace slamlite
