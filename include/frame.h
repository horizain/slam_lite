#ifndef __SLAM_LITE_FRAME_H
#define __SLAM_LITE_FRAME_H

#include "include/common_include.h"

namespace slamlite {


struct Frame
{
  public:
    // eigen 内存对齐
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    // 重命名Frame指针
    typedef std::shared_ptr<Frame> Ptr;

    // 帧id
    unsigned long _id = 0;
    // 对应的关键帧id
    unsigned long _keyframe_id = 0;
    // 是否为关键帧
    bool _is_keyframe = false;
    // 时间戳
    double _time_stamp;
    // Tcw 位姿
    SE3 _pose;
    // pose 数据锁
    std::mutex _pose_mutex;

    cv::Mat _img, _img_right;



};
}

#endif