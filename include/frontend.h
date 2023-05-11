#ifndef __SLAM_LITE_FRONTEND_H
#define __SLAM_LITE_FRONTEND_H

#include "backend.h"
#include "include/common_include.h"
#include "include/frame.h"
#include "include/map.h"
#include <opencv2/features2d.hpp>

namespace slamlite
{
// 枚举类，限定了枚举值的作用于，使用方法：FrontendStatus::INITING
enum class FrontendStatus
{
    INITING,
    TRACKING_GOOD,
    TRACKING_BAD,
    LOST
};
class Frontend
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Frontend> Ptr;

    // data
    FrontendStatus _status = FrontendStatus::INITING;
    Frame::Ptr _current_frame = static_cast<Frame::Ptr>(nullptr);
    Frame::Ptr _last_frame = static_cast<Frame::Ptr>(nullptr);

    Map::Ptr _map = static_cast<Map::Ptr>(nullptr);
    Backend::Ptr _backend = static_cast<Backend::Ptr>(nullptr);

    // 当前帧与上一帧的相对运动，用于估计当前帧pose初值
    SE3 _relative_motion;

    // 跟踪到的内点数量
    int _tracking_inliers = 0;

    // 参数
    int _num_features = 200;
    int _num_features_init = 100;
    int _num_features_tracking = 50;
    int _num_features_tracking_bad = 20;
    int _num_features_needed_for_keyframe = 80;

    // fast 特征点提取器
    cv::Ptr<cv::FastFeatureDetector> _fast_detector;
};
} // namespace slamlite
#endif