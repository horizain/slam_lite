#ifndef __SLAM_LITE_FEATURE_H
#define __SLAM_LITE_FEATURE_H

#include "include/common_include.h"
#include "include/mapPoint.h"
#include <opencv2/features2d.hpp>

namespace slamlite
{
struct Feature
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Feature> Ptr;

    // 持有该feature的fream
    std::weak_ptr<Frame> _frame;
    cv::KeyPoint _point;
    // 对应的地图点
    std::weak_ptr<MapPoint> _map_point;

    bool _is_inlier = false;
    bool _is_on_right_img = false;

  public:
    Feature()
    {
    }

    Feature(std::shared_ptr<Frame> frame, const cv::KeyPoint &kp) : _frame(frame), _point(kp)
    {
    }
};
} // namespace slamlite

#endif