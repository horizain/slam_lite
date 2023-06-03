#ifndef __SLAM_LITE_FEATURE_H
#define __SLAM_LITE_FEATURE_H

#include "common_include.h"
#include "mapPoint.h"
#include <opencv2/core/types.hpp>
#include <opencv2/features2d.hpp>

namespace slamlite {
// 32 bit unsigned int, will have 8, 8x32=256
typedef std::vector<uint32_t> DescType;

struct Feature {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  typedef std::shared_ptr<Feature> Ptr;

  // 持有该feature的fream
  std::weak_ptr<Frame> _frame;
  cv::KeyPoint _point;
  // 对应的地图点
  std::weak_ptr<MapPoint> _map_point;
  // 图像金字塔每层的特征点和描述子
  std::vector<std::vector<cv::KeyPoint>> _keypoints;
  std::vector<std::vector<DescType>> _descriptors;
  // 特征点的深度信息
  std::vector<double> _depths;

  bool _is_inlier = false;
  bool _is_on_right_img = false;
  bool _is_good = false;

public:
  Feature() {}

  Feature(std::shared_ptr<Frame> frame, const cv::KeyPoint &kp)
      : _frame(frame), _point(kp) {}
};
} // namespace slamlite

#endif