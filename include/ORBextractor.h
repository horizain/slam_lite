#ifndef __SLAM_LITE_ORBEXARACTOR_H
#define __SLAM_LITE_ORBEXARACTOR_H

#include "common_include.h"
#include "feature.h"
#include <opencv2/features2d.hpp>

namespace slamlite {
// 32 bit unsigned int, will have 8, 8x32=256
typedef std::vector<uint32_t> DescType;

class ORBextractor {
public:
public:
  ORBextractor();
  ~ORBextractor();

  int DetectFeatures();

  /**
   * @brief linear triangulation with SVD
   * 利用匹配点对和位姿可以三角化得到三维点
   */
  inline bool triangulation(SE3 &pose, SE3 &pose_right, Vec3 &point,
                            Vec3 &point_right, Vec3 &point_world);
};
} // namespace slamlite

#endif