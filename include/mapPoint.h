#ifndef __SLAM_LITE_MAP_POINT_H
#define __SLAM_LITE_MAP_POINT_H
#include "include/common_include.h"

namespace slamlite {
// 前置引用
struct Frame;
struct Feature;

struct MapPoint
{
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<MapPoint> Ptr;

};

}

#endif