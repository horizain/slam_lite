#ifndef __SLAM_LITE_MAP_H
#define __SLAM_LITE_MAP_H

#include "include/common_include.h"
#include "include/frame.h"
#include "include/mapPoint.h"
#include <unordered_map>

namespace slamlite
{

class Map
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Map> Ptr;
    typedef std::unordered_map<uint64_t, MapPoint::Ptr> LandmarksType;
    typedef std::unordered_map<uint64_t, Frame::Ptr> KeyframesType;

  private:
    // 数据锁
    std::mutex _data_mutex;
    // 所有的路标点
    LandmarksType _landmarks;
    // 激活的路标点
    LandmarksType _active_landmarks;
    // 所有的关键帧
    KeyframesType _keyframes;
    // 激活的关键帧
    KeyframesType _active_keyframes;

    Frame::Ptr _current_frame = static_cast<Frame::Ptr>(nullptr);

    // 激活的关键帧数量
    int _num_active_keyframes = 7;

  public:
    Map(/* args */){};
    ~Map(){};
};

} // namespace slamlite

#endif