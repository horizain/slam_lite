#ifndef __SLAM_LITE_MAP_POINT_H
#define __SLAM_LITE_MAP_POINT_H
#include "include/common_include.h"
#include "include/feature.h"

namespace slamlite {
// 前置引用
struct Frame;
struct Feature;

/**
 * @brief 路标点类
 * 特征点在三角化后形成路标点
 * 
 */
struct MapPoint
{
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<MapPoint> Ptr;

    uint64_t _id = 0;
    bool _is_inlier = true;
    Vec3 _pos = Vec3::Zero();
    // 数据锁
    std::mutex _data_mutex;

    int _observed_times = 0;
    std::list<std::weak_ptr<Feature>> _observations;

    public:
    MapPoint() {}

    MapPoint(int64_t id, Vec3 position);

    Vec3 Pos() {
        std::unique_lock<std::mutex> lck(_data_mutex);
        return _pos;
    }

    void SetPos(const Vec3 &pos)
    {
        std::unique_lock<std::mutex> lck(_data_mutex);
        _pos = pos;
    }

    void AddObservation(std::shared_ptr<Feature> feature)
    {
        std::unique_lock<std::mutex> lck(_data_mutex);
        _observations.push_back(feature);
        ++_observed_times;
    }

    void RemoveObservation(std::shared_ptr<Feature> feature);

    std::list<std::weak_ptr<Feature>> GetObs()
    {
        std::unique_lock<std::mutex> lck(_data_mutex);
        return _observations;
    }

    static MapPoint::Ptr CreateNewMapPoint();


};

}

#endif