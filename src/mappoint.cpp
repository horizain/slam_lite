#include "mapPoint.h"

namespace slamlite
{
    MapPoint::MapPoint(int64_t id, Vec3 position) : _id(id), _pos(position) {}

    MapPoint::Ptr MapPoint::CreateNewMapPoint()
    {
        static int64_t factory_id = 0;
        MapPoint::Ptr new_mappoint(new MapPoint);
        new_mappoint->_id = factory_id++;
        return new_mappoint;
    }

    void MapPoint::RemoveObservation(std::shared_ptr<Feature> feature)
    {
        std::unique_lock<std::mutex> lck(_data_mutex);
        for (auto iter = _observations.begin(); iter != _observations.end(); iter++)
        {
            if(iter->lock() == feature)
            {
                _observations.erase(iter);
                feature->_map_point.reset();
                --_observed_times;
                break;
            }
        }
    }
} // namespace slamlite
