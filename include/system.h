#ifndef __SLAM_LITE_SYSTEM_H
#define __SLAM_LITE_SYSTEM_H

#include "common_include.h"
#include <cstdlib>
namespace slamlite {
    class system
    {
        public:
        enum _SensorType
        {
            MONOCULAR=0,
            STEREO,
            IMU_MONOCULAR,
            IMU_STEREO
        };

    };
}

#endif /* __SLAM_LITE_SYSTEM_H */
