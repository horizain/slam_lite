#ifndef __SLAM_LITE_BACKEND_H
#define __SLAM_LITE_BACKEND_H

#include "common_include.h"
#include "frame.h"
#include "map.h"

namespace slamlite
{
    class Backend
    {
        public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Backend> Ptr;
    private:
        /* data */
    public:
        Backend(/* args */);
        ~Backend();
    };    
}

#endif