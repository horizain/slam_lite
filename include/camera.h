#ifndef __SLAM_LITE_CAMERA_H
#define __SLAM_LITE_CAMERA_H

#include "include/common_include.h"

namespace slamlite
{
    class Camera
    {
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Camera> Ptr;
    
    double _fx = 0;
    double _fy = 0;
    double _cx = 0;
    double _cy = 0;
    double _baseline = 0;

    // 位姿
    SE3 _pose;
    SE3 _pose_inv;

    public:
        Camera();
        Camera(double fx, double fy, double cx, double cy, double baseline, const SE3 &pose);
        ~Camera();

        SE3 pose() const;

        Mat33 K() const;

        // 常用的几种坐标转换
        /**
         * @brief 世界坐标系下的3d点转到相机坐标系
         * 
         * @param p_w 世界坐标系下的3d点
         * @param T_c_w 世界坐标系到相机坐标系的变换矩阵
         * @return Vec3 相机坐标系下的3d点
         */
        Vec3 world2camera(const Vec3 &p_w, const SE3 &T_c_w);

        /**
         * @brief 相机坐标系下的3d点转到世界坐标系
         * 
         * @param p_c 相机坐标系下的3d点
         * @param T_c_w 
         * @return Vec3 
         */
        Vec3 camera2world(const Vec3 &p_c, const SE3 &T_c_w);

        /**
         * @brief 相机坐标系下的3d点转到像素坐标系
         * 
         * @param p_c 
         * @return Vec3 
         */
        Vec3 camera2pixel(const Vec3 &p_c);

        /**
         * @brief 像素坐标系下的2d点转到相机坐标系
         * 
         * @param p_p 
         * @param depth 
         * @return Vec3 
         */
        Vec3 pixel2camera(const Vec2 &p_p, double depth = 1);

        /**
         * @brief 像素坐标系下的2d点转到世界坐标系
         * 
         * @param p_p 
         * @param T_c_w 
         * @param depth 
         * @return Vec3 
         */
        Vec3 pixel2world(const Vec2 &p_p, const SE3 &T_c_w, double depth = 1);

        /**
         * @brief 世界坐标系下的3d点转到像素坐标系
         * 
         * @param p_w 
         * @param T_c_w 
         * @return Vec3 
         */
        Vec3 world2pixel(const Vec3 &p_w, const SE3 &T_c_w);

    };    
} // namespace slamlite


#endif