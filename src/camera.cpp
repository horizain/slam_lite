#include "camera.h"

namespace slamlite
{
Camera::Camera(double fx, double fy, double cx, double cy, double baseline, const SE3 &pose)
    : _fx(fx), _fy(fy), _cx(cx), _cy(cy), _baseline(baseline), _pose(pose)
{
    _pose_inv = _pose.inverse();
}

Vec3 Camera::world2camera(const Vec3 &p_w, const SE3 &T_c_w)
{
    return _pose * T_c_w * p_w;
}

Vec3 Camera::camera2world(const Vec3 &p_c, const SE3 &T_c_w)
{
    return T_c_w.inverse() * _pose_inv * p_c;
}
} // namespace slamlite
