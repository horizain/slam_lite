#include "frontend.h"
#include "config.h"
#include "feature.h"

namespace slamlite
{
Frontend::Frontend()
{
    _num_features_init = Config::Get<int>("num_features_init");
    _num_features = Config::Get<int>("num_features");
}

bool Frontend::AddFrame(Frame::Ptr frame)
{
    _current_frame = frame;
    _last_frame = _current_frame;
    return true;
}

bool Frontend::InsertKeyframe()
{
    if (_tracking_inliers >= _num_features_needed_for_keyframe)
        return false;
    _current_frame->SetKeyframe();
    
    // todo : map->insertKeyframe

    // todo : Log(...)
    return true;
}

void Frontend::SetObservationsForKeyframe()
{
    
}

inline bool Frontend::triangulation(SE3 &pose_reference, SE3 &pose_current, Vec3 &point_reference, Vec3 &point_current, Vec3 &point_world)
{
    Mat44 A;
    Vec4 b;
    b.setZero();
    auto m = pose_reference.matrix3x4();
    A.row(0) = point_reference[1] * pose_reference.matrix3x4().row(2) - pose_reference.matrix3x4().row(1);
    A.row(1) = point_reference[0] * pose_reference.matrix3x4().row(2) - pose_reference.matrix3x4().row(0);
    A.row(2) = point_current[1] * pose_current.matrix3x4().row(2) - pose_current.matrix3x4().row(1);
    A.row(3) = point_current[0] * pose_current.matrix3x4().row(2) - pose_current.matrix3x4().row(0);
    auto svd = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
    point_world = (svd.matrixV().col(3) / svd.matrixV()(3, 3)).head<3>();

    if (svd.singularValues()[3] / svd.singularValues()[2] < 1e-2)
    {
        return true;
    }
    return false;
}


} // namespace slamlite
