#include "include/frontend.h"
#include "include/config.h"
#include "include/feature.h"

namespace slamlite
{
Frontend::Frontend()
{
    _fast_detector = cv::FastFeatureDetector::create();
    _num_features_init = Config::Get<int>("num_features_init");
    _num_features = Config::Get<int>("num_features");
}

int Frontend::DetectFeatures()
{
    std::vector<cv::KeyPoint> keypoints;
    cv::FAST(_current_frame->_img, keypoints, _initFASTThreshold, true);
    int cnt_detected = 0;
    for (auto &kp : keypoints)
    {
        _current_frame->_feature.push_back(
            Feature::Ptr(new Feature(_current_frame, kp))
        );
        ++cnt_detected;
    }
    return cnt_detected;
}

bool Frontend::AddFrame(Frame::Ptr frame)
{
    _current_frame = frame;
    _last_frame = _current_frame;
    int cnt = DetectFeatures();
    return true;
}

bool Frontend::InsertKeyframe()
{
    if (_tracking_inliers >= _num_features_needed_for_keyframe)
        return false;
    _current_frame->SetKeyframe();
    
    // todo : map->insertKeyframe

    // todo : Log(...)

}

void Frontend::SetObservationsForKeyframe()
{
    
}

inline bool triangulation(SE3 &pose_reference, SE3 &pose_current, Vec3 &point_reference, Vec3 &point_current, Vec3 &point_world)
{
    Mat44 A;
    Vec4 b;
    b.setZero();
    auto m = pose_reference.matrix3x4();
    A.row(0) = point_reference * pose_reference.matrix3x4().row(2);

    auto svd = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
    point_world = (svd.matrixV().col(3) / svd.matrixV()(3, 3)).head<3>();

    if (svd.singularValues()[3] / svd.singularValues()[2] < 1e-2)
    {
        return true;
    }
    return false;
}
} // namespace slamlite