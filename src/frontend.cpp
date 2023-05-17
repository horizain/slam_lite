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

} // namespace slamlite
