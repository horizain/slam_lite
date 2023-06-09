#include "frame.h"
#include <cstdint>
#include <opencv2/core/mat.hpp>
#include <thread>

namespace slamlite {
// 初始化类的静态成员变量
uint64_t Frame::_nextID = 0;
uint64_t Frame::_nextKeyPointID = 0;

Frame::Frame(long id, double time_stamp, const SE3 &pose, const cv::Mat &img,
             const cv::Mat &img_right)
    : _id(id), _timeStamp(time_stamp), _pose(pose), _image(img),
      _image_right(img_right) {}

Frame::Frame(const cv::Mat &image, const cv::Mat &image_right,
             const double &timeStamp, ORBextractor::Ptr orbextractor,
             ORBextractor::Ptr orbextractor_right, Camera::Ptr camera,
             Camera::Ptr camera_right, const double &thresholdDepth)
    : _image(image), _image_right(image_right), _timeStamp(timeStamp),
      _ORBextractor(orbextractor), _ORBextractor_right(orbextractor_right),
      _camera(camera), _camera_rightt(camera_right),
      _thresholdDepth(thresholdDepth) {
  _isStrero = true;
  std::thread threadORB(&Frame::ComputeORB, this, _image);
  std::thread threadORBRight(&Frame::ComputeORB, this, _image_right);
  threadORB.join();
  threadORBRight.join();

  // 获取特征点的个数
  for (int i = 0; i < _allFeatures.size(); ++i) {
    _allFeaturesNum[i] = _allFeatures[i].size();
  }
  if (_allFeatures.empty())
    return;
}

//
void Frame::ComputeStereoMatches() {}

Frame::Ptr Frame::CreateFrame() {
  static int64_t factory_id = 0;
  Frame::Ptr new_frame(new Frame);
  new_frame->_id = ++factory_id;
  return new_frame;
}

void Frame::ComputeORB(const cv::Mat &image) {
  (*_ORBextractor)(image, _features);
  if (_isStrero == true)
    (*_ORBextractor_right)(_image, _features_right);
}

void Frame::SetKeyframe() {
  _isKeyframe = true;
  _keyframeID = ++_nextKeyPointID;
}
} // namespace slamlite
