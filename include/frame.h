#ifndef __SLAM_LITE_FRAME_H
#define __SLAM_LITE_FRAME_H

#include "ORBextractor.h"
#include "camera.h"
#include "common_include.h"
#include "feature.h"
#include "mapPoint.h"
#include "setting.h"
#include <cstdint>
#include <opencv2/core/hal/interface.h>

namespace slamlite {

struct Frame {
public:
  // eigen 内存对齐
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  // 重命名Frame指针
  typedef std::shared_ptr<Frame> Ptr;

  static uint64_t _nextID;
  static uint64_t _nextKeyPointID;

  // 帧id
  uint64_t _id = 0;
  // 对应的关键帧id
  uint64_t _keyframe_id = 0;
  // 是否为关键帧
  bool _is_keyframe = false;
  // 时间戳
  double _time_stamp;

  double _thresholdDepth = 0;

  std::vector<MapPoint> _maps;

  cv::Mat _image;
  Camera::Ptr _camera;
  Feature _features;
  ORBextractor::Ptr _ORBextractor;
#ifdef STEREO
  cv::Mat _image_right;
  Camera::Ptr _camera_rightt;
  Feature _features_right;
  ORBextractor::Ptr _ORBextractor_right;

#endif
  // Tcw 位姿
  SE3 _pose;
  Mat33 _Rcw;
  Mat33 _Rwc;

  // pose 数据锁
  std::mutex _data_mutex;
  // 左右图像

  // 图像金字塔
  // std::vector<cv::Mat> _imagePyramid;

  // 储存特征
  // std::vector<Feature> _feature;
  // 图像金字塔对应的特征
  std::vector<std::vector<Feature>> _allFeatures;
  std::vector<int> _allFeaturesNum;
  // std::vector<std::shared_ptr<Feature>> _feature_right;

public:
  Frame(){};

  Frame(long id, double time_stamp, const SE3 &pose, const cv::Mat &img,
        const cv::Mat &right);

#ifdef STEREO
  Frame(const cv::Mat &image, const cv::Mat &image_right,
        const double &timeStamp, ORBextractor::Ptr orbextractor,
        ORBextractor::Ptr orbextractor_right, Camera::Ptr camera,
        Camera::Ptr camera_right, const double &thresholdDepth);

  void ComputeStereoMatches();
#endif
  Frame(const cv::Mat &image, const double &timeStamp,
        ORBextractor::Ptr orbextractor, Camera::Ptr camera,
        const double &thresholdDepth);
  // Frame(const cv::Mat &img, const SE3 &pose, )

  SE3 GetPose();

  void SetPose(const SE3 &pose);

  void ComputeORB(const cv::Mat &image);

  void SetKeyframe();

  // 工厂构建模式，分配id
  static std::shared_ptr<Frame> CreateFrame();
};
} // namespace slamlite

#endif