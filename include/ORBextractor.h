#ifndef __SLAM_LITE_ORBEXARACTOR_H
#define __SLAM_LITE_ORBEXARACTOR_H

#include "common_include.h"
#include "feature.h"
#include "frame.h"
#include "setting.h"
#include <memory>
#include <opencv2/core/types.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>

namespace slamlite {

class ORBextractor {
public:
  enum { FAST_SCORE = 1 };
  typedef std::shared_ptr<ORBextractor> Ptr;

  // 图像金字塔层与层之间的缩放因子
  double _scaleFactor;
  // 图像金字塔的层数
  int _levels;
  // 储存图像金字塔每层的图像
  std::vector<cv::Mat> _perImagePyramid;
  // 每层储存的特征点个数
  std::vector<int> _perNumFeatures;

  // 灰度质心法图像块center到每一行的坐标边界的距离
  // *---------- u
  // |
  // |    c(center)
  // v
  std::vector<int> _icPatchUMaxs;

  // 初始化的提取FAST响应值阈值
  int _initFASTThreshold = 40;
  // 最小的提取FAST响应值阈值
  int _minFASTThreshold;
  // 参数
  int _numFeatures = 200;
  int _numFeaturesInit = 100;
  int _numFeaturesTracking = 50;
  int _numFeaturesTracking_bad = 20;
  int _numFeaturesNeededForKeyframe = 80;

  // 每层图像的缩放因子
  std::vector<double> _perScaleFactor;
  // 每层缩放因子的倒数
  std::vector<double> _perInvScaleFactor;
  // 每层的sigma^2，缩放因子的平方
  std::vector<double> _perScale2;
  // 每层缩放因子的平方的倒数
  std::vector<double> _perInvScale2;

public:
  ORBextractor();
  ORBextractor(int numFeatures, double scaleFactor, int levels,
               int initFASTThreshold, int minFASTThreshold);
  ~ORBextractor();

  int DetectFeatures();

  void operator()(cv::InputArray _image, Feature features);

  /**
   * @brief 未完成
   *
   * @param image
   * @return true
   * @return false
   */
  bool ComputePyramid(const cv::Mat &image);

  /**
   * @brief 仅实现提取角点功能，没有分布均衡处理
   *
   * @param allkeypoints
   * @return true
   * @return false
   */
  bool ComputeKeyPoints(std::vector<std::vector<cv::KeyPoint>> &allkeypoints);

  void ComputeDescriptor(const cv::Mat &image, const cv::KeyPoint &keypoint,
                         DescType &descriptor);

  void ComputeDescriptors(const cv::Mat &image,
                          const std::vector<cv::KeyPoint> &keypoints,
                          std::vector<DescType> &descriptors);

  void ComputeOrientation(const cv::Mat &image,
                          std::vector<cv::KeyPoint> &keypoints,
                          const std::vector<int> u_max);

  double ComputeAngle(const cv::Mat &image, const cv::Point2f &pt);

  void ComputeOrientation(const cv::Mat &image,
                          std::vector<cv::KeyPoint> &keypoints);

  inline int ComputeDescriptorDistance(const DescType &a, const DescType &b);
};
} // namespace slamlite

#endif