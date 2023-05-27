#ifndef __SLAM_LITE_ORBEXARACTOR_H
#define __SLAM_LITE_ORBEXARACTOR_H

#include <opencv2/features2d.hpp>
#include "common_include.h"
#include "feature.h"
#include "frame.h"

namespace slamlite
{
// 32 bit unsigned int, will have 8, 8x32=256
typedef std::vector<uint32_t> DescType;

class ORBextractor
{
  public:
    enum
    {
        FAST_SCORE = 1
    };

    // 图像金字塔层与层之间的缩放因子
    double _scaleFactor;
    // 图像金字塔的层数
    int _levels;
    // 储存图像金字塔每层的图像
    std::vector<cv::Mat> _imagePyramid;

    // std::weak_ptr<Frame> _frame;
    Frame::Ptr _frame = static_cast<Frame::Ptr>(nullptr);
    
    // 初始化的提取FAST响应值阈值
    int _initFASTThreshold = 40;
    // 最小的提取FAST响应值阈值
    int _minFASTThreshold;
    // 参数
    int _num_features = 200;
    int _num_features_init = 100;
    int _num_features_tracking = 50;
    int _num_features_tracking_bad = 20;
    int _num_features_needed_for_keyframe = 80;

    // 用于计算描述子的随机采样点集合
    std::vector<cv::Point> _pattern;

  public:
    ORBextractor();
    ORBextractor(int features, float scaleFactor, int levels, int initFASTThreshold, int minFASTThreshold);
    ~ORBextractor();

    int DetectFeatures();

    void operator()(cv::InputArray image, cv::InputArray mask, std::vector<cv::KeyPoint> &keypoints,
                    std::vector<DescType> &descriptors);

    bool ComputePyramid(cv::Mat &image);

    bool ComputeKeyPoints(std::vector<std::vector<cv::KeyPoint>> &allkeypoints);

    bool ComputeORB(const cv::Mat &img, std::vector<cv::KeyPoint> &keypoints, std::vector<DescType> &descriptors);

    static void ComputeDescriptor(const cv::Mat &image, const cv::KeyPoint &keypoint, DescType &descriptor);

    static int ComputeDescriptorDistance(const DescType &a, const DescType &b);

    inline bool Triangulation(SE3 &pose, SE3 &pose_right, Vec3 &point, Vec3 &point_right, Vec3 &point_world);
};
} // namespace slamlite

#endif