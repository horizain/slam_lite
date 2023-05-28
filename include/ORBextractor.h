#ifndef __SLAM_LITE_ORBEXARACTOR_H
#define __SLAM_LITE_ORBEXARACTOR_H

#include "common_include.h"
#include "feature.h"
#include "frame.h"
#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>

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

    // 每层图像的缩放因子
    std::vector<double> _perLevelScaleFactor;
    // 每层缩放因子的倒数
    std::vector<double> _perLevelInvScaleFactor;
    // 每层的sigma^2，缩放因子的平方
    std::vector<double> _perLevelSimga2;
    // 每层缩放因子的平方的倒数
    std::vector<double> _perLevelInvSimga2;

  public:
    ORBextractor();
    ORBextractor(int features, double scaleFactor, int levels, int initFASTThreshold, int minFASTThreshold);
    ~ORBextractor();

    int DetectFeatures();

    void operator()(cv::InputArray _image, std::vector<std::vector<cv::KeyPoint>> &allkeypoints,
                    std::vector<std::vector<DescType>> &alldescriptors);

    /**
     * @brief 未完成
     *
     * @param image
     * @return true
     * @return false
     */
    bool ComputePyramid(cv::Mat &image);

    /**
     * @brief 仅实现提取角点功能，没有分布均衡处理
     *
     * @param allkeypoints
     * @return true
     * @return false
     */
    bool ComputeKeyPoints(std::vector<std::vector<cv::KeyPoint>> &allkeypoints);

    /**
     * @brief
     *
     * @param img
     * @param keypoints
     * @param descriptors
     * @return true
     * @return false
     */
    bool ComputeORB(const cv::Mat &img, std::vector<std::vector<cv::KeyPoint>> &allkeypoints,
                    std::vector<DescType> &descriptors);

    void ComputeDescriptor(const cv::Mat &image, const cv::KeyPoint &keypoint, DescType &descriptor);

    void ComputeDescriptors(const cv::Mat &image, const std::vector<cv::KeyPoint> &keypoints,
                            std::vector<DescType> &descriptors);

    void ComputeOrientation(const cv::Mat &image, std::vector<cv::KeyPoint> &keypoints, const std::vector<int> u_max);

    double ComputeAngle(const cv::Mat &image, const cv::Point2f &pt, const std::vector<int> &u_max);

    inline int ComputeDescriptorDistance(const DescType &a, const DescType &b);

    inline bool Triangulation(SE3 &pose, SE3 &pose_right, Vec3 &point, Vec3 &point_right, Vec3 &point_world);
};
} // namespace slamlite

#endif