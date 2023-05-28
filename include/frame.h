#ifndef __SLAM_LITE_FRAME_H
#define __SLAM_LITE_FRAME_H

#include "common_include.h"
#include "feature.h"
#include "ORBextractor.h"

namespace slamlite {


struct Frame
{
  public:
    // eigen 内存对齐
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    // 重命名Frame指针
    typedef std::shared_ptr<Frame> Ptr;

    // 帧id
    unsigned long _id = 0;
    // 对应的关键帧id
    unsigned long _keyframe_id = 0;
    // 是否为关键帧
    bool _is_keyframe = false;
    // 时间戳
    double _time_stamp;
    // Tcw 位姿
    SE3 _pose;
    // pose 数据锁
    std::mutex _data_mutex;
    // 左右图像
    cv::Mat _img, _img_right;

    // 图像金字塔
    std::vector<cv::Mat> _imagePyramid;

    // 储存特征
    std::vector<std::shared_ptr<Feature>> _feature;
    // 图像金字塔对应的特征
    std::vector<std::vector<std::shared_ptr<Feature>>> _featurePyramid;
    // std::vector<std::shared_ptr<Feature>> _feature_right;

    public:
    Frame() {};

    Frame(long id,  double time_stamp, const SE3 &pose, const Mat &img, const Mat &right);

    SE3 GetPose();

    void SetPose(const SE3 &pose);

    void SetKeyframe();

    // 工厂构建模式，分配id
    static std::shared_ptr<Frame> CreateFrame();
};
}

#endif