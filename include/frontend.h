#ifndef __SLAM_LITE_FRONTEND_H
#define __SLAM_LITE_FRONTEND_H

#include "backend.h"
#include "camera.h"
#include "common_include.h"
#include "frame.h"
#include "map.h"
#include <opencv2/features2d.hpp>

namespace slamlite {
// 枚举类，限定了枚举值的作用于，使用方法：FrontendStatus::INITING
enum class FrontendStatus { INITING, TRACKING_GOOD, TRACKING_BAD, LOST };

class Frontend {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  typedef std::shared_ptr<Frontend> Ptr;

  // data
  FrontendStatus _status = FrontendStatus::INITING;
  Frame::Ptr _current_frame = static_cast<Frame::Ptr>(nullptr);
  Frame::Ptr _last_frame = static_cast<Frame::Ptr>(nullptr);

  Camera::Ptr _camera = static_cast<Camera::Ptr>(nullptr);
  Camera::Ptr _camera_right = static_cast<Camera::Ptr>(nullptr);

  Map::Ptr _map = static_cast<Map::Ptr>(nullptr);
  Backend::Ptr _backend = static_cast<Backend::Ptr>(nullptr);

  // 当前帧与上一帧的相对运动，用于估计当前帧pose初值
  SE3 _relative_motion;

  // 跟踪到的内点数量
  int _tracking_inliers = 0;

public:
  Frontend();

  /**
   * @brief 外部接口，添加一个帧并计算其定位结果
   *
   * @param frame
   * @return true
   * @return false
   */
  bool AddFrame(Frame::Ptr frame);

  /**
   * @brief Get the Status object
   *
   * @return FrontendStatus
   */
  FrontendStatus GetStatus() const { return _status; }

private:
  /**
   * @brief 跟踪普通帧
   *
   * @return true
   * @return false
   */
  bool Track();

  /**
   * @brief 重置前端系统
   *
   * @return true
   * @return false
   */
  bool Reset();

  /**
   * @brief linear triangulation with SVD
   * 利用匹配点对和位姿可以三角化得到三维点
   */
  inline bool Triangulation(SE3 &pose_reference, SE3 &pose_current,
                            Vec3 &point_reference, Vec3 &point_current,
                            Vec3 &point_world);

  bool TrackLastFrame();

  int EstimateCurrentPose();

  bool InsertKeyframe();

  int FeatureMatch(Frame &frame_reference, Frame &frame_current,
                   std::vector<int> matchFeatureIndexs, int windowsSize);

  bool SteroInit();

  int TriangulateNewPoints();

  void SetObservationsForKeyframe();
};
} // namespace slamlite
#endif