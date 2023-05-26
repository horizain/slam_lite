#include "ORBextractor.h"

namespace slamlite {
ORBextractor::ORBextractor() { ; }

ORBextractor::~ORBextractor() {}

int ORBextractor::DetectFeatures() {
  std::vector<cv::KeyPoint> keypoints;
  cv::FAST(_current_frame->_img, keypoints, _initFASTThreshold, true);
  int cnt_detected = 0;
  for (auto &kp : keypoints) {
    _current_frame->_feature.push_back(
        Feature::Ptr(new Feature(_current_frame, kp)));
    ++cnt_detected;
  }
  return cnt_detected;
}

inline bool ORBextractor::triangulation(SE3 &pose_reference, SE3 &pose_current,
                                        Vec3 &point_reference,
                                        Vec3 &point_current,
                                        Vec3 &point_world) {
  Mat44 A;
  Vec4 b;
  b.setZero();
  auto m = pose_reference.matrix3x4();
  A.row(0) = point_reference[1] * pose_reference.matrix3x4().row(2) -
             pose_reference.matrix3x4().row(1);
  A.row(1) = point_reference[0] * pose_reference.matrix3x4().row(2) -
             pose_reference.matrix3x4().row(0);
  A.row(2) = point_current[1] * pose_current.matrix3x4().row(2) -
             pose_current.matrix3x4().row(1);
  A.row(3) = point_current[0] * pose_current.matrix3x4().row(2) -
             pose_current.matrix3x4().row(0);
  auto svd = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
  point_world = (svd.matrixV().col(3) / svd.matrixV()(3, 3)).head<3>();

  if (svd.singularValues()[3] / svd.singularValues()[2] < 1e-2) {
    return true;
  }
  return false;
}
} // namespace slamlite