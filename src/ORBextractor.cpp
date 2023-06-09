#include "ORBextractor.h"

#include <cmath>
#include <opencv2/core.hpp>
#include <opencv2/core/base.hpp>
#include <opencv2/core/fast_math.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

namespace slamlite {

// 31 = 15 * 2 + 1(CENTER)
const static int IC_PATCH_SIZE = 31;      // 灰度质心法的图像块直径
const static int IC_HALF_PATCH_SIZE = 15; // 图像块半径
// 19 = 15(IC_HALF_PATCH_SIZE) + 3(EDGE) + 1(CENTER)
const static int IC_EDGE_THRESHOLD = 19; // 综合考虑边界元素的图像边
const static int FAST_EDGE_THRESHOLD = 3; // 计算角点的图像边界

ORBextractor::ORBextractor(int features, double scaleFactor, int levels,
                           int initFASTThreshold, int minFASTThreshold)
    : _numFeatures(features), _scaleFactor(scaleFactor), _levels(levels),
      _initFASTThreshold(initFASTThreshold),
      _minFASTThreshold(minFASTThreshold) {
  /******************* 图像金字塔 *********************/
  // 根据金字塔层数进行初始化
  _perScaleFactor.reserve(_levels);
  _perScale2.reserve(_levels);
  _perInvScaleFactor.reserve(_levels);
  _perInvScale2.reserve(_levels);

  // 对初始图像（第0层），这些参数都是1
  _perScaleFactor[0] = 1.0f;
  _perScale2[0] = 1.0f;
  _perInvScaleFactor[0] = 1.0f;
  _perInvScale2[0] = 1.0f;

  // sigma^2就是每层图像相对于初始图像缩放因子的平方
  for (int i = 1; i < _levels; ++i) {
    _perScaleFactor[i] = _perScaleFactor[i - 1] * _scaleFactor;
    _perScale2[i] = _perScale2[i] = _perScale2[i];
    _perInvScaleFactor[i] = 1.0f / _perScaleFactor[i];
    _perInvScale2[i] = 1.0f / _perScale2[i];
  }

  _perImagePyramid.resize(_levels);
  _perNumFeatures.resize(_levels);
  /********************* 每层特征点 *********************/
  // 图像降采样缩放系数的倒数
  double factor = 1.0f / _scaleFactor;
  // 第0层图像应该分配的特征点数量
  double levelZeroFeatures = _numFeatures * (1 - factor) /
                             (1 - (double)pow((factor), (double)_levels));
  // 待分配特征点总数
  int sumFeatures = 0;
  // 逐层计算待分配特征点个数
  for (int level; level < _levels - 1; ++level) {
    // 返回最近整数值
    _perNumFeatures[level] = cvRound(levelZeroFeatures);
    // 累计总数
    sumFeatures += _perNumFeatures[level];
    // 计算下一层
    levelZeroFeatures *= factor;
  }
  // 把分配剩下的特征点个数分配到顶层
  _perNumFeatures[level - 1] = std::max(_numFeatures - suimFeatures, 0);

  /******************** 计算特征点方向 ***********************/
  // 初始化灰度质心法算法所需变量
  // +1中的1表示那个圆的中间行
  _icPatchUMaxs.resize(IC_HALF_PATCH_SIZE + 1);
  // 45时的v的坐标
  int vmax = cvFloor(IC_HALF_PATCH_SIZE * sqrt(2.f) / 2.f + 1);
  // 先计算1/4个圆
  const double halfPatch2 = IC_HALF_PATCH_SIZE * IC_HALF_PATCH_SIZE;

  for (int v = 0; v <= vmax; ++v) {
    // v 从0开始增加，意味着_icPatchUMaxs[0]为Umax
    _icPatchUMaxs[v] = cvRound(sqrt(halfPatch2 - v * v));
  }
  // 反过来对称着计算剩下1/4个圆
  int vmin = cvCeil(IC_HALF_PATCH_SIZE * sqrt(2.f) / 2.f);
  // 这段代码来自ORB SLAM，算法原理是什么还没搞清楚 -- 20230529
  for (int v = IC_HALF_PATCH_SIZE, v0 = 0; v >= vmin; --v) {
    while (_icPatchUMaxs[v0] == _icPatchUMaxs[v0 + 1])
      ++v0;
    _icPatchUMaxs[v] = v0;
    ++v0;
  }
}

ORBextractor::ORBextractor() {}

ORBextractor::~ORBextractor() {}

/**
 * @brief
 *
 * @param image
 * @return true
 * @return false
 */
bool ORBextractor::ComputePyramid(const cv::Mat &image) {
  for (int level = 0; level < _levels; ++level) {
    double scale = _perScaleFactor[level];
    cv::Size sz(cvRound((double)image.cols * scale),
                cvRound((double)image.rows * scale));
    cv::Size wholeSize(sz.width + FAST_EDGE_THRESHOLD * 2,
                       sz.height + FAST_EDGE_THRESHOLD * 2);
    cv::Mat tempMat(wholeSize, image.type());
    _perImagePyramid[level] = tempMat(cv::Rect(
        FAST_EDGE_THRESHOLD, FAST_EDGE_THRESHOLD, sz.width, sz.height));
    if (level != 0) {
      // image.copyTo(_perImagePyramid[level]);
      // _perImagePyramid[level] = tempMat;
    } else {
      // image.copyTo(_perImagePyramid[level]);
      // _perImagePyramid[level] = tempMat;
      cv::copyMakeBorder(image, tempMat, FAST_EDGE_THRESHOLD,
                         FAST_EDGE_THRESHOLD, FAST_EDGE_THRESHOLD,
                         FAST_EDGE_THRESHOLD, cv::BORDER_REFLECT_101);
    }
  }
  return true;
}

bool ORBextractor::ComputeKeyPoints(
    std::vector<std::vector<cv::KeyPoint>> &allkeypoints) {
  allkeypoints.resize(_levels);
  for (int level = 0; level < _levels; ++level) {
    const int minBorderX = IC_EDGE_THRESHOLD - FAST_EDGE_THRESHOLD;
    const int maxBorderX =
        _perImagePyramid[level].cols - IC_EDGE_THRESHOLD + FAST_EDGE_THRESHOLD;
    const int minBorderY = minBorderX;
    const int maxBorderY =
        _perImagePyramid[level].rows - IC_EDGE_THRESHOLD + FAST_EDGE_THRESHOLD;
    cv::Mat roi = _perImagePyramid[level](cv::Range(minBorderY, maxBorderY),
                                          cv::Range(minBorderX, maxBorderX));
    FAST(roi, allkeypoints.at(level), _initFASTThreshold, true);
    for (auto kp : allkeypoints.at(level)) {
      kp.octave = level;
    }
    ComputeOrientation(_perImagePyramid[level], allkeypoints[level]);
  }
  return true;
}

double ORBextractor::ComputeAngle(const cv::Mat &image, const cv::Point2f &pt) {
  // 图像的矩，前者按照图像块的y轴加权，后者按照x轴加权
  int m01 = 0, m10 = 0;
  const uchar *center = &image.at<uchar>(cvRound(pt.y), cvRound(pt.x));
  // 先计算v=center.y 即中心线的情况
  for (int u = -IC_HALF_PATCH_SIZE; u <= IC_HALF_PATCH_SIZE; ++u) {
    m10 += u * center[u];
  }
  // 获取整个图像一行有多少个元素，用于访问圆形区域内的下一行
  int step = (int)image.step1();
  for (int v = 1; v <= IC_HALF_PATCH_SIZE; ++v) {
    int v_sum = 0;
    int d = _icPatchUMaxs[v];
    // 对于每一个u，一次处理两个点
    for (int u = -d; u <= d; ++u) {
      // 中心线上面元素的灰度值
      int v_ip = center[u + v * step];
      // 中心线下面元素的灰度值
      int v_bottom = center[u - v * step];
      v_sum += (v_ip - v_bottom);
      m10 += u * (v_ip + v_bottom);
    }
    m01 += v * v_sum;
  }
  return cv::fastAtan2((float)m01, (float)m10);
}

void ORBextractor::ComputeOrientation(const cv::Mat &image,
                                      std::vector<cv::KeyPoint> &keypoints) {
  for (auto &kp : keypoints)
    kp.angle = ComputeAngle(image, kp.pt);
}

// ORB pattern
static int _ORB_pattern[256 * 4] = {
    8,   -3,  9,   5 /*mean (0), correlation (0)*/,
    4,   2,   7,   -12 /*mean (1.12461e-05), correlation (0.0437584)*/,
    -11, 9,   -8,  2 /*mean (3.37382e-05), correlation (0.0617409)*/,
    7,   -12, 12,  -13 /*mean (5.62303e-05), correlation (0.0636977)*/,
    2,   -13, 2,   12 /*mean (0.000134953), correlation (0.085099)*/,
    1,   -7,  1,   6 /*mean (0.000528565), correlation (0.0857175)*/,
    -2,  -10, -2,  -4 /*mean (0.0188821), correlation (0.0985774)*/,
    -13, -13, -11, -8 /*mean (0.0363135), correlation (0.0899616)*/,
    -13, -3,  -12, -9 /*mean (0.121806), correlation (0.099849)*/,
    10,  4,   11,  9 /*mean (0.122065), correlation (0.093285)*/,
    -13, -8,  -8,  -9 /*mean (0.162787), correlation (0.0942748)*/,
    -11, 7,   -9,  12 /*mean (0.21561), correlation (0.0974438)*/,
    7,   7,   12,  6 /*mean (0.160583), correlation (0.130064)*/,
    -4,  -5,  -3,  0 /*mean (0.228171), correlation (0.132998)*/,
    -13, 2,   -12, -3 /*mean (0.00997526), correlation (0.145926)*/,
    -9,  0,   -7,  5 /*mean (0.198234), correlation (0.143636)*/,
    12,  -6,  12,  -1 /*mean (0.0676226), correlation (0.16689)*/,
    -3,  6,   -2,  12 /*mean (0.166847), correlation (0.171682)*/,
    -6,  -13, -4,  -8 /*mean (0.101215), correlation (0.179716)*/,
    11,  -13, 12,  -8 /*mean (0.200641), correlation (0.192279)*/,
    4,   7,   5,   1 /*mean (0.205106), correlation (0.186848)*/,
    5,   -3,  10,  -3 /*mean (0.234908), correlation (0.192319)*/,
    3,   -7,  6,   12 /*mean (0.0709964), correlation (0.210872)*/,
    -8,  -7,  -6,  -2 /*mean (0.0939834), correlation (0.212589)*/,
    -2,  11,  -1,  -10 /*mean (0.127778), correlation (0.20866)*/,
    -13, 12,  -8,  10 /*mean (0.14783), correlation (0.206356)*/,
    -7,  3,   -5,  -3 /*mean (0.182141), correlation (0.198942)*/,
    -4,  2,   -3,  7 /*mean (0.188237), correlation (0.21384)*/,
    -10, -12, -6,  11 /*mean (0.14865), correlation (0.23571)*/,
    5,   -12, 6,   -7 /*mean (0.222312), correlation (0.23324)*/,
    5,   -6,  7,   -1 /*mean (0.229082), correlation (0.23389)*/,
    1,   0,   4,   -5 /*mean (0.241577), correlation (0.215286)*/,
    9,   11,  11,  -13 /*mean (0.00338507), correlation (0.251373)*/,
    4,   7,   4,   12 /*mean (0.131005), correlation (0.257622)*/,
    2,   -1,  4,   4 /*mean (0.152755), correlation (0.255205)*/,
    -4,  -12, -2,  7 /*mean (0.182771), correlation (0.244867)*/,
    -8,  -5,  -7,  -10 /*mean (0.186898), correlation (0.23901)*/,
    4,   11,  9,   12 /*mean (0.226226), correlation (0.258255)*/,
    0,   -8,  1,   -13 /*mean (0.0897886), correlation (0.274827)*/,
    -13, -2,  -8,  2 /*mean (0.148774), correlation (0.28065)*/,
    -3,  -2,  -2,  3 /*mean (0.153048), correlation (0.283063)*/,
    -6,  9,   -4,  -9 /*mean (0.169523), correlation (0.278248)*/,
    8,   12,  10,  7 /*mean (0.225337), correlation (0.282851)*/,
    0,   9,   1,   3 /*mean (0.226687), correlation (0.278734)*/,
    7,   -5,  11,  -10 /*mean (0.00693882), correlation (0.305161)*/,
    -13, -6,  -11, 0 /*mean (0.0227283), correlation (0.300181)*/,
    10,  7,   12,  1 /*mean (0.125517), correlation (0.31089)*/,
    -6,  -3,  -6,  12 /*mean (0.131748), correlation (0.312779)*/,
    10,  -9,  12,  -4 /*mean (0.144827), correlation (0.292797)*/,
    -13, 8,   -8,  -12 /*mean (0.149202), correlation (0.308918)*/,
    -13, 0,   -8,  -4 /*mean (0.160909), correlation (0.310013)*/,
    3,   3,   7,   8 /*mean (0.177755), correlation (0.309394)*/,
    5,   7,   10,  -7 /*mean (0.212337), correlation (0.310315)*/,
    -1,  7,   1,   -12 /*mean (0.214429), correlation (0.311933)*/,
    3,   -10, 5,   6 /*mean (0.235807), correlation (0.313104)*/,
    2,   -4,  3,   -10 /*mean (0.00494827), correlation (0.344948)*/,
    -13, 0,   -13, 5 /*mean (0.0549145), correlation (0.344675)*/,
    -13, -7,  -12, 12 /*mean (0.103385), correlation (0.342715)*/,
    -13, 3,   -11, 8 /*mean (0.134222), correlation (0.322922)*/,
    -7,  12,  -4,  7 /*mean (0.153284), correlation (0.337061)*/,
    6,   -10, 12,  8 /*mean (0.154881), correlation (0.329257)*/,
    -9,  -1,  -7,  -6 /*mean (0.200967), correlation (0.33312)*/,
    -2,  -5,  0,   12 /*mean (0.201518), correlation (0.340635)*/,
    -12, 5,   -7,  5 /*mean (0.207805), correlation (0.335631)*/,
    3,   -10, 8,   -13 /*mean (0.224438), correlation (0.34504)*/,
    -7,  -7,  -4,  5 /*mean (0.239361), correlation (0.338053)*/,
    -3,  -2,  -1,  -7 /*mean (0.240744), correlation (0.344322)*/,
    2,   9,   5,   -11 /*mean (0.242949), correlation (0.34145)*/,
    -11, -13, -5,  -13 /*mean (0.244028), correlation (0.336861)*/,
    -1,  6,   0,   -1 /*mean (0.247571), correlation (0.343684)*/,
    5,   -3,  5,   2 /*mean (0.000697256), correlation (0.357265)*/,
    -4,  -13, -4,  12 /*mean (0.00213675), correlation (0.373827)*/,
    -9,  -6,  -9,  6 /*mean (0.0126856), correlation (0.373938)*/,
    -12, -10, -8,  -4 /*mean (0.0152497), correlation (0.364237)*/,
    10,  2,   12,  -3 /*mean (0.0299933), correlation (0.345292)*/,
    7,   12,  12,  12 /*mean (0.0307242), correlation (0.366299)*/,
    -7,  -13, -6,  5 /*mean (0.0534975), correlation (0.368357)*/,
    -4,  9,   -3,  4 /*mean (0.099865), correlation (0.372276)*/,
    7,   -1,  12,  2 /*mean (0.117083), correlation (0.364529)*/,
    -7,  6,   -5,  1 /*mean (0.126125), correlation (0.369606)*/,
    -13, 11,  -12, 5 /*mean (0.130364), correlation (0.358502)*/,
    -3,  7,   -2,  -6 /*mean (0.131691), correlation (0.375531)*/,
    7,   -8,  12,  -7 /*mean (0.160166), correlation (0.379508)*/,
    -13, -7,  -11, -12 /*mean (0.167848), correlation (0.353343)*/,
    1,   -3,  12,  12 /*mean (0.183378), correlation (0.371916)*/,
    2,   -6,  3,   0 /*mean (0.228711), correlation (0.371761)*/,
    -4,  3,   -2,  -13 /*mean (0.247211), correlation (0.364063)*/,
    -1,  -13, 1,   9 /*mean (0.249325), correlation (0.378139)*/,
    7,   1,   8,   -6 /*mean (0.000652272), correlation (0.411682)*/,
    1,   -1,  3,   12 /*mean (0.00248538), correlation (0.392988)*/,
    9,   1,   12,  6 /*mean (0.0206815), correlation (0.386106)*/,
    -1,  -9,  -1,  3 /*mean (0.0364485), correlation (0.410752)*/,
    -13, -13, -10, 5 /*mean (0.0376068), correlation (0.398374)*/,
    7,   7,   10,  12 /*mean (0.0424202), correlation (0.405663)*/,
    12,  -5,  12,  9 /*mean (0.0942645), correlation (0.410422)*/,
    6,   3,   7,   11 /*mean (0.1074), correlation (0.413224)*/,
    5,   -13, 6,   10 /*mean (0.109256), correlation (0.408646)*/,
    2,   -12, 2,   3 /*mean (0.131691), correlation (0.416076)*/,
    3,   8,   4,   -6 /*mean (0.165081), correlation (0.417569)*/,
    2,   6,   12,  -13 /*mean (0.171874), correlation (0.408471)*/,
    9,   -12, 10,  3 /*mean (0.175146), correlation (0.41296)*/,
    -8,  4,   -7,  9 /*mean (0.183682), correlation (0.402956)*/,
    -11, 12,  -4,  -6 /*mean (0.184672), correlation (0.416125)*/,
    1,   12,  2,   -8 /*mean (0.191487), correlation (0.386696)*/,
    6,   -9,  7,   -4 /*mean (0.192668), correlation (0.394771)*/,
    2,   3,   3,   -2 /*mean (0.200157), correlation (0.408303)*/,
    6,   3,   11,  0 /*mean (0.204588), correlation (0.411762)*/,
    3,   -3,  8,   -8 /*mean (0.205904), correlation (0.416294)*/,
    7,   8,   9,   3 /*mean (0.213237), correlation (0.409306)*/,
    -11, -5,  -6,  -4 /*mean (0.243444), correlation (0.395069)*/,
    -10, 11,  -5,  10 /*mean (0.247672), correlation (0.413392)*/,
    -5,  -8,  -3,  12 /*mean (0.24774), correlation (0.411416)*/,
    -10, 5,   -9,  0 /*mean (0.00213675), correlation (0.454003)*/,
    8,   -1,  12,  -6 /*mean (0.0293635), correlation (0.455368)*/,
    4,   -6,  6,   -11 /*mean (0.0404971), correlation (0.457393)*/,
    -10, 12,  -8,  7 /*mean (0.0481107), correlation (0.448364)*/,
    4,   -2,  6,   7 /*mean (0.050641), correlation (0.455019)*/,
    -2,  0,   -2,  12 /*mean (0.0525978), correlation (0.44338)*/,
    -5,  -8,  -5,  2 /*mean (0.0629667), correlation (0.457096)*/,
    7,   -6,  10,  12 /*mean (0.0653846), correlation (0.445623)*/,
    -9,  -13, -8,  -8 /*mean (0.0858749), correlation (0.449789)*/,
    -5,  -13, -5,  -2 /*mean (0.122402), correlation (0.450201)*/,
    8,   -8,  9,   -13 /*mean (0.125416), correlation (0.453224)*/,
    -9,  -11, -9,  0 /*mean (0.130128), correlation (0.458724)*/,
    1,   -8,  1,   -2 /*mean (0.132467), correlation (0.440133)*/,
    7,   -4,  9,   1 /*mean (0.132692), correlation (0.454)*/,
    -2,  1,   -1,  -4 /*mean (0.135695), correlation (0.455739)*/,
    11,  -6,  12,  -11 /*mean (0.142904), correlation (0.446114)*/,
    -12, -9,  -6,  4 /*mean (0.146165), correlation (0.451473)*/,
    3,   7,   7,   12 /*mean (0.147627), correlation (0.456643)*/,
    5,   5,   10,  8 /*mean (0.152901), correlation (0.455036)*/,
    0,   -4,  2,   8 /*mean (0.167083), correlation (0.459315)*/,
    -9,  12,  -5,  -13 /*mean (0.173234), correlation (0.454706)*/,
    0,   7,   2,   12 /*mean (0.18312), correlation (0.433855)*/,
    -1,  2,   1,   7 /*mean (0.185504), correlation (0.443838)*/,
    5,   11,  7,   -9 /*mean (0.185706), correlation (0.451123)*/,
    3,   5,   6,   -8 /*mean (0.188968), correlation (0.455808)*/,
    -13, -4,  -8,  9 /*mean (0.191667), correlation (0.459128)*/,
    -5,  9,   -3,  -3 /*mean (0.193196), correlation (0.458364)*/,
    -4,  -7,  -3,  -12 /*mean (0.196536), correlation (0.455782)*/,
    6,   5,   8,   0 /*mean (0.1972), correlation (0.450481)*/,
    -7,  6,   -6,  12 /*mean (0.199438), correlation (0.458156)*/,
    -13, 6,   -5,  -2 /*mean (0.211224), correlation (0.449548)*/,
    1,   -10, 3,   10 /*mean (0.211718), correlation (0.440606)*/,
    4,   1,   8,   -4 /*mean (0.213034), correlation (0.443177)*/,
    -2,  -2,  2,   -13 /*mean (0.234334), correlation (0.455304)*/,
    2,   -12, 12,  12 /*mean (0.235684), correlation (0.443436)*/,
    -2,  -13, 0,   -6 /*mean (0.237674), correlation (0.452525)*/,
    4,   1,   9,   3 /*mean (0.23962), correlation (0.444824)*/,
    -6,  -10, -3,  -5 /*mean (0.248459), correlation (0.439621)*/,
    -3,  -13, -1,  1 /*mean (0.249505), correlation (0.456666)*/,
    7,   5,   12,  -11 /*mean (0.00119208), correlation (0.495466)*/,
    4,   -2,  5,   -7 /*mean (0.00372245), correlation (0.484214)*/,
    -13, 9,   -9,  -5 /*mean (0.00741116), correlation (0.499854)*/,
    7,   1,   8,   6 /*mean (0.0208952), correlation (0.499773)*/,
    7,   -8,  7,   6 /*mean (0.0220085), correlation (0.501609)*/,
    -7,  -4,  -7,  1 /*mean (0.0233806), correlation (0.496568)*/,
    -8,  11,  -7,  -8 /*mean (0.0236505), correlation (0.489719)*/,
    -13, 6,   -12, -8 /*mean (0.0268781), correlation (0.503487)*/,
    2,   4,   3,   9 /*mean (0.0323324), correlation (0.501938)*/,
    10,  -5,  12,  3 /*mean (0.0399235), correlation (0.494029)*/,
    -6,  -5,  -6,  7 /*mean (0.0420153), correlation (0.486579)*/,
    8,   -3,  9,   -8 /*mean (0.0548021), correlation (0.484237)*/,
    2,   -12, 2,   8 /*mean (0.0616622), correlation (0.496642)*/,
    -11, -2,  -10, 3 /*mean (0.0627755), correlation (0.498563)*/,
    -12, -13, -7,  -9 /*mean (0.0829622), correlation (0.495491)*/,
    -11, 0,   -10, -5 /*mean (0.0843342), correlation (0.487146)*/,
    5,   -3,  11,  8 /*mean (0.0929937), correlation (0.502315)*/,
    -2,  -13, -1,  12 /*mean (0.113327), correlation (0.48941)*/,
    -1,  -8,  0,   9 /*mean (0.132119), correlation (0.467268)*/,
    -13, -11, -12, -5 /*mean (0.136269), correlation (0.498771)*/,
    -10, -2,  -10, 11 /*mean (0.142173), correlation (0.498714)*/,
    -3,  9,   -2,  -13 /*mean (0.144141), correlation (0.491973)*/,
    2,   -3,  3,   2 /*mean (0.14892), correlation (0.500782)*/,
    -9,  -13, -4,  0 /*mean (0.150371), correlation (0.498211)*/,
    -4,  6,   -3,  -10 /*mean (0.152159), correlation (0.495547)*/,
    -4,  12,  -2,  -7 /*mean (0.156152), correlation (0.496925)*/,
    -6,  -11, -4,  9 /*mean (0.15749), correlation (0.499222)*/,
    6,   -3,  6,   11 /*mean (0.159211), correlation (0.503821)*/,
    -13, 11,  -5,  5 /*mean (0.162427), correlation (0.501907)*/,
    11,  11,  12,  6 /*mean (0.16652), correlation (0.497632)*/,
    7,   -5,  12,  -2 /*mean (0.169141), correlation (0.484474)*/,
    -1,  12,  0,   7 /*mean (0.169456), correlation (0.495339)*/,
    -4,  -8,  -3,  -2 /*mean (0.171457), correlation (0.487251)*/,
    -7,  1,   -6,  7 /*mean (0.175), correlation (0.500024)*/,
    -13, -12, -8,  -13 /*mean (0.175866), correlation (0.497523)*/,
    -7,  -2,  -6,  -8 /*mean (0.178273), correlation (0.501854)*/,
    -8,  5,   -6,  -9 /*mean (0.181107), correlation (0.494888)*/,
    -5,  -1,  -4,  5 /*mean (0.190227), correlation (0.482557)*/,
    -13, 7,   -8,  10 /*mean (0.196739), correlation (0.496503)*/,
    1,   5,   5,   -13 /*mean (0.19973), correlation (0.499759)*/,
    1,   0,   10,  -13 /*mean (0.204465), correlation (0.49873)*/,
    9,   12,  10,  -1 /*mean (0.209334), correlation (0.49063)*/,
    5,   -8,  10,  -9 /*mean (0.211134), correlation (0.503011)*/,
    -1,  11,  1,   -13 /*mean (0.212), correlation (0.499414)*/,
    -9,  -3,  -6,  2 /*mean (0.212168), correlation (0.480739)*/,
    -1,  -10, 1,   12 /*mean (0.212731), correlation (0.502523)*/,
    -13, 1,   -8,  -10 /*mean (0.21327), correlation (0.489786)*/,
    8,   -11, 10,  -6 /*mean (0.214159), correlation (0.488246)*/,
    2,   -13, 3,   -6 /*mean (0.216993), correlation (0.50287)*/,
    7,   -13, 12,  -9 /*mean (0.223639), correlation (0.470502)*/,
    -10, -10, -5,  -7 /*mean (0.224089), correlation (0.500852)*/,
    -10, -8,  -8,  -13 /*mean (0.228666), correlation (0.502629)*/,
    4,   -6,  8,   5 /*mean (0.22906), correlation (0.498305)*/,
    3,   12,  8,   -13 /*mean (0.233378), correlation (0.503825)*/,
    -4,  2,   -3,  -3 /*mean (0.234323), correlation (0.476692)*/,
    5,   -13, 10,  -12 /*mean (0.236392), correlation (0.475462)*/,
    4,   -13, 5,   -1 /*mean (0.236842), correlation (0.504132)*/,
    -9,  9,   -4,  3 /*mean (0.236977), correlation (0.497739)*/,
    0,   3,   3,   -9 /*mean (0.24314), correlation (0.499398)*/,
    -12, 1,   -6,  1 /*mean (0.243297), correlation (0.489447)*/,
    3,   2,   4,   -8 /*mean (0.00155196), correlation (0.553496)*/,
    -10, -10, -10, 9 /*mean (0.00239541), correlation (0.54297)*/,
    8,   -13, 12,  12 /*mean (0.0034413), correlation (0.544361)*/,
    -8,  -12, -6,  -5 /*mean (0.003565), correlation (0.551225)*/,
    2,   2,   3,   7 /*mean (0.00835583), correlation (0.55285)*/,
    10,  6,   11,  -8 /*mean (0.00885065), correlation (0.540913)*/,
    6,   8,   8,   -12 /*mean (0.0101552), correlation (0.551085)*/,
    -7,  10,  -6,  5 /*mean (0.0102227), correlation (0.533635)*/,
    -3,  -9,  -3,  9 /*mean (0.0110211), correlation (0.543121)*/,
    -1,  -13, -1,  5 /*mean (0.0113473), correlation (0.550173)*/,
    -3,  -7,  -3,  4 /*mean (0.0140913), correlation (0.554774)*/,
    -8,  -2,  -8,  3 /*mean (0.017049), correlation (0.55461)*/,
    4,   2,   12,  12 /*mean (0.01778), correlation (0.546921)*/,
    2,   -5,  3,   11 /*mean (0.0224022), correlation (0.549667)*/,
    6,   -9,  11,  -13 /*mean (0.029161), correlation (0.546295)*/,
    3,   -1,  7,   12 /*mean (0.0303081), correlation (0.548599)*/,
    11,  -1,  12,  4 /*mean (0.0355151), correlation (0.523943)*/,
    -3,  0,   -3,  6 /*mean (0.0417904), correlation (0.543395)*/,
    4,   -11, 4,   12 /*mean (0.0487292), correlation (0.542818)*/,
    2,   -4,  2,   1 /*mean (0.0575124), correlation (0.554888)*/,
    -10, -6,  -8,  1 /*mean (0.0594242), correlation (0.544026)*/,
    -13, 7,   -11, 1 /*mean (0.0597391), correlation (0.550524)*/,
    -13, 12,  -11, -13 /*mean (0.0608974), correlation (0.55383)*/,
    6,   0,   11,  -13 /*mean (0.065126), correlation (0.552006)*/,
    0,   -1,  1,   4 /*mean (0.074224), correlation (0.546372)*/,
    -13, 3,   -9,  -2 /*mean (0.0808592), correlation (0.554875)*/,
    -9,  8,   -6,  -3 /*mean (0.0883378), correlation (0.551178)*/,
    -13, -6,  -8,  -2 /*mean (0.0901035), correlation (0.548446)*/,
    5,   -9,  8,   10 /*mean (0.0949843), correlation (0.554694)*/,
    2,   7,   3,   -9 /*mean (0.0994152), correlation (0.550979)*/,
    -1,  -6,  -1,  -1 /*mean (0.10045), correlation (0.552714)*/,
    9,   5,   11,  -2 /*mean (0.100686), correlation (0.552594)*/,
    11,  -3,  12,  -8 /*mean (0.101091), correlation (0.532394)*/,
    3,   0,   3,   5 /*mean (0.101147), correlation (0.525576)*/,
    -1,  4,   0,   10 /*mean (0.105263), correlation (0.531498)*/,
    3,   -6,  4,   5 /*mean (0.110785), correlation (0.540491)*/,
    -13, 0,   -10, 5 /*mean (0.112798), correlation (0.536582)*/,
    5,   8,   12,  11 /*mean (0.114181), correlation (0.555793)*/,
    8,   9,   9,   -6 /*mean (0.117431), correlation (0.553763)*/,
    7,   -4,  8,   -12 /*mean (0.118522), correlation (0.553452)*/,
    -10, 4,   -10, 9 /*mean (0.12094), correlation (0.554785)*/,
    7,   3,   12,  4 /*mean (0.122582), correlation (0.555825)*/,
    9,   -7,  10,  -2 /*mean (0.124978), correlation (0.549846)*/,
    7,   0,   12,  -2 /*mean (0.127002), correlation (0.537452)*/,
    -1,  -6,  0,   -11 /*mean (0.127148), correlation (0.547401)*/
};

void ORBextractor::operator()(cv::InputArray _image, Feature features) {
  if (_image.empty())
    return;

  auto &allkeypoints = features._keypoints;
  auto &alldescriptors = features._descriptors;

  cv::Mat image = _image.getMat();
  assert(image.type() == CV_8UC1);

  ComputePyramid(image);

  // 需要对边缘进行筛选，否则计算描述子时会发生段错误
  ComputeKeyPoints(allkeypoints);

  // 统计所有层图像金字塔的特征点总数
  int num_keypoints = 0;
  for (int level = 0; level < _levels; ++level) {
    num_keypoints += (int)allkeypoints[level].size();
  }

  alldescriptors.resize(allkeypoints.size());

  for (int level = 0; level < _levels; ++level) {
    // 获取在allKeypoints中当前层特征点容器的句柄
    std::vector<cv::KeyPoint> &keypoints = allkeypoints[level];
    std::vector<DescType> &descriptors = alldescriptors[level];
    descriptors.resize(keypoints.size());

    // 如果特征点数目为0，跳出本次循环，继续下一层金字塔
    if ((int)keypoints.size() == 0)
      continue;
    // 对图像进行高斯模糊
    cv::Mat workingMat = _perImagePyramid[level].clone();
    cv::GaussianBlur(workingMat, workingMat, cv::Size(7, 7), 2, 2,
                     cv::BORDER_REFLECT_101);

    ComputeDescriptors(workingMat, keypoints, descriptors);

    // 对于0层以上的图像，进行坐标还原
    if (level != 0) {
      double scale = _perScaleFactor[level];
      for (auto &kp : keypoints)
        kp.pt *= scale;
    }
  }
}

int ORBextractor::ComputeDescriptorDistance(const DescType &a,
                                            const DescType &b) {
  int distance = 0;
  for (int i = 0; i < 8; i++) {
    uint32_t v = a[i] ^ b[i];
    v = v - ((v >> 1) & 0x55555555);
    v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
    distance += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
  }
  return distance;
}

void ORBextractor::ComputeDescriptors(
    const cv::Mat &image, const std::vector<cv::KeyPoint> &keypoints,
    std::vector<DescType> &descriptors) {
  for (int i = 0; i < keypoints.size(); ++i) {
    ComputeDescriptor(image, keypoints[i], descriptors[i]);
  }
}

// 一度对应着多少弧度
const static double factorPI = (double)(CV_PI / 180.f);

void ORBextractor::ComputeDescriptor(const cv::Mat &image,
                                     const cv::KeyPoint &keypoint,
                                     DescType &descriptor) {
  descriptor.clear();
  // keypoint.angle是角度制，范围是[0,360)度，转成弧度制
  double angle = (double)keypoint.angle * factorPI;
  // 计算这个角度的余弦值和正弦值
  double c = (double)cos(angle), s = (double)sin(angle);

  // cvRound() 四舍五入函数
  const uchar *center =
      &image.at<uchar>(cvRound(keypoint.pt.y), cvRound(keypoint.pt.x));
  const int step = (int)image.step;

// x'= xcos(θ) - ysin(θ),  y'= xsin(θ) + ycos(θ)
// y' * step + x'
// idx + 0  : x 坐标
// idx + 1  : y 坐标
#define GET_VALUE(t, idx, offset)                                              \
  {                                                                            \
    t = center[cvRound(_ORB_pattern[idx * 32 + offset + 0] * s +               \
                       _ORB_pattern[idx * 32 + offset + 1] * c) *              \
                   step +                                                      \
               cvRound(_ORB_pattern[idx * 32 + offset + 0] * c -               \
                       _ORB_pattern[idx * 32 + offset + 1] * s)];              \
    offset += 2;                                                               \
  }
  DescType temp_descriptor(8, 0);
  for (int i = 0; i < 8; ++i) {
    int offset = 0;
    // t0 参与比较的第1个特征点的灰度值
    // t1 参与比较的第2个特征点的灰度值
    // val 描述子这个字节的比较结果，0或1
    int t0, t1, val;
    for (int j = 0; j < 32; ++j) {
      if (j == 0) {
        GET_VALUE(t0, i, offset);
        GET_VALUE(t1, i, offset);
        val = t0 < t1;
      } else {
        GET_VALUE(t0, i, offset);
        GET_VALUE(t1, i, offset);
        val |= (t0 < t1) << j;
      }
    }
    temp_descriptor[i] = (uint32_t)val;
  }
  descriptor = temp_descriptor;
#undef GET_VALUE
}

} // namespace slamlite