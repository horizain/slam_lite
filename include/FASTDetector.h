#ifndef __FAST_DETECTOR_H
#define __FAST_DETECTOR_H
#include "keyPoint.h"
#include <opencv2/core/mat.hpp>



class FASTDetector {
private:
  int _mnFeatures;
  float _mfScaleFactor;
  int _mnLevels;
  int _mnInitThFAST;
  int _mnMinThFAST;
  int _mnFAST9Mask[16 * 2] = {-3, 0,  -3, 1,  -2, 2,  -1, 3,  0,  3, 1,
                         3,  2,  2,  3,  1,  3,  0,  3,  -1, 2, -2,
                         1,  -3, 0,  -3, -1, -3, -2, -2, -3, -1};
  enum _meLastFastState
  {
    SMALLER = -1,
    INIT = 0,
    BIGGER = 1
  };

public:
  FASTDetector();
  FASTDetector(int nfeatures, float fscaleFactor, int nlevels, int ninitThFAST,
               int nminThFAST);
  ~FASTDetector();

  void computePyramid();

  bool detect(const cv::Mat &img, double threshold, std::vector<cv::KeyPoint> &vpkp);
  bool preCheck(const cv::Mat &img, int u, int v, double threshold);
  bool check9(const cv::Mat &img, int u, int v, double threshold);
  void selectMax(const cv::Mat &img, int windows_size, std::vector<cv::KeyPoint> &vpkp);
  std::vector<cv::KeyPoint> nonMaxSuppression(const cv::Mat &img, const std::vector<cv::KeyPoint>& pixels);
};

#endif
