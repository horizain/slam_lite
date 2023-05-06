#include "FASTDetector.h"
#include <cstdint>

FASTDetector::FASTDetector() {}

FASTDetector::FASTDetector(int nfeatures, float fscaleFactor, int nlevels,
                           int ninitThFAST, int nminThFAST) {}

FASTDetector::~FASTDetector() {}

bool FASTDetector::preFASTCheck(const cv::Mat &img, int u, int v,
                                double threshold) {
  uint8_t bigger = 0;
  uint8_t smaller = 0;

  std::vector<uchar> pixels;
  uchar point_1 = img.at<uchar>(v - 3, u);
  uchar point_5 = img.at<uchar>(v, u + 3);
  uchar point_9 = img.at<uchar>(v + 3, u);
  uchar point_13 = img.at<uchar>(v, u - 3);

  pixels.push_back(point_1);
  pixels.push_back(point_5);
  pixels.push_back(point_9);
  pixels.push_back(point_13);

  uchar temp = img.at<uchar>(v, u);
  for (auto p : pixels) {
    if (p > (1.0 + threshold) * img.at<uchar>(v, u)) {
      ++bigger;
    } else if (p < (1.0 - threshold) * img.at<uchar>(v, u)) {
      ++smaller;
    }
  }

  if (bigger >= 3 || smaller >= 3)
    return true;
  else
    return false;
}

bool FASTDetector::FASTCheck(const cv::Mat &img, int u, int v,
                             double threshold) {
  uint8_t bigger = 0;
  uint8_t smaller = 0;
  int last = INIT;

  for (int i = 0; i < 16; ++i) {
    if (bigger >= 12 || smaller >= 12) {
      return true;
    }
    uchar p =
        img.at<uchar>(v + _mnFAST9Mask[2 * i], u + _mnFAST9Mask[2 * i + 1]);
    if (p > (1.0 + threshold) * img.at<uchar>(v, u)) {
      ++bigger;
    } else if (p < (1.0 - threshold) * img.at<uchar>(v, u)) {
      ++smaller;
    } else {
      bigger = 0;
      smaller = 0;
    }
  }
  return false;
}