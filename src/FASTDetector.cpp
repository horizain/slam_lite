#include "FASTDetector.h"
#include <cstdint>

FASTDetector::FASTDetector() {}

FASTDetector::FASTDetector(int nfeatures, float fscaleFactor, int nlevels,
                           int ninitThFAST, int nminThFAST) {}

FASTDetector::~FASTDetector() {}

bool FASTDetector::preCheck(const cv::Mat &img, int u, int v, double threshold)
{
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
  for (auto p : pixels)
  {
    if (p > (1.0 + threshold) * img.at<uchar>(v, u))
    {
      ++bigger;
    }
    else if (p < (1.0 - threshold) * img.at<uchar>(v, u))
    {
      ++smaller;
    }
  }

  if (bigger >= 3 || smaller >= 3)
    return true;
  else
    return false;
}

bool FASTDetector::check9(const cv::Mat &img, int u, int v, double threshold)
{
  uint8_t bigger = 0;
  uint8_t smaller = 0;
  int last = INIT;

  for (int i = 0; i < 16; ++i)
  {
    if (bigger >= 12 || smaller >= 12)
    {
      return true;
    }
    uchar p = img.at<uchar>(v + _mnFAST9Mask[2 * i], u + _mnFAST9Mask[2 * i + 1]);
    if (p > (1.0 + threshold) * img.at<uchar>(v, u))
    {
      ++bigger;
    }
    else if (p < (1.0 - threshold) * img.at<uchar>(v, u))
    {
      ++smaller;
    }
    else
    {
      bigger = 0;
      smaller = 0;
    }
  }
  return false;
}
bool FASTDetector::detect(const cv::Mat &img, double threshold, std::vector<cv::KeyPoint> &vpkp)
{
  vpkp.clear();
  for (int v = 4; v < img.rows - 4; ++v)
  {
    for (int u = 4; u < img.cols - 4; ++u)
    {
      if (preCheck(img, u, v, threshold))
      {
        if (check9(img, u, v, threshold))
        {
          cv::KeyPoint kp;
          kp.pt.x = u;
          kp.pt.y = v;
          vpkp.push_back(kp);
        }
      }
    }
  }
  int windows_size = 7;
  selectMax(img, windows_size, vpkp);
  if (vpkp.size() != 0)
    return true;
  else
    return false;
}

// 局部极大值抑制，这里利用fast特征点的响应值做比较
void FASTDetector::selectMax(const cv::Mat &img, int windows_size, std::vector<cv::KeyPoint> &vpkp)
{
  int u, v;
  for (auto kp : vpkp)
  {
    size_t nms = 0;
    u = kp.pt.x;
    v = kp.pt.y;
    for (int i = 0; i < 16; ++i)
    {
      uchar p = img.at<uchar>(v + _mnFAST9Mask[2 * i], u + _mnFAST9Mask[2 * i + 1]);
      nms += abs(img.at<uchar>(v, u) - p);
    }
    
  }
}
