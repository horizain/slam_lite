#include "FASTDetector.h"

FASTDetector::FASTDetector(int nfeatures, float fscaleFactor, int nlevels, int ninitThFAST, int nminThFAST)
{

}

FASTDetector::~FASTDetector()
{

}

inline bool FASTDetector::preFASTCheck(const cv::Mat &img, int u, int v, double threshold)
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

    for (auto p : pixels)
    {
        if (p > img.at<uchar>(v, u) + threshold * img.at<uchar>(v, u))
        {
            ++bigger;
        }
        else if (p < img.at<uchar>(v, u) - threshold * img.at<uchar>(v, u))
        {
            ++bigger;
        }
    }

    if (bigger >= 3)
        return true;
    else
        return false;
}

