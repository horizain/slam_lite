#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "FASTDetector.h"

int main(int argc, char *argv[])
{
    using namespace cv;
    if (argc != 3)
    {
        std::cout << "Usage: exe img threshold" << std::endl;
    }
    Mat img = imread(argv[1], CV_LOAD_IMAGE_COLOR);

    FASTDetector FASTer;
    std::vector<KeyPoint> kps;
    assert(img.data != nullptr);

    for (int v = 20; v < img.rows - 20; ++v)
    {
        for (int u = 20; u < img.cols - 20; ++u)
        {
            if (!FASTer.preFASTCheck(img, u, v, atof(argv[2])))
            {
                continue;
            }
            else
            {
                KeyPoint kp;
                kp.pt.x = u;
                kp.pt.y = v;
                kps.push_back(kp);
            }
        }
    }
    // 画出可视化图像
    Mat outimg;
    drawKeypoints(img, kps, outimg, Scalar::all(-1), DrawMatchesFlags::DEFAULT);

    imshow("quick fast features", outimg);
    waitKey(0);
}