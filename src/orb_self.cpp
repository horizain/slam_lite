#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "FASTDetector.h"

int main(int argc, char *argv[])
{
    using namespace cv;
    if (argc != 3)
    {
        std::cout << "Usage: exe img threshold" << std::endl;
    }
    Mat img = imread(argv[1], CV_LOAD_IMAGE_GRAYSCALE);

    FASTDetector FASTer;
    std::vector<KeyPoint> kps;
    assert(img.data != nullptr);

    for (int v = 4; v < img.rows - 4; ++v)
    {
        for (int u = 4; u < img.cols - 4; ++u)
        {
            if (!FASTer.preFASTCheck(img, u, v, atof(argv[2])))
            {
                continue;
            }
            else
            {
                if (FASTer.FASTCheck(img, u, v, atof(argv[2])))
                {
                    KeyPoint kp;
                    kp.pt.x = u;
                    kp.pt.y = v;
                    kps.push_back(kp);
                }

            }
        }
    }
    // 画出可视化图像
    Mat outimg;
    drawKeypoints(img, kps, outimg, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
    imshow("quick fast features", outimg);
    // std::vector<KeyPoint> kp_1;
    // Ptr<FeatureDetector> detector = ORB::create();
    // detector->detect(img, kp_1);
    // // 画出可视化图像
    // Mat outimg2;
    // drawKeypoints(img, kp_1, outimg2, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
    // imshow("openCV fast features", outimg2);
    waitKey(0);
}