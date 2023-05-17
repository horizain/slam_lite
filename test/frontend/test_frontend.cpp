#include "include/common_include.h"
#include "include/frontend.h"
#include "include/config.h"
#include "include/frame.h"
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
using namespace slamlite;
using namespace cv;

int main(int argc, char *argv[])
{
    Mat img = imread(argv[1], CV_LOAD_IMAGE_GRAYSCALE);
    Config::SetParameterFile(argv[2]);
    Frontend frontend;
    auto frame = Frame::CreateFrame();
    frame->_img = img;
    frontend._initFASTThreshold = 50;
    std::cout << frontend._initFASTThreshold << std::endl;
    frontend.AddFrame(frame);
    // 画出可视化图像
    Mat outimg;
    std::vector<KeyPoint> kps;
    for (auto i : frontend._current_frame->_feature)
        kps.push_back(i->_point);
    drawKeypoints(img, kps, outimg, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
    imshow("quick fast features", outimg);
    waitKey(0);
    return 0;
}
