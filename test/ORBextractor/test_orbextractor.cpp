#include "ORBextractor.h"
#include "common_include.h"
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace slamlite;
using namespace cv;

int main(int argc, char *argv[])
{
    Mat img = imread(argv[1], CV_LOAD_IMAGE_GRAYSCALE);
    ORBextractor orb;
    orb._levels = 1;
    orb._imagePyramid.resize(orb._levels);
    orb.ComputePyramid(img);
    std::vector<std::vector<cv::KeyPoint>> allkeypoints;
    orb.ComputeKeyPoints(allkeypoints);
    DescType i;
    orb.ComputeDescriptor(img, allkeypoints.at(0).at(100), i);
    for (auto j : i)
    {
        std::cout << j;
    }
    std::cout << std::endl;
    Mat outimg;
    drawKeypoints(img, allkeypoints.at(0), outimg, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
    imshow("quick fast features", outimg);
    waitKey(0);
    return 0;
}