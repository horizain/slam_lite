#include "ORBextractor.h"
#include "common_include.h"
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace slamlite;
using namespace cv;



int main(int argc, char *argv[])
{
    Mat img = imread(argv[1], CV_LOAD_IMAGE_GRAYSCALE);
    ORBextractor orb(100, 1.25, 1, 40, 20);
    orb._levels = 1;
    std::vector<std::vector<cv::KeyPoint>> allkeypoints;
    std::vector<std::vector<DescType>> alldescriptors;
    orb(img, allkeypoints, alldescriptors);
    // DescType i = alldescriptors[0][100];
    for (auto i : alldescriptors[0])
    {
        for (auto j : i)
        {
            std::cout << j;
        }
    }
    std::cout << std::endl;
    Mat outimg;
    drawKeypoints(img, allkeypoints.at(0), outimg, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
    imshow("quick fast features", outimg);
    waitKey(0);
    return 0;
}