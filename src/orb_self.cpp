#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <chrono>
#include "FASTDetector.h"

int main(int argc, char *argv[])
{
    using namespace cv;
    using namespace std;
    if (argc != 3)
    {
        std::cout << "Usage: exe img threshold" << std::endl;
    }
    Mat img = imread(argv[1], CV_LOAD_IMAGE_GRAYSCALE);

    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();

    FASTDetector FASTer;
    std::vector<KeyPoint> kps;
    assert(img.data != nullptr);

    FASTer.detect(img, atof(argv[2]), kps);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "match ORB cost = " << time_used.count() << " seconds. " << endl;
    t1 = chrono::steady_clock::now();
    std::vector<KeyPoint> kps2 = FASTer.nonMaxSuppression(img, kps);
    t2 = chrono::steady_clock::now();
    time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "match nms cost = " << time_used.count() << " seconds. " << endl;
    // 画出可视化图像
    Mat outimg;
    drawKeypoints(img, kps, outimg, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
    imshow("quick fast features", outimg);

    // // 画出可视化图像
    Mat outimg2;
    drawKeypoints(img, kps2, outimg2, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
    imshow("fast nms features", outimg2);

    t1 = chrono::steady_clock::now();
    std::vector<KeyPoint> kp_1;
    Ptr<FeatureDetector> detector = ORB::create();
    detector->detect(img, kp_1);
    t2 = chrono::steady_clock::now();
    time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "match opencv cost = " << time_used.count() << " seconds. " << endl;
    // // 画出可视化图像
    Mat outimg3;
    drawKeypoints(img, kp_1, outimg3, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
    imshow("openCV fast features", outimg3);
    waitKey(0);
}