#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <chrono>

class ORB_Worker
{
private:
    /* data */
public:
    ORB_Worker(/* args */);
    ~ORB_Worker();

    void getFast(cv::Mat img, std::vector<cv::KeyPoint> kp);
};

ORB_Worker::ORB_Worker(/* args */)
{

}

ORB_Worker::~ORB_Worker()
{

}
