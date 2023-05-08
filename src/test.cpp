#include <iostream>
#include <vector>
#include <cmath>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

// 定义FAST角点检测器的阈值
const int THRESHOLD = 20;

// 定义一个结构体，用于存储像素点的坐标和灰度值
struct Pixel {
    int x;
    int y;
    int value;
};

// 定义一个函数，用于计算两个像素点之间的距离
double distance(int x1, int y1, int x2, int y2) {
    return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}

// 定义一个函数，用于检测像素点是否为FAST角点
bool isCorner(int x, int y, const Mat& image) {
    int center = image.at<uchar>(y, x);
    int threshold = THRESHOLD;
    int count = 0;
    for (int i = 0; i < 16; i++) {
        int offset = i * 2;
        int x1 = x + cos(i * M_PI / 8) * 3;
        int y1 = y - sin(i * M_PI / 8) * 3;
        int x2 = x + cos((i + 1) * M_PI / 8) * 3;
        int y2 = y - sin((i + 1) * M_PI / 8) * 3;
        int value1 = image.at<uchar>(y1, x1);
        int value2 = image.at<uchar>(y2, x2);
        if (value1 - center > threshold && value2 - center > threshold) {
            count++;
        }
    }
    return count >= 12;
}

// 定义一个函数，用于进行非极大值抑制
vector<KeyPoint> nonMaxSuppression(const Mat &img, const vector<KeyPoint>& pixels) {
    vector<KeyPoint> result;
    for (int i = 0; i < pixels.size(); i++) {
        bool isMax = true;
        for (int j = 0; j < pixels.size(); j++) {
            if (i != j && distance(pixels[i].pt.x, pixels[i].pt.y, pixels[j].pt.x, pixels[j].pt.y) < 10) {
                if (img.at<uchar>(pixels[i].pt.y, pixels[i].pt.x) < img.at<uchar>(pixels[j].pt.y, pixels[j].pt.x)) {
                    isMax = false;
                    break;
                }
            }
        }
        if (isMax) {
            result.push_back(pixels[i]);
        }
    }
    return result;
}

// 定义一个函数，用于进行快速非极大值抑制
vector<KeyPoint> fastNMS(const Mat &img, const vector<KeyPoint>& pixels, int maxCorners) {
    vector<KeyPoint> result;
    vector<int> scores;
    for (int i = 0; i < pixels.size(); i++) {
        scores.push_back(img.at<uchar>(pixels[i].pt.y, pixels[i].pt.x));
    }
    int k = min(maxCorners, static_cast<int>(pixels.size()));
    nth_element(scores.begin(), scores.begin() + k - 1, scores.end(), greater<int>());
    int threshold = scores[k - 1];
    for (int i = 0; i < pixels.size(); i++) {
        if (img.at<uchar>(pixels[i].pt.y, pixels[i].pt.x) >= threshold) {
            result.push_back(pixels[i]);
        }
        if (result.size() >= maxCorners) {
            break;
        }
    }
    return result;
}

int main() {
    // 读取图像数据
    Mat image = imread("opencv.jpg", IMREAD_GRAYSCALE);

    // 提取FAST角点
    vector<KeyPoint> corners;
    for (int y = 3; y < image.rows - 3; y++) {
        for (int x = 3; x < image.cols - 3; x++) {
            if (isCorner(x, y, image)) {
                KeyPoint pixel;
                pixel.pt.x = x;
                pixel.pt.y = y;
                corners.push_back(pixel);
            }
        }
    }

    // 进行非极大值抑制
    // vector<KeyPoint> result = nonMaxSuppression(image, corners);
    vector<KeyPoint> result = fastNMS(image, corners, 200);

    // 输出结果
    Mat outimg;
    drawKeypoints(image, result, outimg, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
    imshow("fast features", outimg);
    waitKey(0);
    return 0;
}
