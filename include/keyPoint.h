#ifndef __KEYPOINT_H
#define __KEYPOINT_H

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Core>

class keyPoint
{
private:
    int _mnU, _mnV;
    int _mnID;
    int _mnNum;
    double _mdAngle;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    keyPoint();
    keyPoint(int u, int v, int id, int num);
    ~keyPoint();

    void setU(int u);
    void setV(int v);
    void setID(int id);
    void setNum(int num);
    void setAngle(double angle);

    int getID();
    int getNum();
    Eigen::Vector2d getPointCoordi();
};

#endif