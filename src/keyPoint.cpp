#include "include/keyPoint.h"

keyPoint::keyPoint(int u, int v, int id, int num)
{
    _mnU = u;
    _mnV = v;
    _mnID = id;
    _mnNum = num;
}

void keyPoint::setU(int u)
{
    _mnU = u;
}

void keyPoint::setV(int v)
{
    _mnV = v;
}

void keyPoint::setID(int id)
{
    _mnID = id;
}

void keyPoint::setNum(int num)
{
    _mnNum = num;
}

void keyPoint::setAngle(double angle)
{
    _mdAngle = angle;
}

int keyPoint::getID()
{
    return _mnID;
}

int keyPoint::getNum()
{
    return _mnNum;
}

Eigen::Vector2d keyPoint::getPointCoordi()
{
    Eigen::Vector2d point;
    point << _mnU, _mnV;
    return point;
}