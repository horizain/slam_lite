#ifndef __FAST_DETECTOR_H
#define __FAST_DETECTOR_H
#include "keyPoint.h"

class FASTDetector
{
private:
    int _mnFeatures;
    float _mfScaleFactor;
    int _mnLevels;
    int _mnInitThFAST;
    int _mnMinThFAST;

public:
    FASTDetector();
    FASTDetector(int nfeatures, float fscaleFactor, int nlevels, int ninitThFAST, int nminThFAST);
    ~FASTDetector();

    void computePyramid();

    bool preFASTCheck(const cv::Mat &img, int u, int v, double percent);
    
};

#endif