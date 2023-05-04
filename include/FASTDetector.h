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
    FASTDetector(int nfeatures, float fscaleFactor, int nlevels, int ninitThFAST, int nminThFAST);
    ~FASTDetector();

    void computePyramid();

    static bool preFASTCheck(const cv::Mat &img, int u, int v, double percent);
    
};