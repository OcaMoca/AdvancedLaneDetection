#ifndef __TESTMODULE_H__
#define __TESTMODULE_H__

#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>

using namespace std;
using namespace cv;

class TestModule
{
    
    public:
        TestModule();

        void compare_parameters(vector<Point2f>&, vector<Point2i>&);
       
};

#endif