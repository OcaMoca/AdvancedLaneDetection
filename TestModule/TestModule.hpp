#ifndef __TESTMODULE_H__
#define __TESTMODULE_H__

#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <fstream>
#include <iterator>
#include <vector>
#include <numeric>

using namespace std;
using namespace cv;

class TestModule
{
    private:
        vector<int> error;
        ofstream ground_turth;
        char test_text_x[50], test_text_y[50];

    public:
        TestModule();

        void compare_parameters(vector<Point2f>&, vector<Point2i>&, const string);
        void calculate_params_error(vector<Point2i>&, vector<Point2f>&, vector<Point2f>&);
        void get_frame_params(int&, vector<Point2f>&, vector<Point2f>&);
        void calculate_specific_point_error(vector<Point2i>&, vector<Point2f>&, vector<Point2f>&);
        
       
};

#endif