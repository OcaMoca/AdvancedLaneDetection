#ifndef __FRAMEPROCESSING_H__
#define __FRAMEPROCESSING_H__

#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include <nlohmann/json.hpp>
#include <fstream>

using json = nlohmann::json;
using namespace std;
using namespace cv;

class FrameProcessing
{

    private:
        json j;
        int white_range;
        
        float trap_bottom_width; 
        float trap_top_width; 
        float trap_height; 
        float car_hood;

        vector<Point2f> original_roi;
        vector<Point2f> warped_roi;

    public:

        FrameProcessing();

        void trapezoid_roi(const Mat&, string, string, string, string);
        void color_filter(Mat&, Mat&, string&);
        void calculate_sobel(Mat&, Mat&);
        void perspective_transform(const Mat&, Mat&);

        vector<Point2f> get_original_roi() { return original_roi;}
        vector<Point2f> get_warped_roi() { return warped_roi;}

};

#endif