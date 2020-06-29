#ifndef __HMI_H__
#define __HMI_H__

#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"

using namespace std;
using namespace cv;

class HMI

{
    private:
        void transparency(Mat&, Mat&, int);
    
    public:
        HMI();
        
        void get_inverse_points(const vector<float>&, const vector<float>&, const vector<float>&, Mat&);
        void final_perspective(const Mat&, const Mat&, vector<Point2f>&, vector<Point2f>&, Mat&);

        void show_values(float, float, int, Mat&);
        void show_steering_wheel(Mat&, Mat&, int);      
};
#endif