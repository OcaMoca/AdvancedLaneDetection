#ifndef __FEATUREEXTRACTION_H__
#define __FEATUREEXTRACTION_H__

#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include <nlohmann/json.hpp>
#include <fstream>
#include "../Window/Window.hpp"

using namespace cv;
using namespace std;

class FeatureExtraction
{
    private:
        vector<float> succ_avg_speed;
        vector<Point2f> old_point_vector;

        float car_offset;
        int car_speed;
        
        float trim_mean(vector<float>& , float);
       
    public:
        FeatureExtraction();

        void get_histogram(const Mat&, Mat&);
        void calculate_lane_histogram(const Mat&, Point&, Point&);
        void sliding_window(Mat&, const Point&, const Point&, Mat&, vector<Window>&, vector<Window>&);
        void calculate_lane_fit_next_frame(const vector<Point2f>&, Mat&, vector<float>&, vector<float>&, int); 
        void non_sliding_window(const Mat&, Mat&, Mat&, Mat&,  Mat&, int);

        Mat polyfit_windows(const vector<Window>&);
        void polyfit(const Mat&, const Mat&, Mat&, int);
        vector<float> linspace(float, float, int);
        void poly_fit_x(const vector<float>&, vector<float>&, const Mat&);

        float calculate_curvature(const Mat&, int);
        void convert_to_optical(const Mat&, Mat&, Mat&);
        void calculate_car_offset(Mat&, Mat&, Mat&);
        void steering_wheel_rotation(Mat&, float, float&, Mat&);
        
        float get_car_offset() {return car_offset;}
        float get_car_speed() {return car_speed;}
        

};

#endif