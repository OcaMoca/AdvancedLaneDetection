#ifndef __LANEDETECTION_H__
#define __LANEDETECTION_H__

#include <thread>
#include <experimental/filesystem>
#include <iostream>
#include <algorithm> 
#include <opencv2/videoio.hpp>
#include <functional> 
#include <cmath>


#include "../include/Window.hpp"
#include "../include/CameraCalibration.hpp"


class LaneDetection
{
    private:

        VideoCapture capture;
        VideoWriter video_output;

        CameraCalibration calibrator;

        Mat frame;
        Mat prev_frame;
        bool first_frame;

        Mat left_fit_lane, right_fit_lane;

        float trap_bottom_width; 
        float trap_top_width; 
        float trap_height; 
        float car_hood;

        vector<Point2f> original_roi;
        vector<Point2f> warped_roi;
        
        vector<float> succ_avg_speed;
        vector<Point2f> old_point_vector;
        int result_optical;

        void polyfit(const Mat&, const Mat&, Mat&, int);
        vector<float> linspace(float, float, int);
        Mat polyfit_windows(const vector<Window>&);
        void poly_fit_x(const vector<float>&, vector<float>&, const Mat&);

        void trapezoid_roi();
        void color_filter(Mat&);
        void calculate_sobel(Mat&);
        void perspective_transform(const Mat&, Mat&);
        void get_histogram(const Mat&, Mat&);
        void calculate_lane_histogram(const Mat&, Point&, Point&);
        void sliding_window(Mat&, const Point&, const Point&, Mat&, vector<Window>&, vector<Window>&);
        void calculate_lane_fit_next_frame(vector<Point2f>, Mat&, vector<float>&, vector<float>&, int); 
        void non_sliding_window(const Mat&, Mat&, Mat&, Mat&,  Mat&, int);
        void get_inverse_points(const vector<float>&, const vector<float>&, const vector<float>&, Mat&);
        void original_perspective(const Mat&, Mat&, Mat&);
        void final_perspective(const Mat&, const Mat&, Mat&, Mat&);
        float calculate_curvature(const Mat&, int);
        float trim_mean(vector<float>& , float);
        void convert_to_optical(const Mat&, Mat&);

    public:

        LaneDetection();
    
        void init(string file_name, string output_file);
        bool frame_processing();
        void release();

};

#endif