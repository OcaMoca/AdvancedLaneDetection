#ifndef __LANEDETECTION_H__
#define __LANEDETECTION_H__

#include <thread>
#include <experimental/filesystem>
#include <iostream>
#include <algorithm> 
#include <opencv2/videoio.hpp>
#include <functional> 


#include "../include/Window.hpp"
#include "../include/CameraCalibration.hpp"


class LaneDetection
{
    private:

        VideoCapture capture;
        VideoWriter video_output;
        Mat frame;
        CameraCalibration calibrator;

        float trap_bottom_width; 
        float trap_top_width; 
        float trap_height; 
        float car_hood;

        vector<Point2f> original_roi;
        vector<Point2f> warped_roi;

        void polyfit(const Mat&, const Mat&, Mat&, int);
        vector<float> linspace(float, float, int);
        Mat polyfit_windows(vector<Window> const&);
        void poly_fit_x(vector<float> const&, vector<float>&, Mat const&);

        
        //void setup_calibration(int, int, float, Mat&);
        void trapezoid_roi();
        void color_filter(Mat&);
        void perspective_transform(const Mat&, Mat&);
        void get_histogram(Mat const&, Mat&);
        void calculate_lane_histogram(const Mat&, Point&, Point&);
        void sliding_window(Mat&, Point&, Point&, Mat&, vector<Window>&, vector<Window>&);
        void get_inverse_points(vector<float>&, vector<float>&, vector<float>&, Mat&);
        void inverse_perspective(const Mat&, Mat&, Mat&);
        void final_perspective(const Mat&, const Mat&, Mat&, Mat&);
        
    public:

        LaneDetection();
    
        void init(string file_name, string output_file);
        bool frame_processing();
        void release();

};

#endif