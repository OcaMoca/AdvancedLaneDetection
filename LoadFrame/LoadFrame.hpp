#ifndef __LOADFRAME_H__
#define __LOADFRAME_H__

#include <iostream>
#include <fstream>
#include <algorithm>
#include <string>
#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

using namespace std;
using namespace cv;

class LoadFrame
{
    private:
        string input_file;
        string output_file;

        VideoCapture capture;
        VideoWriter video_output;

        Mat frame;

        json j;

    public:

        LoadFrame();
        void open_input_video();
        bool read_frame();
        void get_frame(Mat&);
        void write_to_output_video(Mat&);
        void close_output_video();

};

#endif