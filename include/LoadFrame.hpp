#ifndef __LOADFRAME_H__
#define __LOADFRAME_H__

#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

class LoadFrame
{
    private:
        string file_name;
        string output_file;

        VideoCapture capture;
        VideoWriter video_output;

        Mat frame;

    public:

        LoadFrame();
        void open_input_video();
        bool read_frame();
        void get_frame(Mat&);
        void write_to_output_video(Mat&);
        void close_output_video();

};

#endif