
#ifndef __WINDOW_H__
#define __WINDOW_H__

#include <vector>
#include <numeric>
#include <ostream>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/highgui/highgui.hpp"



using namespace cv;
using namespace std;

class Window
{
    private:

        int x_left, x_right;
        int y_top, y_bottom;
        int x_center;
        int width, height;
        int min_pix;
        Mat window;
        vector<Point2f> non_zero;
        vector<float> non_zero_x;
        
    public:

        Window() : x_left(0), x_right(0), y_top(0), y_bottom(0), x_center(0),
                   width(0), height(0), min_pix(0) {}

        Window(Mat& binary_warped, int x_center, int y_top,
		int width, int height, int min_pix);
       
        int count_nonzero(void) const { return (int)non_zero.size(); }
        const Point get_bottom_left_point(void) const { return Point(x_left, y_bottom);}
        const Point get_top_right_point(void) const { return Point(x_right, y_top); }
        void get_indices(Mat&, Mat&) const;
        Window get_next_window(Mat&);

};

#endif