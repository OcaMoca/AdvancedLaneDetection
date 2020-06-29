#ifndef __CAMERACALIBRATION_H__
#define __CAMERACALIBRATION_H__

#include <vector>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/calib3d/calib3d_c.h>

using namespace std;
using namespace cv;

class CameraCalibration
{
    private:
        vector<vector<Point3f>> object_points;
        vector<vector<Point2f>> image_points;

        Mat camera_matrix;
        Mat dist_coefs;
        Mat map1, map2;

    public: 
        CameraCalibration() {}
        bool add_chessboard_points(vector<string>&, Size&);
        double calibration(Size&);
        void undistort_image(const Mat&, Mat&);

};

#endif