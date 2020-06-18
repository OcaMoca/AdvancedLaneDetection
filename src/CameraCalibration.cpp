#include "../include/CameraCalibration.hpp"

bool CameraCalibration::add_chessboard_points(vector<string>& camera_images, cv::Size& board_size)
{
    vector<Point3f> object_corners;
    vector<Point2f> image_corners;
    bool found;
    bool success;

    for(int i = 0; i < board_size.height; i++)
        for(int j = 0; j < board_size.width; j++)
            object_corners.push_back(Point3f(i, j, 0.0f));

    Mat image;
    Mat gray;

    for(int i = 0; i < camera_images.size(); i++)
    {
        image = imread(camera_images[i]);
        cvtColor(image, gray, COLOR_BGR2GRAY);

        found = findChessboardCorners(gray, board_size, image_corners, CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);

        if (found)
        {
            cornerSubPix(gray, image_corners, Size(5,5), Size(-1,-1), TermCriteria(TermCriteria::MAX_ITER + TermCriteria::EPS, 30, 0.1));
        }

        if (image_corners.size() == board_size.area())
        {
            image_points.push_back(image_corners);
            object_points.push_back(object_corners);
            success = true;
        }
    }

    return success;
}

double CameraCalibration::calibration(cv::Size& image_size)
{
    vector<Mat> rvecs, tvecs;
    double ret;

    ret = cv::calibrateCamera(object_points, image_points, image_size, camera_matrix, dist_coefs, rvecs, tvecs, CALIB_RATIONAL_MODEL); 

    return ret;
}

void CameraCalibration::undistort_image(const Mat& image, Mat& undistorted)
{
    undistort(image, undistorted, camera_matrix, dist_coefs);
    return;
}

void CameraCalibration::get_camera_matrix(Mat& camera_matrix)
{
    camera_matrix = this->camera_matrix;
    return;
}
void CameraCalibration::get_dist_coefs(Mat& dist_coefs)
{
    dist_coefs = this->dist_coefs;
    return;
}
