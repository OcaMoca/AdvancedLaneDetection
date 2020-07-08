#include "HMI.hpp"

HMI::HMI(){}

void HMI::get_inverse_points(const vector<float>& plot_y, const vector<float>& left_fit_x, const vector<float>& right_fit_x, Mat& color_warp_output, vector<Point2f>& pts_left, vector<Point2f>& pts_right, vector<Point>& pts)
{

  for (int i = 0; i < (int)plot_y.size(); i++)
  {
    pts_left.push_back(Point2f((float)left_fit_x[i], (float)plot_y[i])); 
    pts_right.push_back(Point2f((float)right_fit_x[i], (float)plot_y[i]));
  }

  pts.reserve(2 * (int)plot_y.size());
  pts.insert(pts.end(), pts_left.begin(), pts_left.end());
  pts.insert(pts.end(), pts_right.rbegin(), pts_right.rend());

  vector<vector<Point>> ptsarray{ pts };

  fillPoly(color_warp_output, ptsarray, Scalar(0, 255, 0));

  return;
}

void HMI::final_perspective(const Mat& color_warped, const Mat& original_image, vector<Point2f>& original_roi, vector<Point2f>& warped_roi, Mat& final_perspective_output)
{
  Mat new_warp;
  Mat Minv(2,4,CV_32FC2);
  Minv = getPerspectiveTransform(warped_roi, original_roi);

  warpPerspective(color_warped, new_warp, Minv, color_warped.size(),INTER_LINEAR);
  addWeighted(original_image, 1, new_warp, 0.3, 0, final_perspective_output);

  return;
}

void HMI::show_values(float R_curve_avg, float car_offset, int car_speed, Mat& output_frame)
{
    char radius_text[100];
    char speed_text[100];
    char car_offset_text[100];
    sprintf(radius_text, "Radius of curvature: %f m", R_curve_avg);
    sprintf(speed_text, "Car speed: %d km/h", car_speed);
    sprintf(car_offset_text, "Car offset: %f m", car_offset);

    cv::putText(output_frame, radius_text, Point2f(50,50), FONT_HERSHEY_SIMPLEX, 1, Scalar(0,0,0,0));

    if(car_speed > 120)
    {
        cv::putText(output_frame, speed_text, Point2f(50,100), FONT_HERSHEY_SIMPLEX, 1, Scalar(0,0,255,0));
    } else {
        cv::putText(output_frame, speed_text, Point2f(50,100), FONT_HERSHEY_SIMPLEX, 1, Scalar(0,0,0,0));
    }

    cv::putText(output_frame, car_offset_text, Point2f(50,150), FONT_HERSHEY_SIMPLEX, 1, Scalar(0,0,0,0));
}

void HMI::show_steering_wheel(Mat& frame, Mat& transparent, int offset)
{
    Mat mask;
    vector<Mat> channels;

    split(transparent, channels); 
    Mat rgb[3] = { channels[0],channels[1],channels[2] };
    mask = channels[3]; 
    merge(rgb, 3, transparent);
    transparent.copyTo(frame.rowRange(frame.rows - transparent.rows - offset, frame.rows - offset).colRange(offset, transparent.cols + offset), mask);
}


